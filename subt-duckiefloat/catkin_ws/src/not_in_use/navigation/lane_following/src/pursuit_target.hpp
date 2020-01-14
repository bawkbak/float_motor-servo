#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <lane_following/target_pose.h>
#include "astar.hpp"
#include "helper.hpp"

/*
 *   Pursuit target pose from human assign either from console (service call)
 *   or from 2D nav goal in RViz
 *
 *   Author: Sean Lu
 *   Last edited: 4/8, 2019
 *   Subscribe topic:
 *     ~occupancy_grid (nav_msgs::OccupancyGrid)
 *     /move_base_simple/goal (geometry_msgs::PoseStamped)
 *   Publish topic:
 *     ~planned_path (nav_msgs::Path)
 *     ~marker (visualization_msgs::Marker)
 *   Service:
 *     ~setTarget (lane_following::target_pose)
 *   Parameters:
 *     ~verbose (bool) Whether if publish marker, publish if set to true
 */
 
bool ASTAR_VERBOSE = true; // Print astar information
bool ASTAR_SHOWTIME = true; // Print astar palnning time

class PathPlanning{
 private:
  bool state; // 1 if new request in, 0 otherwise
  bool verbose; // if true, publish marker for visualization
  AStar::AStar planner;
  Eigen::MatrixXd map;
  // ROS
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_grid;
  ros::Subscriber sub_rviz;
  ros::Publisher pub_path;
  ros::Publisher pub_marker;
  ros::ServiceServer srv_setTarget;
  ros::Timer timer; // Timer to publish marker
  // Pose-related variables
  geometry_msgs::PoseStamped target, target_ps;
  nav_msgs::Path planned_path;
  nav_msgs::MapMetaData mapMetaData;
  AStar::NODE_LIST res;
  // Visualization
  visualization_msgs::Marker marker;
  // Topic and service strings
  const std::string GRID_TOPIC = "occupancy_grid";
  const std::string VIZ_TOPIC = "/move_base_simple/goal";
  const std::string PATH_TOPIC = "planned_path";
  const std::string POSE_TOPIC = "target_pose";
  const std::string SRV_NAME = "setTarget";
  const std::string MAP_FRAME = "odom";
  std::string node_name;
  tf::Transform mat;
  // Callback for sub_grid
  void cbMap(const nav_msgs::OccupancyGrid msg){
    // Check if there are target request
    mapMetaData = msg.info;
    if(state){
      planned_path.poses.clear(); // Clear pose in path
      marker.points.clear(); marker.colors.clear();
      res.clear();
      // Convert occupancy grid to Eigen::MatrixXd
      map = Eigen::MatrixXd(msg.info.height, msg.info.width);
      grid_to_matrix(msg, map);
      int x, y;
      if(!pose_to_idx(msg.info, 0, 0, x, y)) {ROS_WARN("Start pose out of map, ignoring..."); state=0; return;} 
      AStar::Node start(y, x, NULL); // Have to change xy order
      geometry_msgs::PoseStamped ps = target;
      convertFrame(mat, ps);
      std::cout << ps.pose.position.x << " " << ps.pose.position.y << "\n";
      if(!pose_to_idx(msg.info, ps.pose.position.x, ps.pose.position.y, x, y))
      {ROS_WARN("Target pose out of map, ignoring..."); state = 0; return;}
      AStar::Node end(y, x, NULL); // Have to change xy order
      ROS_INFO("A star planner received new request: ");
      ROS_INFO("\t Start node: (%f, %f)", start.get_x(), start.get_y());
      ROS_INFO("\t End node:   (%f, %f)", end.get_x(), end.get_y());
      ROS_INFO("Start processing...");
      planner.initial(start, end, map);
      if(!planner.plan(res)){ROS_WARN("Fail to find path, ignoring..."); state = false; return;}
      planned_path.header.frame_id = MAP_FRAME;
      for(AStar::NODE_LIST::iterator it=res.begin(); it!=res.end(); ++it){
        geometry_msgs::PoseStamped ps = indice_to_pose(msg.info, it->get_x(), it->get_y());
        convertFrame(mat.inverse(), ps); // Have to convert back to fixed frame
        planned_path.poses.push_back(ps); if(verbose){
          geometry_msgs::Point p = indice_to_point(msg.info, it->get_x(), it->get_y());
          convertFrame(mat.inverse(), p); // Have to convert back to fixed frame
          marker.points.push_back(p);
          std_msgs::ColorRGBA c; c.r = 1.0f; c.a = 1.0f;
          marker.colors.push_back(c);
        }
      } // End for
      pub_marker.publish(marker);
      geometry_msgs::Point p = marker.points.back();
      ROS_INFO("Target point in [/odom] frame: (%f, %f)", p.x, p.y);
      pub_path.publish(planned_path); state = 0;
      ROS_INFO("Path with length %d published", (int)planned_path.poses.size()); ;
    } // End if
  } 
  // Callback for sub_rviz
  void cbRviz(const geometry_msgs::PoseStamped msg){
    if(state) return; // Still processing, ignore...
    tf::TransformListener listener;
    tf::StampedTransform transform;
    try{
      listener.waitForTransform("base_link", MAP_FRAME, ros::Time(0), ros::Duration(1.0));
      listener.lookupTransform("base_link", MAP_FRAME, ros::Time(0), transform);
    } catch(tf::TransformException ex) {ROS_ERROR("%s", ex.what());}
    mat = tf::Transform(transform.getRotation(), transform.getOrigin());
    target = msg;
    state = true;
    ROS_INFO("[%s] Get new target (x, y, th) = (%f, %f) in frame %s.", node_name.c_str(), 
            msg.pose.position.x, msg.pose.position.y, msg.header.frame_id.c_str());
    if(msg.header.frame_id == "odom"){
    }
    else {
      ROS_WARN("Please use fixed frame: %s", MAP_FRAME.c_str()); return;
    }
    std::cout << mat.getOrigin().getX() << " " << mat.getOrigin().getY() << " " << mat.getOrigin().getZ() << "\n";
    target_ps = target;
    ROS_INFO("[%s] W.r.t. fixed frame [/odom]: (%f, %f)", node_name.c_str(), target_ps.pose.position.x, target_ps.pose.position.y);
  }
  // Timer callback to publish target marker
  void cbTimer(const ros::TimerEvent& event){
    tf::TransformListener listener;
    tf::StampedTransform transform;
    try{
      listener.waitForTransform("base_link", MAP_FRAME, ros::Time(0), ros::Duration(1.0));
      listener.lookupTransform("base_link", MAP_FRAME, ros::Time(0), transform);
    } catch(tf::TransformException ex) {ROS_ERROR("%s", ex.what());}
    mat = tf::Transform(transform.getRotation(), transform.getOrigin());
    if(verbose) {
      pub_marker.publish(marker);
    } // End if
  }
 public:
  // Constructor
  PathPlanning(ros::NodeHandle nh, ros::NodeHandle pnh): nh_(nh), pnh_(pnh){
    node_name = ros::this_node::getName();
    state = false;
    sub_grid = pnh_.subscribe(GRID_TOPIC, 1, &PathPlanning::cbMap, this);
    sub_rviz = pnh_.subscribe(VIZ_TOPIC, 1, &PathPlanning::cbRviz, this);
    pub_path = pnh_.advertise<nav_msgs::Path>(PATH_TOPIC, 1);
    timer = pnh_.createTimer(ros::Duration(0.1), &PathPlanning::cbTimer, this);
    srv_setTarget = pnh_.advertiseService(SRV_NAME, &PathPlanning::cbRequest, this);
    if(!pnh_.getParam("verbose", verbose)){
      ROS_INFO("verbose set to true"); verbose = true;
    } if(verbose){
      pub_marker = pnh_.advertise<visualization_msgs::Marker>("marker", 1);
      // Initial marker
      marker.header.frame_id = MAP_FRAME;
      marker.type = visualization_msgs::Marker::POINTS;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.1; marker.scale.y = 0.1; marker.scale.z = 0.1;
      marker.color.r = 1.0f; marker.color.g = 0.0f; marker.color.b = 0.0f; marker.color.a = 1.0f;
    } // End if
  }
  // Callback for srv_setTarget
  bool cbRequest(lane_following::target_pose::Request& req, lane_following::target_pose::Response& res){
    if(state) return true; // Still processing, ignore...
    target = req.target; 
    ROS_INFO("[%s] Get new target (x, y, th) = (%f, %f, %f).", node_name.c_str(), 
            target.pose.position.x, target.pose.position.y, 
            2*atan2(target.pose.orientation.z, target.pose.orientation.w)); 
    state = true; return true;
  }
};
