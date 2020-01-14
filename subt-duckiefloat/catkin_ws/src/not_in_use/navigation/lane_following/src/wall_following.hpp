#include <cstdlib> // std::srand, std::rand
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include "astar.hpp"
#include "helper.hpp"

/*
 *   Wall following autonomously
 *
 *   Author: Sean Lu
 *   Last edited: 4/8, 2019
 *   Subscribe topic:
 *     ~occupancy_grid (nav_msgs::OccupancyGrid)
 *   Publish topic:
 *     ~planned_path (nav_msgs::Path)
 *     ~marker (visualization_msgs::Marker)
 *   Parameters:
 *     ~verbose (bool) Whether if publish marker, publish if set to true
 *     ~cml_verbose (bool) Whether bash will output verbose target index of searching
 *     ~left (bool) Near left or right, left if set to true
 *     ~radius (double) Radius of circle to search walkable pose
 *     ~timer_execution(int): 
 */
 
bool ASTAR_VERBOSE = false;
bool ASTAR_SHOWTIME = false;

class WallFollower{
 private:
  // Parameters
  const static int NUM = 25; // NUmber of search for target
  const static int STUCK_MAX = 10; // Stuck too long
  int timer_execution; // Execution timer, from parameter server
  int count; // Execution counter
  int stuck_counter; // Counter to record stuck times
  bool use_odom;
  bool verbose; // Whether publish marker
  bool cml_verbose; // Whether bash will output verbose target index of searching
  bool left; // Follow left wall or right, true if left
  double radius; // Look ahead distance for target
  double angle[NUM];
  const std::string MAP_FRAME = "odom";
  const std::string ROBOT_FRAME = "base_link";

  // ROS
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_grid;
  ros::Publisher pub_path;
  ros::Publisher pub_marker;
  ros::Timer timer; // Timer to publish marker
  nav_msgs::OccupancyGrid grid;
  nav_msgs::MapMetaData mapMetaData;
  AStar::AStar planner;
  AStar::NODE_LIST res;
  Eigen::MatrixXd map;
  tf::Transform mat;
  visualization_msgs::Marker marker;
  // Callback for sub_grid
  void cbMap(nav_msgs::OccupancyGrid msg){
    res.clear();
    nav_msgs::Path planned_path;
    mapMetaData = msg.info;
    map = Eigen::MatrixXd(mapMetaData.height, mapMetaData.width);
    if(count!=timer_execution) {++count; return;}
    else {
      count=0; marker.points.clear(); marker.colors.clear(); 
      if(cml_verbose) ROS_INFO("Counter set to 0."); 
    }
    grid = msg;
    grid_to_matrix(msg, map);
    res = find_target();
    if(res.empty()){
      // Not walkable
      ++stuck_counter;
      ROS_WARN("Not walkable for every searching position, stuck_counter is now: %d", stuck_counter);
      if(stuck_counter >= STUCK_MAX){
        // Random search
        ROS_INFO("Try random search to escape stuck..."); int while_count = 0;
        srand(time(NULL));
        while(true){
          ++while_count;
          float radius_mul = ((rand() % 11)+10)/10.0;
          int angle_idx = (rand() % NUM);
          int idx_x, idx_y;
          pose_to_idx(mapMetaData, 0, 0, idx_x, idx_y); AStar::Node start(idx_y, idx_x, NULL); // Have to change xy order
          double x = radius * radius_mul * cos(angle[angle_idx]*M_PI/180.0),
                 y = radius * radius_mul * sin(angle[angle_idx]*M_PI/180.0);
          if(!pose_to_idx(mapMetaData, x, y, idx_x, idx_y)) continue;
          AStar::Node end(idx_y, idx_x, NULL); // Have to change xy order
          planner.initial(start, end, map);
          if(planner.plan(res)) {ROS_INFO("Break stuck!"); break;}
        } // End while
        stuck_counter = 0;
      } // End if
      if(res.empty()) return; // No publish if empty list
    } // End if
    else stuck_counter = 0;
    for(AStar::NODE_LIST::iterator it=res.begin(); it!=res.end(); ++it){
      geometry_msgs::PoseStamped ps = indice_to_pose(mapMetaData, it->get_x(), it->get_y());
      // If use_odom is true, we have to convert the pose back to odom frame
      if(use_odom) convertFrame(mat.inverse(), ps); // Have to convert back to fixed frame
      planned_path.poses.push_back(ps);  
      if(verbose){
        geometry_msgs::Point p = indice_to_point(mapMetaData, it->get_x(), it->get_y());
        if(use_odom) convertFrame(mat.inverse(), p); // Have to convert back to fixed frame
        marker.points.push_back(p);
        std_msgs::ColorRGBA c; c.r = 1.0f; c.a = 1.0f; marker.colors.push_back(c);
      }
    } planned_path.header.frame_id = (use_odom==true? MAP_FRAME: ROBOT_FRAME);
    pub_path.publish(planned_path);
    if(verbose) pub_marker.publish(marker);
  }
  // Initial marker after constructed
  void initial_marker(visualization_msgs::Marker& marker){
    marker.header.frame_id = (use_odom==true? MAP_FRAME: ROBOT_FRAME); // Fixed frame, i.e., odom
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1; marker.scale.y = 0.1; marker.scale.z = 0.1; 
  }
  // Try to find walkable target in the predefined searching points
  AStar::NODE_LIST find_target(void){
    AStar::NODE_LIST nl;
    //AStar::Node start(0, 0, NULL);
    for(int i=0; i<NUM; ++i){
      int idx_x, idx_y;
      pose_to_idx(mapMetaData, 0, 0, idx_x, idx_y); AStar::Node start(idx_y, idx_x, NULL); // Have to change xy order
      double x = radius * cos(angle[i]*M_PI/180.0), 
             y = radius * sin(angle[i]*M_PI/180.0);
      if(!pose_to_idx(mapMetaData, x, y, idx_x, idx_y)) continue;
      AStar::Node end(idx_y, idx_x, NULL); // Have to change xy order
      planner.initial(start, end, map);
      if(planner.plan(nl)){
        if(cml_verbose)
          ROS_INFO("[%s] Find walable target at index: %d, set pose to (%f, %f)", ros::this_node::getName().c_str(), i+1, x, y);
        break; // Find walkable path, exit for
      }
    }
    return nl;
  }
  void cbTimer(const ros::TimerEvent& event){
    if(use_odom){
      tf::TransformListener listener;
      tf::StampedTransform transform;
      try{
        listener.waitForTransform(ROBOT_FRAME, MAP_FRAME, ros::Time(0), ros::Duration(1.0));
        listener.lookupTransform(ROBOT_FRAME, MAP_FRAME, ros::Time(0), transform);
      } catch(tf::TransformException ex) {ROS_ERROR("%s", ex.what());}
      mat = tf::Transform(transform.getRotation(), transform.getOrigin());
    }
    if(verbose) {
      pub_marker.publish(marker);
    } // End if
  }
 public:
  WallFollower(ros::NodeHandle nh, ros::NodeHandle pnh): nh_(nh), pnh_(pnh), count(0){
   stuck_counter = 0;
   // Subscriber and publisher
   sub_grid = pnh_.subscribe("occupancy_grid", 1, &WallFollower::cbMap, this);
   pub_path = pnh_.advertise<nav_msgs::Path>("planned_path", 1);
   timer = pnh_.createTimer(ros::Duration(0.1), &WallFollower::cbTimer, this);
   // Get parameters
   if(!nh_.getParam("use_odom", use_odom)) {use_odom=true; ROS_INFO("[%s] use_odom set to true", ros::this_node::getName().c_str());}
   if(!pnh_.getParam("verbose", verbose)) {verbose=true; ROS_INFO("[%s] verbose set to true", ros::this_node::getName().c_str());}
   if(!pnh_.getParam("cml_verbose", cml_verbose)) {cml_verbose=true; ROS_INFO("[%s] cml_verbose set to true", ros::this_node::getName().c_str());}
   if(!pnh_.getParam("left", left)) {left=true; ROS_INFO("[%s] left set to true", ros::this_node::getName().c_str());}
   if(!pnh_.getParam("radius", radius)) {radius=3.; ROS_INFO("[%s] radius set to %f", ros::this_node::getName().c_str(), radius);}
   if(!pnh_.getParam("timer_execution", timer_execution)) {
   timer_execution=5; ROS_INFO("[%s] timer_execution set to %d", ros::this_node::getName().c_str(), timer_execution);}
   // Initial variables after get parameters
   if(verbose) {pub_marker=pnh_.advertise<visualization_msgs::Marker>("marker", 1); initial_marker(marker);}
   if(left) {for(int i=0; i<NUM; ++i) angle[i]=15-i*30./(NUM-1);} // Degree [90, -180]
   else {for(int i=0; i<NUM; ++i) angle[i]=-15+i*30./(NUM-1);} // Degree [-90, 180]
   for(int i=0; i<NUM; ++i)
      printf("%.2f, ", angle[i]);
    printf("\n");
  }
};
