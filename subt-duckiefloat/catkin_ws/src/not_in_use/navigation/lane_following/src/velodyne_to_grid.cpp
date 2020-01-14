/**********************************
Author: David Chen
Revised by Sean Lu
Date: 2019/03/30 
Last update: 2019/04/01
Point Cloud Obstacle Detection
Subscribe: 
  /velodyne_points      (sensor_msgs/PointCloud2)
Publish:
  /pcl_preprocess       (sensor_msgs/PointCloud2)
  /local_map            (nav_msgs/OccupancyGrid)
***********************************/ 
//ROS Lib
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
//PCL lib
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/kdtree/kdtree_flann.h>
//TF lib
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;

class Obstacle_Detection{
private:
  string node_name;
  string robot_frame;

  // Only point cloud in these range will be take into account
  double range_min;
  double range_max;
  double angle_min;
  double angle_max;
  double height_max;
  double height_min;

  // Range of robot itself
  double robot_x_max;
  double robot_x_min;
  double robot_y_max;
  double robot_y_min;
  double robot_z_max;
  double robot_z_min;

  // Define map
  int map_size; // Square assumed
  float map_resolution;
  nav_msgs::OccupancyGrid occupancygrid;
  int obs_size;
  int dilating_size;
  int map_center;

  // Define map shifting
  int map_shift_x;
  int map_shift_y;

  // Define wall
  int wall_thick;
  int wall_length;
  int wall_x_dis;
  int wall_x;
  int wall_y;

  ros::NodeHandle nh, pnh;
  ros::Subscriber sub_cloud;
  ros::Subscriber sub_point;
  ros::Publisher  pub_cloud;
  ros::Publisher  pub_points;
  ros::Publisher  pub_map;

  ros::ServiceServer service;

public:
  Obstacle_Detection(ros::NodeHandle&, ros::NodeHandle&);
  void cbCloud(const sensor_msgs::PointCloud2ConstPtr&);
  bool obstacle_srv(std_srvs::Trigger::Request&, std_srvs::Trigger::Response&);
  void pcl_preprocess(const PointCloudXYZRGB::Ptr, PointCloudXYZRGB::Ptr);
  void mapping(const PointCloudXYZRGB::Ptr);
  void map2occupancygrid(float&, float&);
};

Obstacle_Detection::Obstacle_Detection(ros::NodeHandle &n, ros::NodeHandle &pn):
                                       nh(n), pnh(pn){
  node_name = ros::this_node::getName();

  //Read yaml file and set costumes parameters
  if(!pnh.getParam("range_min", range_min)) range_min=0.0;
  if(!pnh.getParam("range_max", range_max)) range_max=30.0;
  if(!pnh.getParam("angle_min", angle_min)) angle_min = -180.0;
  if(!pnh.getParam("angle_max", angle_max)) angle_max = 180.0;
  if(!pnh.getParam("height_min", height_min)) height_min = -0.3;
  if(!pnh.getParam("height_max", height_max)) height_max = 0.5;
  if(!pnh.getParam("robot_x_max", robot_x_max)) robot_x_max=0.05;
  if(!pnh.getParam("robot_x_min", robot_x_min)) robot_x_min=-0.6;
  if(!pnh.getParam("robot_y_max", robot_y_max)) robot_y_max=0.1;
  if(!pnh.getParam("robot_y_min", robot_y_min)) robot_y_min=-0.1;
  if(!pnh.getParam("robot_z_max", robot_z_max)) robot_z_max=1.;
  if(!pnh.getParam("robot_z_min", robot_z_min)) robot_z_min=-1.5;
  if(!pnh.getParam("robot_frame", robot_frame)) robot_frame="/base_link";
  if(!pnh.getParam("map_resolution", map_resolution)) map_resolution=0.3;
  if(!pnh.getParam("obs_size", obs_size)) obs_size=2;
  if(!pnh.getParam("dilating_size", dilating_size)) dilating_size=4;
  if(!pnh.getParam("map_size", map_size)) map_size=200;
  // Parameter information
  ROS_INFO("[%s] Initializing ", node_name.c_str());
  ROS_INFO("[%s] Param [range_max] = %f, [range_min] = %f", node_name.c_str(), range_max, range_min);
  ROS_INFO("[%s] Param [angle_max] = %f, [angle_min] = %f", node_name.c_str(), angle_max, angle_min);
  ROS_INFO("[%s] Param [height_max] = %f, [height_min] = %f", node_name.c_str(), height_max, height_min);
  ROS_INFO("[%s] Param [robot_x_max] = %f, [robot_x_min] = %f", node_name.c_str(), robot_x_max, robot_x_min);
  ROS_INFO("[%s] Param [robot_y_max] = %f, [robot_y_min] = %f", node_name.c_str(), robot_y_max, robot_y_min);
  ROS_INFO("[%s] Param [robot_z_max] = %f, [robot_z_min] = %f", node_name.c_str(), robot_z_max, robot_z_min);
  ROS_INFO("[%s] Param [robot_frame] = %s, [map_resolution] = %f", node_name.c_str(), robot_frame.c_str(), map_resolution);
  ROS_INFO("[%s] Param [obs_size] = %d, [dilating_size] = %d", node_name.c_str(), obs_size, dilating_size);
  ROS_INFO("[%s] Param [map_size] = %d", node_name.c_str(), map_size);
  // Set map meta data
  occupancygrid.header.frame_id = robot_frame;
  occupancygrid.info.resolution = map_resolution;
  occupancygrid.info.width = map_size;
  occupancygrid.info.height = map_size;
  occupancygrid.info.origin.position.x = -map_size*map_resolution/2.;
  occupancygrid.info.origin.position.y = -map_size*map_resolution/2.;
  // Service
  service = pnh.advertiseService("obstacle_srv", &Obstacle_Detection::obstacle_srv, this);
  // Publisher
  pub_cloud = pnh.advertise<sensor_msgs::PointCloud2> ("pcl_preprocess", 1);
  pub_map = pnh.advertise<nav_msgs::OccupancyGrid> ("local_map", 1);
  // Subscriber
  sub_cloud = pnh.subscribe("/camera/depth_registered/points", 1, &Obstacle_Detection::cbCloud, this);
}
// Service callback template
bool Obstacle_Detection::obstacle_srv(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
  res.success = 1;
  res.message = "Call obstacle detection service";
  cout << "Call detection service" << endl;
  return true;
}

void Obstacle_Detection::cbCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
  // transfer ros msg to point cloud
  PointCloudXYZRGB::Ptr cloud_in_rgb(new PointCloudXYZRGB()); 
  PointCloudXYZ::Ptr cloud_in(new PointCloudXYZ()); 
  PointCloudXYZRGB::Ptr cloud_out(new PointCloudXYZRGB());
  pcl::fromROSMsg (*cloud_msg, *cloud_in); 

  copyPointCloud(*cloud_in, *cloud_in_rgb); 

  tf::Quaternion q;
  q.setRPY(-M_PI/2, 0, -M_PI/2);
  tf::Transform tf1;
  tf1.setRotation(q);
  pcl_ros::transformPointCloud(*cloud_in_rgb, *cloud_in_rgb, tf1);

  
  clock_t time = clock();
  // Remove out of range points and robot points
  pcl_preprocess(cloud_in_rgb, cloud_out); //printf("PRE: %f\n", (double)(clock()-time)/CLOCKS_PER_SEC); 
  time = clock();
  mapping(cloud_out); //printf("Map: %f\n", (double)(clock()-time)/CLOCKS_PER_SEC);
  // Publish point cloud
  sensor_msgs::PointCloud2 pcl_output;
  pcl::toROSMsg(*cloud_out, pcl_output);
  pcl_output.header = cloud_msg->header;
  pcl_output.header.frame_id = robot_frame;
  pub_cloud.publish(pcl_output);
}

void Obstacle_Detection::pcl_preprocess(const PointCloudXYZRGB::Ptr cloud_in, PointCloudXYZRGB::Ptr cloud_out){
  clock_t time = clock();
  // Remove NaN point
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, indices);
  //printf("Remove nan: %f\n", (double)(clock()-time)/CLOCKS_PER_SEC); time = clock();
  // Range filter
  float dis2, angle = 0;
  int num = 0; // Number of points counter
  // Conditional or to define range that out side the robot
  // x
  pcl::ConditionOr<pcl::PointXYZRGB>::Ptr range_cond(new pcl::ConditionOr<pcl::PointXYZRGB>());
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new
    pcl::FieldComparison<pcl::PointXYZRGB>("x", pcl::ComparisonOps::GT, robot_x_max)));
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new
    pcl::FieldComparison<pcl::PointXYZRGB>("x", pcl::ComparisonOps::LT, robot_x_min)));
  // y
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new
    pcl::FieldComparison<pcl::PointXYZRGB>("y", pcl::ComparisonOps::GT, robot_y_max)));
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new
    pcl::FieldComparison<pcl::PointXYZRGB>("y", pcl::ComparisonOps::LT, robot_y_min)));  
  // z
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new
    pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::LT, robot_z_max)));  
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new
    pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::LT, robot_z_min)));  
  // Build conditional removal filter
  pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
  condrem.setCondition (range_cond);
  condrem.setInputCloud (cloud_in);
  // Then apply it
  condrem.filter (*cloud_in);  
  // Consider only pz in [height_min, height_max]  
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(cloud_in);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(height_min, height_max);
  pass.filter(*cloud_in);
  //printf("Range filter: %f\n", (double)(clock()-time)/CLOCKS_PER_SEC); time = clock();
  // Since we need to calculate angle of each point, use iteration to traverse through all points
  // This is not pretty computation heavy because we have filtered out lots points
  
  for(PointCloudXYZRGB::iterator it=cloud_in->begin(); it != cloud_in->end();it++){
    dis2 = it->x * it->x + it->y * it->y; // Only consider XY distance square
    angle = atan2f(it->y, it->x);
    angle = angle * 180 / M_PI; // Angle of the point in robot coordinate space
    // Filter out if the point is in or out of the range we define
    bool is_in_range =  dis2 >= range_min*range_min  &&  dis2 <= range_max*range_max && 
                        angle >= angle_min && angle <= angle_max;
    if(is_in_range){cloud_out->points.push_back(*it); ++num;}
  }
  cloud_out->width = num;
  cloud_out->height = 1;
  cloud_out->points.resize(num);
  
  /*
  // Project all inliers to XY-plane
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
  coefficients->values.resize(4);
  coefficients->values[0] = coefficients->values[1] = coefficients->values[3] = 0.0;
  coefficients->values[2] = 1.0;
  pcl::ProjectInliers<pcl::PointXYZRGB> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);
  proj.setInputCloud(cloud_in);
  proj.setModelCoefficients(coefficients);
  proj.filter(*cloud_in);
  // Build KD tree
  pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
  kdtree.setInputCloud(cloud_in);
  pcl::PointXYZRGB search_point; // (0, 0, 0)
  search_point.x = search_point.y = search_point.z = 0.0;
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquareDistance;
  float radius = range_max;
  if(kdtree.radiusSearch(search_point, radius, pointIdxRadiusSearch, pointRadiusSquareDistance) >0){
    for(size_t i=0; i<pointIdxRadiusSearch.size(); ++i){
      cloud_out->points.push_back(cloud_in->points[pointIdxRadiusSearch[i]]);
    }
  }
  */
  //printf("Distance filter: %f\n", (double)(clock()-time)/CLOCKS_PER_SEC); time = clock();
  // Point cloud noise filter
  /* Cost too much time!
  pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
  outrem.setInputCloud(cloud_out);
  outrem.setRadiusSearch(0.8);
  outrem.setMinNeighborsInRadius(2);
  outrem.filter(*cloud_out);
  printf("Noise filter: %f\n", (double)(clock()-time)/CLOCKS_PER_SEC);
  */
}

void Obstacle_Detection::mapping(const PointCloudXYZRGB::Ptr cloud){
  float x;
  float y;
  occupancygrid.data.clear();

  int map_array[map_size][map_size] = {0};

  for (PointCloudXYZRGB::iterator it = cloud->begin(); it != cloud->end() ; ++it){
    x = it->x;
    y = it->y;
    map2occupancygrid(x, y);
    
// Check bound
    if (int(y) < map_size && int(x) < map_size && int(y) >= 0 && int(x) >= 0){
      map_array[int(y)][int(x)] = 100;
    }
  }

  // Map dilating
  for (int j = 0; j < map_size; j++){
    for (int i = 0; i < map_size; i++){
      if (map_array[j][i] == 100){
        for (int m = -dilating_size; m < dilating_size + 1; m++){
          for (int n = -dilating_size; n < dilating_size + 1; n++){
            if (j+m<0 or j+m>=map_size or i+n<0 or i+n>=map_size) continue;
            if (map_array[j+m][i+n] != 100){
              if (m > obs_size || m < -obs_size || n > obs_size || n < -obs_size){
                if (map_array[j+m][i+n] != 80){
                  map_array[j+m][i+n] = 50;
                }
              }
              else{
                if(j+m<0 or j+m>=map_size or i+n<0 or i+n>=map_size) continue;
                map_array[j+m][i+n] = 80;
              }
            }
          }
        }
      }
    }
  }

  /*
  // Map shifting
  int map_temp[map_size][map_size] = {0};
  for (int i = 0; i < map_size; i++){
    for (int j = 0; j < map_size; j++){
	map_shift_x = -10;
	map_shift_y = -5;
        if (((map_size + map_shift_x) < 0) || ((map_size + map_shift_x) > map_size)){
	  map_shift_x = 0;
	}
	if (((map_size + map_shift_y) < 0) || ((map_size + map_shift_y) > map_size)){
	  map_shift_y = 0;
	}
        //map_array[i][j] = map_array[i + map_shift_x][j];
	map_temp[i - map_shift_x][j - map_shift_y] = map_array[i][j];
      }
  }
  for (int i = 0; i < (map_size); i++){
    for (int j = 0; j < (map_size); j++){
      map_array[i][j] = map_temp[i][j];
    }
  }
  //map_array = map_temp;
  */

  // Add wall
  wall_thick = 20;
  wall_length = 50;
  wall_x_dis = 8;
  map_center = map_size / 2;
  for (int j = 0; j < wall_length; j++){
    for (int i = 0; i < wall_thick; i++){
      wall_x = wall_x_dis + i;
      wall_y = j;
      map_array[map_center - wall_x][map_center + wall_y] = 100;
      map_array[map_center - wall_x][map_center - wall_y] = 100;
      map_array[map_center + wall_x][map_center + wall_y] = 100;
      map_array[map_center + wall_x][map_center - wall_y] = 100;
    }
  }
  for (int j = 0; j < map_size; j++){
    for (int i = 0; i < map_size; i++){
      occupancygrid.data.push_back(map_array[j][i]);
    }
  }
  pub_map.publish(occupancygrid);
  return;
}

void Obstacle_Detection::map2occupancygrid(float& x, float& y){
  x = int((x - occupancygrid.info.origin.position.x)/map_resolution);
  y = int((y - occupancygrid.info.origin.position.y)/map_resolution);
}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "velodyne_to_grid");
  ros::NodeHandle nh, pnh("~");
  Obstacle_Detection od(nh, pnh);
  ros::spin ();
  return 0;
}
