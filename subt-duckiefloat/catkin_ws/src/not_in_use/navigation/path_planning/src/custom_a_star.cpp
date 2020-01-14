#include <iostream>
#include <math.h>
#include <vector>
#include <map>
#include <queue>
#include <time.h>
#include <algorithm>
#include <array>
#include "custom_a_star.h"
//ros library
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Bool.h"
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <visualization_msgs/Marker.h>

using namespace std;
class PathPlanning{
private:
    string node_name;
    double update_time;
    vector<array<double, 2> >pose_array;

    AStar a_star;

    nav_msgs::OccupancyGrid map;

    ros::NodeHandle nh;
    ros::Publisher  pub_path;

    ros::Subscriber sub_map;
    ros::Timer timer;

public:
    PathPlanning(ros::NodeHandle&);
    void cbMap(const nav_msgs::OccupancyGrid&);
    void Planning(const ros::TimerEvent&);
};
PathPlanning::PathPlanning(ros::NodeHandle& n){
    nh = n;
    node_name = ros::this_node::getName();
    update_time = 0.5;
    nh.getParam("update_time", update_time);

    ROS_INFO("[%s] Initializing ", node_name.c_str());

    //Publisher
    pub_path = nh.advertise<nav_msgs::Path>("/global_path", 1);

    timer = nh.createTimer(ros::Duration(update_time), &PathPlanning::Planning, this);

    //Subscriber
    sub_map = nh.subscribe("/map", 1, &PathPlanning::cbMap, this);

}
void PathPlanning::Planning(const ros::TimerEvent& event){
    /*
    ***************************************************************
        Planning timer
    ***************************************************************
    */
    //cout << "Start planning" << endl; 
    clock_t t_start = clock();
    nav_msgs::Path  global_path;
    global_path.header.frame_id = map.header.frame_id;
    global_path.header.stamp = ros::Time::now();

	double local_cost = 0;
	double vehicle_size = 0;
	geometry_msgs::Pose start, goal;
	start.position.x = 0;
	start.position.y = 0;
	goal.position.x = 10;
	goal.position.y = 0;
    vector<Node> path = a_star.Planning(map, start, goal, vehicle_size, local_cost);
    for (int i = 0 ; i < path.size() ; i++){
        Node tmp = path[i];
        double x = tmp.x*map.info.resolution+map.info.origin.position.x;
        double y = tmp.y*map.info.resolution+map.info.origin.position.y;

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id =  map.header.frame_id;
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0;
        global_path.poses.push_back(pose);

    }
    pub_path.publish(global_path);

    clock_t t_end = clock();
    cout << "A start planning time taken = " << (t_end-t_start)/(double)(CLOCKS_PER_SEC) << endl;
}

void PathPlanning::cbMap(const nav_msgs::OccupancyGrid& msg_map){
    map = msg_map;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PathPlanning");
    ros::NodeHandle nh("~");
    PathPlanning pp(nh);
    
    ros::spin();
    return 0;
}
