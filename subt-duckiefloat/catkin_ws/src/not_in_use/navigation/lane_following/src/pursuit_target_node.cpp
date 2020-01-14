#include "pursuit_target.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "subt_lane_following_node");
  ros::NodeHandle nh, pnh("~");
  PathPlanning pp(nh, pnh);
  while(ros::ok()) ros::spinOnce();
  return 0;
}
