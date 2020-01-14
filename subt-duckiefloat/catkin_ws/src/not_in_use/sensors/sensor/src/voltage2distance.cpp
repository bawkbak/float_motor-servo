#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "sensor_msgs/Range.h"
#include "math.h"

sensor_msgs::Range height_value;

int message = 0;
float voltage_temp = 0;
float p = -1.301;
float m = 7e04;

void voltage_cb(const sensor_msgs::Range& voltage_value){
  height_value = voltage_value;
  voltage_temp = voltage_value.range;
//  ROS_INFO_STREAM("Reveived /infrared");
  height_value.range = powf(voltage_temp, p) * m;
  message = 1;
}

int main(int argc, char **argv){
  // Setup the subscriver and publisher
  ros::init(argc, argv, "get_height");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/infrared", 1000, voltage_cb);
  ros::Publisher  pub = nh.advertise<sensor_msgs::Range>("height", 1000);

  ros::Rate loop_rate(20);

  while(ros::ok()){
    if(message == 1){
      message = 0;
      pub.publish(height_value);
//      ROS_INFO_STREAM("Publish to /height");
    }
    else{
//      ROS_INFO_STREAM("No message revieved from infrared sensor");
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
