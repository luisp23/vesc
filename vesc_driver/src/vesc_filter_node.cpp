#include <ros/ros.h>

#include "vesc_filter.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vesc_filter_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  vesc_filter::VescFilter vesc_filter(nh, private_nh);

  ros::spin();

  return 0;
}