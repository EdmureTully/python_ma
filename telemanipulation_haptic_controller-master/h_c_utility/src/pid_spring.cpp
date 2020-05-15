#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pid_spring");
  ros::NodeHandle nh;

  ROS_INFO("Hello world!");
}
