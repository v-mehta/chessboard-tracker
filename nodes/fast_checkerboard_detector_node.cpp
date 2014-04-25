/**
 * author: Vaibhav Mehta <vaibhavmehta.in@gmail.com>
 */
 
#include <ros/ros.h>

#include <fast_checkerboard_detector/fast_checkerboard_detector.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fast_checkerboard_detector");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  fast_checkerboard_detector::FastCheckerboardDetector node(nh, private_nh);

  ros::spin();

  return 0;
}
