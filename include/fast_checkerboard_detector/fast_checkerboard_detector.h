/**
 * author: Vaibhav Mehta <vaibhavmehta.in@gmail.com>
 */

#ifndef FAST_CHECKERBOARD_DETECTOR_H_
#define FAST_CHECKERBOARD_DETECTOR_H_


#include <fstream>
#include <boost/foreach.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <eigen_conversions/eigen_msg.h>

namespace fast_checkerboard_detector
{

class FastCheckerboardDetector
{
private:
  image_transport::ImageTransport it_;
  image_transport::Subscriber img_subscriber_;

  ros::Subscriber camera_info_subscriber_;
  ros::Publisher camera_pose_publisher_;

  int grid_size_x, grid_size_y;
  double rect_size_x, rect_size_y;

  cv::Mat intrinsic_matrix_;
  std::vector<cv::Point3f> object_points_;

  Eigen::Affine3d previous_transform_;

  double translation[3];
  double rotation[3];
  cv::Mat cv_translation, cv_rotation, cv_distortion;

  std::vector <double> distortion_vector;

  cv::Rect roi_;

  std::vector<cv::Point2f> corners_;

  bool writePoseToFile;
  std::ostream *trajectory_out_;
  std::string TrajectoryFile;

public:  

  FastCheckerboardDetector(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

  virtual ~FastCheckerboardDetector()
  {
  }

  void handleCameraInfo(const sensor_msgs::CameraInfoConstPtr &info);

  void handleImageMessage(const sensor_msgs::ImageConstPtr& image);

  void resetROI();

  void limitROI(const cv::Mat& image);

  void updateROI();

  void fixCorners();
};

} /* namespace fast_checkerboard_detector */
#endif /* FAST_CHECKERBOARD_DETECTOR_H_ */
