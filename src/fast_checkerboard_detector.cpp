/**
 * author: Vaibhav Mehta <vaibhavmehta.in@gmail.com>
 */

#include <fast_checkerboard_detector/fast_checkerboard_detector.h>

namespace fast_checkerboard_detector
{

FastCheckerboardDetector::FastCheckerboardDetector(ros::NodeHandle& nh, ros::NodeHandle& nh_private) :
    it_(nh),
    cv_translation(3, 1, cv::DataType<double>::type, translation),
    cv_rotation(3, 1, cv::DataType<double>::type, rotation),
    cv_distortion(cv::Mat::zeros(1, 4, cv::DataType<double>::type))
  {
    if(!nh_private.getParam("grid_size_x", grid_size_x))
    {
      ROS_ERROR("Missing parameter ''!");
    }
    if(!nh_private.getParam("grid_size_y", grid_size_y))
    {
      ROS_ERROR("Missing parameter 'grid_size_y'!");
    }
    if(!nh_private.getParam("rect_size_x", rect_size_x))
    {
      ROS_ERROR("Missing parameter 'rect_size_x'!");
    }
    if(!nh_private.getParam("rect_size_y", rect_size_y))
    {
      ROS_ERROR("Missing parameter 'rect_size_y'!");
    }
    if(!nh_private.getParam("write_trajectory", writePoseToFile))
    {
      ROS_ERROR("Missing parameter 'write_trajectory'!");
    }

    corners_.reserve(grid_size_x * grid_size_y);

    double x_offset = rect_size_x * (grid_size_x - 1) / 2.0;
    double y_offset = rect_size_y * (grid_size_y - 1) / 2.0;

    for(int y = 0; y < grid_size_y; y++)
    {
      for(int x = 0; x < grid_size_x; x++)
      {
        object_points_.push_back(cv::Point3f(x * rect_size_x - x_offset, y * rect_size_y - y_offset, 0));
      }
    }
    
    if(writePoseToFile)
    {
      if(nh_private.getParam("trajectory_file", TrajectoryFile))
      {
        trajectory_out_ = new std::ofstream(TrajectoryFile.c_str());
        if(trajectory_out_->fail())
        {
          delete trajectory_out_;

          std::cerr << "Failed to open '" << "Checkerboard_marker_trajectory.txt" << "'!" << std::endl;
        }
      }
      else
      {
        trajectory_out_ = &std::cout;
      }
    }


    camera_pose_publisher_ = nh.advertise<geometry_msgs::PoseStamped>("pose", 1);
    camera_info_subscriber_ = nh.subscribe("camera_info", 1, &FastCheckerboardDetector::handleCameraInfo, this);

    resetROI();
  }

void FastCheckerboardDetector::handleCameraInfo(const sensor_msgs::CameraInfoConstPtr &info)
{
  camera_info_subscriber_.shutdown();

  intrinsic_matrix_ = cv::Mat(3, 3, cv::DataType<float>::type);

  const double* info_ptr = info->P.begin();

  for(int row_idx = 0; row_idx < intrinsic_matrix_.rows; ++row_idx)
  {
      float* row_ptr = intrinsic_matrix_.ptr<float>(row_idx);

      for(int col_idx = 0; col_idx < intrinsic_matrix_.cols; ++col_idx, row_ptr++, info_ptr++)
      {
        *row_ptr = (float) *info_ptr;
      }
      // skip 4th column
      info_ptr++;
  }

  img_subscriber_ = it_.subscribe("camera_image", 1, &FastCheckerboardDetector::handleImageMessage, this);
}


void FastCheckerboardDetector::handleImageMessage(const sensor_msgs::ImageConstPtr& message)
{
  cv_bridge::CvImageConstPtr image_bridge = cv_bridge::toCvShare(message, "mono8");
  const cv::Mat image = image_bridge->image;

  limitROI(image);

  //ROS_INFO_STREAM("ROI x: " << roi_.x << " y: " << roi_.y << " width: " << roi_.width << " height: " << roi_.height);

  bool found_chessboard = cv::findChessboardCorners(cv::Mat(image, roi_), cv::Size(grid_size_x, grid_size_y), corners_, 0);

  fixCorners();
  updateROI();

  if(found_chessboard)
  {

    cv::cornerSubPix(image, corners_, cv::Size(4, 4), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 30, 0.01));
    
    cv::drawChessboardCorners(image, cv::Size(grid_size_x,grid_size_y), cv::Mat(corners_), found_chessboard);

    cv::imshow("Corners",image);

    cv::waitKey(5);

    cv::solvePnP(object_points_, corners_, intrinsic_matrix_, cv_distortion, cv_rotation, cv_translation);
    
    Eigen::Vector3d rotation_vector(rotation[0], rotation[1], rotation[2]);
    Eigen::Translation3d translation_(translation[0], translation[1], translation[2]);

    Eigen::AngleAxisd rotation_;

    if(!rotation_vector.isZero(1e-6))
    {
      rotation_ = Eigen::AngleAxisd(rotation_vector.norm(), rotation_vector.normalized());
    }
    else
    {
      rotation_ = Eigen::AngleAxisd(0, Eigen::Vector3d(0, 0, 1));
    }

    Eigen::Affine3d transform = translation_ * rotation_;
    
    // disambiguate by tracking..
    Eigen::Affine3d rotated_transform = transform; //* Eigen::AngleAxisd(M_PI, Eigen::Vector3d(0,0,1));
    Eigen::Affine3d diff = previous_transform_.inverse() * transform;
    Eigen::Affine3d final_transform = diff(0, 0) >= 0 ? transform : rotated_transform;

    previous_transform_ = final_transform;

    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = message->header.frame_id;
    msg.header.stamp = message->header.stamp;

    tf::poseEigenToMsg(final_transform, msg.pose);

    camera_pose_publisher_.publish(msg);

    if(writePoseToFile)
      {
        Eigen::Affine3d camera_pose = final_transform;
        Eigen::Quaterniond q(camera_pose.rotation());

        (*trajectory_out_)
            << msg.header.stamp << " "
            << camera_pose.translation()(0) << " "
            << camera_pose.translation()(1) << " "
            << camera_pose.translation()(2) << " "
            << q.x() << " "
            << q.y() << " "
            << q.z() << " "
            << q.w() << " "
            << std::endl;
      }

  }
  else
  {
    resetROI();

    ROS_WARN_STREAM("Checkerboard tracking lost!");
  }
}



void FastCheckerboardDetector::resetROI()
{
  roi_ = cv::Rect(0, 0, 1280, 960);
}

void FastCheckerboardDetector::limitROI(const cv::Mat& image)
{
  roi_ &= cv::Rect(0, 0, image.cols, image.rows);
}

void FastCheckerboardDetector::fixCorners()
{
  BOOST_FOREACH(cv::Point2f& p, corners_)
  {
    p.x += roi_.x;
    p.y += roi_.y;
  }
}

void FastCheckerboardDetector::updateROI()
{
  // corner indices
  static size_t c1 = 0, c2 = grid_size_x - 1, c3 = grid_size_x * grid_size_y - grid_size_x, c4 = grid_size_x * grid_size_y - 1;

  int min_x = (int) std::min(corners_[c1].x, std::min(corners_[c2].x, std::min(corners_[c3].x, corners_[c4].x)));
  int min_y = (int) std::min(corners_[c1].y, std::min(corners_[c2].y, std::min(corners_[c3].y, corners_[c4].y)));
  int max_x = (int) std::max(corners_[c1].x, std::max(corners_[c2].x, std::max(corners_[c3].x, corners_[c4].x)));
  int max_y = (int) std::max(corners_[c1].y, std::max(corners_[c2].y, std::max(corners_[c3].y, corners_[c4].y)));

  cv::Rect updated_roi(cv::Point(min_x, min_y), cv::Point(max_x, max_y));

  updated_roi.x -= updated_roi.width / 2.0;
  updated_roi.y -= updated_roi.height / 2.0;
  updated_roi.width *= 2.0;
  updated_roi.height *= 2.0;

  roi_ = updated_roi;
}

} /* namespace fast_checkerboard_detector */
