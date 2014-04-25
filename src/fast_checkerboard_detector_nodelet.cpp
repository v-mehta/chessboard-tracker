/**
 * author: Christian Kerl <christian.kerl@in.tum.de>
 */

#include <fast_checkerboard_detector/fast_checkerboard_detector_nodelet.h>
#include <pluginlib/class_list_macros.h>


PLUGINLIB_DECLARE_CLASS(fast_checkerboard_detector, detector, fast_checkerboard_detector::FastCheckerboardDetectorNodelet, nodelet::Nodelet)

namespace fast_checkerboard_detector
{

void FastCheckerboardDetectorNodelet::onInit()
{
  detector_.reset(new FastCheckerboardDetector(getNodeHandle(), getPrivateNodeHandle()));
}

} /* namespace fast_checkerboard_detector */
