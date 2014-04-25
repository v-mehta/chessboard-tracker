/**
 * author: Vaibhav Mehta <vaibhavmehta.in@gmail.com>
 */

#ifndef FAST_CHECKERBOARD_DETECTOR_NODELET_H_
#define FAST_CHECKERBOARD_DETECTOR_NODELET_H_

#include <nodelet/nodelet.h>

#include <fast_checkerboard_detector/fast_checkerboard_detector.h>

namespace fast_checkerboard_detector
{

class FastCheckerboardDetectorNodelet : public nodelet::Nodelet
{
private:
  std::auto_ptr<FastCheckerboardDetector> detector_;
public:
  FastCheckerboardDetectorNodelet()
  {
  }

  virtual ~FastCheckerboardDetectorNodelet()
  {
  }

  virtual void onInit();
};

} /* namespace fast_checkerboard_detector */
#endif /* FAST_CHECKERBOARD_DETECTOR_NODELET_H_ */
