#ifndef GLOWSTICK_DETECTION_NODE_HPP
#define GLOWSTICK_DETECTION_NODE_HPP

#include "image_capture_glowstick.hpp"
#include "glowstick_detector.hpp"
#include "glowstick.hpp"

class GlowstickDetectionNode : public rclcpp::Node
{
  public:
    GlowstickDetectionNode();
    ~GlowstickDetectionNode();

  private:
    ImageCaptureGlowstick _camera;
    rclcpp::TimerBase::SharedPtr _timer;
    static constexpr const char* _pipeline = "latency=0 drop-on-latency=true protocols=tcp ! decodebin ! videorate max-rate=20 ! videoconvert ! queue max-size-buffers=1 leaky=downstream ! appsink sync=false";
    static constexpr const char* _pipelineWebcam = "latency=0 drop-on-latency=true protocols=tcp ! v4l2src device=/dev/video0 !  decodebin ! videorate max-rate=20 ! videoconvert ! queue max-size-buffers=1 leaky=downstream ! appsink sync=false"; //For debug
};

#endif