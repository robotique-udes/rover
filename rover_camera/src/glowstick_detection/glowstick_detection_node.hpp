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
};

#endif