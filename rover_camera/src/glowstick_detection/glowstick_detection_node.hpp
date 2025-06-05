#ifndef GLOWSTICK_DETECTION_NODE_HPP
#define GLOWSTICK_DETECTION_NODE_HPP

#include "image_capture_glowstick.hpp"  

class GlowStickDetectionNode : public rclcpp::Node
{
public:
    GlowStickDetectionNode();
    ~GlowStickDetectionNode();

private:

    std::unique_ptr<ImageCaptureGlowstick> _camera;
    rclcpp::TimerBase::SharedPtr _timer;
};

#endif