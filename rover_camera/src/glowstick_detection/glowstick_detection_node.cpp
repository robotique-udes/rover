#include "glowstick_detection_node.hpp"
#include "glowstick_detector.hpp"
#include "glowstick_detection.hpp"


GlowStickDetectionNode::GlowStickDetectionNode() : Node("glowstick_detection_node")
{
    _camera = std::make_unique<ImageCaptureGlowstick>("file:///dev/video0");

    cv::Mat frameCam;
    cv::Mat frameGlowStick;
    Glowstick glowstick;
    std::vector<cv::Point> gsPosition;

    while (true)
    {
        _camera->_cap >> frameCam;
        frameGlowStick = frameCam.clone();

        if (glowstick.drawGlowstick(frameGlowStick, frameCam))

        cv::imshow("Laptop Camera", frameCam);
        cv::imshow("GlowStick Cam", frameGlowStick);

        if (cv::waitKey(27) >=0) break;
    }

    _camera->_cap.release();
    cv::destroyAllWindows();

}

GlowStickDetectionNode::~GlowStickDetectionNode()
{}

int main(int argc, char * argv[])
{
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GlowStickDetectionNode>());
    return 0;
}