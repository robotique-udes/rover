#include "glowstick_detection_node.hpp"
#include "glowstick_detector.hpp"


GlowStickDetectionNode::GlowStickDetectionNode() : Node("glowstick_detection_node")
{
    _camera = std::make_unique<ImageCaptureGlowstick>("file:///dev/video0");

    cv::Mat frameCam;
    cv::Mat frameGlowStick;
    GlowstickDetector gsDetector;
    std::vector<cv::Point> gsPosition;

    while (true)
    {
        _camera->_cap >> frameCam;
        frameGlowStick = frameCam.clone();

        if (gsDetector.detectGlowstick(frameCam))
        {
            cv::drawContours(frameGlowStick, gsDetector.redContours, -1, cv::Scalar(0, 0, 255),2);
            cv::drawContours(frameGlowStick, gsDetector.blueContours, -1, cv::Scalar(255, 0, 0),2);
            cv::drawContours(frameGlowStick, gsDetector.whiteContours, -1, cv::Scalar(0, 255, 0),2);
        }

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