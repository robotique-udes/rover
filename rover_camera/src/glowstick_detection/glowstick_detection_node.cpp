#include "glowstick_detection_node.hpp"
#include "glowstick_detector.hpp"


GlowStickDetectionNode::GlowStickDetectionNode() : Node("glowstick_detection_node")
{
    _camera = std::make_unique<ImageCaptureGlowstick>("file:///dev/video0");

    cv::Mat frameCam;
    cv::Mat frameGlowStick;
    GlowsitckDetector gsDetector;
    std::vector<cv::Point> gsPosition;

    while (true)
    {
        _camera->_cap >> frameCam;

        cv::imshow("Laptop Camera", frameCam);
        //cv::imshow("GlowStick Cam", frameGlowStick);

        gsPosition = gsDetector.getPosition();

        if (gsDetector.detectGlowstick(frameCam))
        {
            for (uint16_t i=0;i<gsPosition.size();i++)
            {
                cv::circle(frameCam, gsPosition[i], 5, cv::Scalar(0, 255, 0), 2);
            }
        }

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