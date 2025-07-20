#include "glowstick_detection_node.hpp"
#include "../../../rover_shared_libs/rover_lib2/src/rover_lib2/helpers/constants.hpp"

GlowstickDetectionNode::GlowstickDetectionNode():
    Node("glowstick_detection_node")
{
    _camera = ImageCaptureGlowstick(
        //"rtspsrc location=" + Constants::CameraInfo::CAMERA_URL_MAP.at("Antenna") + _pipeline
        _pipelineWebcam
        );

    cv::Mat frameCam;
    cv::Mat frameGlowStick;
    GlowstickDetector glowsticks;
    std::vector<cv::Point> gsPosition;
    bool stopProgram = false;

    while(!stopProgram)
    {
        _camera._cap >> frameCam;
        frameGlowStick = frameCam.clone();

        glowsticks.drawGlowsticks(frameCam, frameGlowStick);
 
        cv::imshow("Laptop Camera", frameCam);
        cv::imshow("GlowStick Cam", frameGlowStick);

        if (cv::waitKey(GS_CONFIGURATION::WAIT_KEY_DELAY_MS) >= 0)
        {
            stopProgram = true;
        }
    }

    _camera._cap.release();
    cv::destroyAllWindows();
}

GlowstickDetectionNode::~GlowstickDetectionNode() {}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GlowstickDetectionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}