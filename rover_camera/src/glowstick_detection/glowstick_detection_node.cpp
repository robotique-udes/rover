#include "glowstick_detection_node.hpp"

GlowstickDetectionNode::GlowstickDetectionNode():
    Node("glowstick_detection_node")
{
    _camera = ImageCaptureGlowstick(
        //"rtspsrc location=rtsp://192.168.144.31:554/1/h264major latency=0 drop-on-latency=true protocols=tcp ! "
        "v4l2src device=/dev/video0 ! "
        "decodebin ! "
        "videorate max-rate=20 ! "
        "videoconvert ! "
        "queue max-size-buffers=1 leaky=downstream ! "
        "appsink sync=false");

    cv::Mat frameCam;
    cv::Mat frameGlowStick;
    GlowstickDetector glowsticks;
    std::vector<cv::Point> gsPosition;

    while (true)
    {
        _camera._cap >> frameCam;
        frameGlowStick = frameCam.clone();

        if (glowsticks.drawGlowsticks(frameCam, frameGlowStick))

            cv::imshow("Laptop Camera", frameCam);
        cv::imshow("GlowStick Cam", frameGlowStick);

        if (cv::waitKey(27) >= 0)
            break;
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