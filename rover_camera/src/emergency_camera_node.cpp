#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/srv/set_comms_link.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <opencv2/opencv.hpp>
#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

class EmergencyCameraNode : public rclcpp::Node
{
    static constexpr const char* TOPIC_COMMS_MODE = "/rover/network/comms_mode";
    static constexpr const char* TOPIC_EMERGENCY_SNAPSHOT = "/rover/camera/emergency_snapshot";

  public:
    EmergencyCameraNode() : Node("emergency_camera_node")
    {
        this->declare_parameter<std::string>("camera_url", "rtsp://192.168.144.30:554/1/h264major");
        this->declare_parameter<int>("interval_ms", 1500);
        this->declare_parameter<int>("target_width", 320);
        this->declare_parameter<int>("target_height", 240);
        this->declare_parameter<int>("jpeg_quality", 35);

        _cameraUrl = this->get_parameter("camera_url").as_string();
        _intervalMs = this->get_parameter("interval_ms").as_int();
        _targetWidth = this->get_parameter("target_width").as_int();
        _targetHeight = this->get_parameter("target_height").as_int();
        _jpegQuality = this->get_parameter("jpeg_quality").as_int();

        _pubSnapshot = this->create_publisher<sensor_msgs::msg::CompressedImage>(
            TOPIC_EMERGENCY_SNAPSHOT, rclcpp::QoS(1));

        _subCommsMode = this->create_subscription<std_msgs::msg::UInt8>(
            TOPIC_COMMS_MODE, rclcpp::QoS(1),
            [this](const std_msgs::msg::UInt8& msg_)
            {
                this->onCommsModeChanged(msg_.data);
            });

        _timerCapture = this->create_wall_timer(
            std::chrono::milliseconds(_intervalMs),
            [this]()
            {
                this->captureAndPublishSnapshot();
            });

        RCLCPP_INFO(this->get_logger(),
                    "Emergency Camera Node initialized (URL: %s, Interval: %d ms, Resolution: %dx%d, Quality: %d%%). Standing by.",
                    _cameraUrl.c_str(), _intervalMs, _targetWidth, _targetHeight, _jpegQuality);
    }

    ~EmergencyCameraNode() override
    {
        this->closeCamera();
    }

  private:
    void onCommsModeChanged(uint8_t mode_)
    {
        _commsMode = mode_;
        if (_commsMode == rover_msgs::srv::SetCommsLink::Request::MODE_PRIMARY_5G)
        {
            // Release camera resources when dormant on primary 5G Wi-Fi
            this->closeCamera();
            RCLCPP_INFO(this->get_logger(), "Primary link active. Emergency camera standing by.");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Fallback or Auto link active. Emergency camera capturing enabled.");
        }
    }

    bool openCamera()
    {
        if (_cap.isOpened())
        {
            return true;
        }

        // Try GStreamer RTSP pipeline for low-latency single frame extraction
        if (_cameraUrl.rfind("rtsp://", 0) == 0)
        {
            std::string pipeline = "rtspsrc location=\"" + _cameraUrl +
                                   "\" latency=0 ! decodebin ! videoconvert ! appsink sync=false drop=true max-buffers=1";
            _cap.open(pipeline, cv::CAP_GSTREAMER);
        }
        else
        {
            // USB / V4L2 fallback (e.g. /dev/video0)
            _cap.open(_cameraUrl);
        }

        if (!_cap.isOpened())
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Failed to open emergency camera stream: %s", _cameraUrl.c_str());
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Emergency camera stream opened: %s", _cameraUrl.c_str());
        return true;
    }

    void closeCamera()
    {
        if (_cap.isOpened())
        {
            _cap.release();
        }
    }

    void captureAndPublishSnapshot()
    {
        // Only run when 900MHz link is active or auto-failover is engaged
        if (_commsMode == rover_msgs::srv::SetCommsLink::Request::MODE_PRIMARY_5G)
        {
            return;
        }

        if (!this->openCamera())
        {
            return;
        }

        cv::Mat frame;
        if (!_cap.read(frame) || frame.empty())
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 4000,
                                 "Failed to grab frame from emergency camera stream.");
            this->closeCamera();
            return;
        }

        // Downscale to target resolution for low bandwidth
        cv::Mat resized;
        if (frame.cols != _targetWidth || frame.rows != _targetHeight)
        {
            cv::resize(frame, resized, cv::Size(_targetWidth, _targetHeight), 0, 0, cv::INTER_AREA);
        }
        else
        {
            resized = frame;
        }

        // Compress as JPEG
        std::vector<uchar> encodedBuf;
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, _jpegQuality};
        if (!cv::imencode(".jpg", resized, encodedBuf, params))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to compress snapshot to JPEG.");
            return;
        }

        // Publish CompressedImage
        sensor_msgs::msg::CompressedImage msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "emergency_camera";
        msg.format = "jpeg";
        msg.data = std::move(encodedBuf);

        _pubSnapshot->publish(msg);
    }

    std::string _cameraUrl;
    int _intervalMs{1500};
    int _targetWidth{320};
    int _targetHeight{240};
    int _jpegQuality{35};
    uint8_t _commsMode{rover_msgs::srv::SetCommsLink::Request::MODE_PRIMARY_5G};

    cv::VideoCapture _cap;

    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr _pubSnapshot;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr _subCommsMode;
    rclcpp::TimerBase::SharedPtr _timerCapture;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EmergencyCameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

