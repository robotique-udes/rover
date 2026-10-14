#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/science_info.hpp>

class ScienceLoggerNode : public rclcpp::Node
{
    static constexpr const char* TOPIC_SCIENCE_INFO = "/rover/science/info";

  public:
    ScienceLoggerNode():
        Node("science_logger"),
        _startTime(std::chrono::steady_clock::now())
    {
        this->declare_parameter<std::string>("log_dir", "");
        this->declare_parameter<bool>("print_console", true);

        std::string customLogDir = this->get_parameter("log_dir").as_string();
        _printConsole = this->get_parameter("print_console").as_bool();

        std::filesystem::path logDir;
        if (!customLogDir.empty())
        {
            logDir = customLogDir;
        }
        else
        {
            const char* home = std::getenv("HOME");
            logDir = (home != nullptr) ? (std::filesystem::path(home) / "rover_science_logs") : std::filesystem::current_path();
        }

        std::filesystem::create_directories(logDir);

        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << "science_data_" << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S") << ".csv";
        _filePath = (logDir / ss.str()).string();

        _csvFile.open(_filePath, std::ios::out);
        if (_csvFile.is_open())
        {
            _csvFile << "Timestamp_ISO,Elapsed_Sec,Sample_Index,Sensor_1_PPM,Sensor_2_PPM,Sensor_3_PPM,Humidity\n";
            _csvFile.flush();
            RCLCPP_INFO(this->get_logger(), "Logging science sensor data to: %s", _filePath.c_str());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV log file: %s", _filePath.c_str());
        }

        _subScienceInfo = this->create_subscription<rover_msgs::msg::ScienceInfo>(
            TOPIC_SCIENCE_INFO,
            rclcpp::QoS(10),
            [this](const rover_msgs::msg::ScienceInfo& msg)
            {
                this->scienceInfoCallback(msg);
            });

        RCLCPP_INFO(this->get_logger(), "Subscribed to %s. Waiting for science sensor data...", TOPIC_SCIENCE_INFO);
    }

    ~ScienceLoggerNode() override
    {
        if (_csvFile.is_open())
        {
            _csvFile.close();
            RCLCPP_INFO(this->get_logger(), "Closed CSV log file: %s (Total samples: %zu)", _filePath.c_str(), _sampleCount);
        }
    }

  private:
    void scienceInfoCallback(const rover_msgs::msg::ScienceInfo& msg)
    {
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

        std::stringstream timeStr;
        timeStr << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %H:%M:%S")
                << '.' << std::setfill('0') << std::setw(3) << ms.count();

        auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - _startTime);

        if (_csvFile.is_open())
        {
            _csvFile << timeStr.str() << ","
                     << std::fixed << std::setprecision(3) << elapsed.count() << ","
                     << msg.sample_index << ","
                     << msg.sensor_1 << ","
                     << msg.sensor_2 << ","
                     << msg.sensor_3 << ","
                     << msg.humidity << "\n";
            _csvFile.flush();
            _sampleCount++;
        }

        if (_printConsole)
        {
            RCLCPP_INFO(this->get_logger(),
                        "[%s] #%u | S1: %4u ppm | S2: %4u ppm | S3: %4u ppm | Hum: %u",
                        timeStr.str().c_str(),
                        msg.sample_index,
                        msg.sensor_1,
                        msg.sensor_2,
                        msg.sensor_3,
                        msg.humidity);
        }
    }

    rclcpp::Subscription<rover_msgs::msg::ScienceInfo>::SharedPtr _subScienceInfo;
    std::ofstream _csvFile;
    std::string _filePath;
    std::chrono::steady_clock::time_point _startTime;
    size_t _sampleCount = 0;
    bool _printConsole = true;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ScienceLoggerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
