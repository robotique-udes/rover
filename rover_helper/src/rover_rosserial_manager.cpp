#include <fcntl.h>    // Contains file controls like O_RDWR
#include <errno.h>    // Error integer and strerror() function
#include <termios.h>  // Contains POSIX terminal control definitions
#include <unistd.h>   // write(), read(), close()
#include <sys/file.h> // flock()

#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "rover_ros_serial/rover_ros_serial.hpp"
#include "rovus_lib/timer.hpp"

#include "rover_msgs/msg/gps.hpp" // ros2's definition
#include "rover_msgs/msg/Gps.hpp" // rover_ros_serial's definition

#include "rover_msgs/msg/antenna_cmd.hpp" // ros2's definition

volatile sig_atomic_t shutdownFlag = 0;
void signal_handler(int signo);

class RoverRosSerialManager
{
public:
    RoverRosSerialManager(std::string nodeName_, std::string serialPortName_, speed_t baudrate_ = B4000000, unsigned long serialTimeoutUS_ = 1000u)
    {
        _nodeName = nodeName_;
        _nodeNameFromSerial = nodeName_ + "(From UC)";

        _serialPortName = serialPortName_;
        _baudrate = baudrate_;

        if (serialTimeoutUS_ < 1u)
        {
            RCLCPP_WARN_ONCE(rclcpp::get_logger(_nodeName), "timeoutUS_ cannot be lower than 1us");
            serialTimeoutUS_ = 1u;
        }
        timerTimeout.init(serialTimeoutUS_);
    }
    virtual ~RoverRosSerialManager()
    {
        flock(_serialPortFD, LOCK_UN);
    }
    virtual void spinOnce()
    {
        this->checkWatchdog();
        this->checkSerialPortState();
        this->checkForMsg();
    }

private:
    std::string _nodeName;
    std::string _nodeNameFromSerial;
    std::string _serialPortName;
    int _serialPortFD;
    speed_t _baudrate;

    bool _connected = false;
    RovusLib::Timer<unsigned long, &RovusLib::millis> timerWatchdog = RovusLib::Timer<unsigned long, &RovusLib::millis>(250u);
    RovusLib::Timer<unsigned long, &RovusLib::micros> timerTimeout;
    RovusLib::Timer<unsigned long, &RovusLib::millis> timerReconnect = RovusLib::Timer<unsigned long, &RovusLib::millis>(1000u);

    int openSerialPort()
    {
        _serialPortFD = open(_serialPortName.c_str(), O_RDWR);
        if (_serialPortFD < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger(_nodeName), "Error %i from open: %s\n\tOn device: %s", errno, strerror(errno), _serialPortName.c_str());
            return -1;
        }

        if (flock(_serialPortFD, LOCK_EX | LOCK_NB) != 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger(_nodeName), "Failed to acquire lock on %s. Another process may have the port open.\n", _serialPortName.c_str());
            this->closeSerialPort();
            return -1;
        }

        struct termios options;
        if (tcgetattr(_serialPortFD, &options) != 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger(_nodeName), "Error %i from tcgetattr: %s\n", errno, strerror(errno));
            this->closeSerialPort();
            return -1;
        }

        // Disabling line processing cause we want to parse character by character
        options.c_lflag = 0;
        cfsetispeed(&options, _baudrate); // In baud rate
        cfsetospeed(&options, _baudrate); // Out baud rate

        if (tcsetattr(_serialPortFD, TCSANOW, &options) != 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger(_nodeName), "Error %i from tcsetattr TCSANOW: %s\n", errno, strerror(errno));
            this->closeSerialPort();
            return -1;
        }

        if (tcsetattr(_serialPortFD, TCSAFLUSH, &options))
        {
            RCLCPP_ERROR(rclcpp::get_logger(_nodeName), "Error %i from tcsetattr TCSAFLUSH: %s\n", errno, strerror(errno));
            this->closeSerialPort();
            return -1;
        }

        _connected = true;
        RCLCPP_INFO(rclcpp::get_logger(_nodeName), "Successfully connected with serial port %s", _serialPortName.c_str());
        return _serialPortFD;
    }
    void closeSerialPort()
    {
        flock(_serialPortFD, LOCK_UN);
        close(_serialPortFD);
    }
    void checkSerialPortState()
    {
        if (!_connected)
        {
            if (timerReconnect.isDone())
            {
                _serialPortFD = this->openSerialPort();
            }
        }
    }
    void checkWatchdog()
    {
        // Means it didn't received a msg in the allowed time window
        if (timerWatchdog.isDone())
        {
            this->closeSerialPort();
            _connected = false;
        }
    }
    void checkForMsg()
    {
        if (!_connected)
        {
            return;
        }

        // Check for inbound msg
        uint8_t __start = 0;
        timerTimeout.reset();
        for (;;)
        {
            read(_serialPortFD, &__start, sizeof(__start));

            if (__start == RoverRosSerial::Constant::BEGIN)
            {
                break;
            }

            // Loop timeout
            if (timerTimeout.isDone())
            {
                // RCLCPP_INFO(_nodePtr->get_logger(), "No msg inbound before timeout");
                return;
            }
        }

        // Get's header and parse
        RoverRosSerial::Constant::uHeader packetHeader;
        read(_serialPortFD, &packetHeader, sizeof(packetHeader));

        switch (packetHeader.header.type)
        {
        case RoverRosSerial::Constant::BEGIN:
            RCLCPP_WARN(rclcpp::get_logger(_nodeName), "Shouldn't receive a \"BEGIN(%u)\" here, dropping current and next msgs", RoverRosSerial::Constant::BEGIN);
            break;

        case RoverRosSerial::Constant::eHeaderType::log:
            cbLog(_serialPortFD, packetHeader.header.length);
            break;

        case RoverRosSerial::Constant::eHeaderType::heartbeat:
            this->cbWatchdog();
            break;

        default:
            break;
        }

        // RCLCPP_ERROR(rclcpp::get_logger(_nodeName), "Shouldn't fall here, error in serialization, dropping current and next msgs");
        return;
    }
    void cbWatchdog()
    {
        timerWatchdog.reset();
    }
    void cbLog(int _serialPortFD, uint16_t msgLength)
    {
        RoverRosSerial::SerialLogger packetLogger;
        read(_serialPortFD, &packetLogger.uMsg.packetData, msgLength);

        if (packetLogger.uMsg.packetMsg.severity == static_cast<uint8_t>(rclcpp::Logger::Level::Debug))
        {
            RCLCPP_DEBUG(rclcpp::get_logger(_nodeNameFromSerial), "%s", packetLogger.uMsg.packetMsg.msg);
        }
        else if (packetLogger.uMsg.packetMsg.severity == static_cast<uint8_t>(rclcpp::Logger::Level::Info))
        {
            RCLCPP_INFO(rclcpp::get_logger(_nodeNameFromSerial), "%s", packetLogger.uMsg.packetMsg.msg);
        }
        else if (packetLogger.uMsg.packetMsg.severity == static_cast<uint8_t>(rclcpp::Logger::Level::Warn))
        {
            RCLCPP_WARN(rclcpp::get_logger(_nodeNameFromSerial), "%s", packetLogger.uMsg.packetMsg.msg);
        }
        else if (packetLogger.uMsg.packetMsg.severity == static_cast<uint8_t>(rclcpp::Logger::Level::Error))
        {
            RCLCPP_ERROR(rclcpp::get_logger(_nodeNameFromSerial), "%s", packetLogger.uMsg.packetMsg.msg);
        }
        else if (packetLogger.uMsg.packetMsg.severity == static_cast<uint8_t>(rclcpp::Logger::Level::Fatal))
        {
            RCLCPP_FATAL(rclcpp::get_logger(_nodeNameFromSerial), "%s", packetLogger.uMsg.packetMsg.msg);
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::signal(SIGINT, signal_handler);

    rclcpp::Node::SharedPtr nodePtr = std::make_shared<rclcpp::Node>("rover_serial_node");

    // Create ROS objects here
    rclcpp::Publisher<rover_msgs::msg::Gps>::SharedPtr pubGps;
    pubGps = nodePtr->create_publisher<rover_msgs::msg::Gps>("/base/antenna/gps/position", 1);
    // rclcpp::Publisher<rover_msgs::msg::AntennaCmd>::SharedPtr pubAntennaCmd;
    // nodePtr->create_publisher<rover_msgs::msg::AntennaCmd>("/base/antenna/cmd/out/motor", 1);

    RoverRosSerialManager roverRosSerialManager(nodePtr->get_name(), "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5573017171-if00", B4000000, 1000u);

    while (!shutdownFlag)
    {
        roverRosSerialManager.spinOnce();
        rclcpp::spin_some(nodePtr);
    }

    rclcpp::shutdown();
    return 0;
}

// Necessary for clean shutdown using spin_some
void signal_handler(int signo)
{
    (void)signo;
    shutdownFlag = 1;
}
