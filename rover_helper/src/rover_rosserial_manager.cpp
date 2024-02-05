#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()

#include "rclcpp/rclcpp.hpp"

#include "rover_ros_serial.hpp"

rclcpp::Node::SharedPtr nodePtr;

int getPort(const char *port, speed_t baudrate);
void checkForMsg(int serialPort, rclcpp::Duration timeout);
void parseLog(int serialPort, uint16_t msgLength);

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    nodePtr = std::make_shared<rclcpp::Node>("rover_serial_node");

    int serialPort = getPort("/dev/serial/by-id/usb-1a86_USB_Single_Serial_5573016028-if00", B115200);

    while (rclcpp::ok())
    {
        checkForMsg(serialPort, rclcpp::Duration(0, 1000u));
        rclcpp::spin_some(nodePtr);
    }

    rclcpp::shutdown();
    return 0;
}

int getPort(const char *port, speed_t baudrate)
{

    int serialPort = open(port, O_RDWR);
    if (serialPort < 0)
    {
        printf("Error %i from open: %s\n", errno, strerror(errno));
    }

    struct termios options;
    if (tcgetattr(serialPort, &options) != 0)
    {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }

    // Disabling line processing cause I want to parse character by character
    options.c_lflag = 0;

    cfsetispeed(&options, baudrate); // In baud rate
    cfsetospeed(&options, baudrate); // Out baud rate

    if (tcsetattr(serialPort, TCSANOW, &options) != 0)
    {
        printf("Error %i from tcsetattr TCSANOW: %s\n", errno, strerror(errno));
    }

    if (tcsetattr(serialPort, TCSAFLUSH, &options))
    {
        printf("Error %i from tcsetattr TCSAFLUSH: %s\n", errno, strerror(errno));
    }

    return serialPort;
}

void checkForMsg(int serialPort, rclcpp::Duration timeout)
{
    if (timeout.seconds() < rclcpp::Duration(0, 1000u).seconds())
    {
        RCLCPP_WARN_ONCE(nodePtr->get_logger(), "timeout cannot be lower than 1ms");
        timeout = rclcpp::Duration(0, 1000u);
    }

    // Check for inbound msg
    rclcpp::Time startTime = nodePtr->get_clock()->now();
    uint8_t __start = 0;
    for (;;)
    {
        read(serialPort, &__start, sizeof(__start));
        // printf("__start?: %u\n", __start);

        if (__start == RoverRosSerial::Constant::BEGIN)
        {
            // printf("Start of a msg detected!\n");
            break;
        }

        // Loop timeout
        if (nodePtr->get_clock()->now().seconds() > startTime.seconds() + timeout.seconds())
        {
            return;
        }
    }

    // Get's header and parse
    RoverRosSerial::Constant::uHeader packetHeader;
    read(serialPort, &packetHeader, sizeof(packetHeader));
    // printf("Type: %u | Length: %u\n", packetHeader.header.type, packetHeader.header.length);

    switch (packetHeader.header.type)
    {
    case RoverRosSerial::Constant::BEGIN:
        RCLCPP_WARN(nodePtr->get_logger(), "Shouldn't receive a \"BEGIN(%u)\" here, dropping both msgs", RoverRosSerial::Constant::BEGIN);
        break;

    case RoverRosSerial::Constant::eHeaderType::log:
        parseLog(serialPort, packetHeader.header.length);
        break;

    case RoverRosSerial::Constant::eHeaderType::heartbeat:
        RCLCPP_INFO(nodePtr->get_logger(), "Received heartbeat!");
        break;

    default:
        break;
    }

    return;
}

void parseLog(int serialPort, uint16_t msgLength)
{
    RoverRosSerial::SerialLogger packetLogger;
    read(serialPort, &packetLogger.uData.packetData, msgLength);

    if (packetLogger.uData.packetMsg.severity == static_cast<uint8_t>(rclcpp::Logger::Level::Debug))
    {
        RCLCPP_DEBUG(nodePtr->get_logger(), "%s", packetLogger.uData.packetMsg.msg);
    }
    else if (packetLogger.uData.packetMsg.severity == static_cast<uint8_t>(rclcpp::Logger::Level::Info))
    {
        RCLCPP_INFO(nodePtr->get_logger(), "%s", packetLogger.uData.packetMsg.msg);
    }
    else if (packetLogger.uData.packetMsg.severity == static_cast<uint8_t>(rclcpp::Logger::Level::Warn))
    {
        RCLCPP_WARN(nodePtr->get_logger(), "%s", packetLogger.uData.packetMsg.msg);
    }
    else if (packetLogger.uData.packetMsg.severity == static_cast<uint8_t>(rclcpp::Logger::Level::Error))
    {
        RCLCPP_ERROR(nodePtr->get_logger(), "%s", packetLogger.uData.packetMsg.msg);
    }
    else if (packetLogger.uData.packetMsg.severity == static_cast<uint8_t>(rclcpp::Logger::Level::Fatal))
    {
        RCLCPP_FATAL(nodePtr->get_logger(), "%s", packetLogger.uData.packetMsg.msg);
    }

    // printf("__buffer: %s\n", packetLogger.uData.packetMsg.msg);
}
