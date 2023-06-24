#include "ros/ros.h"

#include <fcntl.h>   // Contains file controls like O_RDWR
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "gps_node");
    ros::NodeHandle nh;

    std::string device_name = "/dev/ttyUSB0";

    int serial_port = open(device_name.c_str(), O_RDWR);
    struct termios tty;

    if (tcgetattr(serial_port, &tty) != 0)
    {
        ROS_ERROR("Error on device %s: Failed to get attr: %s\n", device_name.c_str(), strerror(errno));
        return 2;
    }

    // Setting baudrate
    cfsetispeed(&tty, B4800);

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0)
    {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }

    if (serial_port < 0)
    {
        ROS_ERROR("Error on device %s: %s", device_name.c_str(), strerror(errno));
        return 1;
    }

    ros::Rate rate(1);

    while (!ros::isShuttingDown())
    {
        char read_buf[1000];
        int n = read(serial_port, &read_buf, sizeof(read_buf));

        if (n == 0)
        {
            ROS_ERROR("Some error occured while reading serial");
        }
        // if (read_buf[0] == '$')
        ROS_INFO("%s", read_buf);

        for (int i = 0; i < 1000; i++)
        {
            read_buf[i] = '\0';
        }

        // rate.sleep();
    }

    return 0;
}
