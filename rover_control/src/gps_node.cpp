#include "ros/ros.h"

#include <fcntl.h>   // Contains file controls like O_RDWR
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()
#include <unordered_map>
#include "rover_control_msgs/gps.h"

#define IN
#define IN_OUT
#define OUT
#define GET_WORST_OF(X, Y) ((X) < (Y) ? (X) : (Y))
#define ERROR(X) static_cast<int>(X)
#define SUCCESS static_cast<int>(ErrorCodes::Success)

#define SIZE_MSG_BUFFER 1000
#define SIZE_NB_ELEMENTS 20
#define SIZE_VALUES 20

enum class ErrorCodes
{
    Failure = INT8_MIN,
    NotImplemented,
    InvalidArguments,
    SyntaxError,
    Drop,
    DeviceError,
    InvalidPosition,
    EmptyMessage = 0,
    Success = 1
};

enum class GPSFixQuality
{
    Invalid = rover_control_msgs::gps::FIX_INVALID,       // Bad
    Standalone = rover_control_msgs::gps::FIX_STANDALONE, // 30m +0m -20m
    DGPS = rover_control_msgs::gps::FIX_DGPS,             // 10m +0m -8m
    RTKFixed = rover_control_msgs::gps::FIX_RTK_FIXED,    // 1.0m +0.0m -0.9m
    RTKFloat = rover_control_msgs::gps::FIX_RTK_FLOAT     // No info
};

enum class GGAElements
{
    Latitude = 2,
    LatitudeDir,
    Longitude,
    LongitudeDir,
    GPSFixQuality,
    Satellite,
    Height = 9,
};

enum class RMCElements
{
    PositionStatus = 2,
    Latitude,
    LatitudeDir,
    Longitude,
    LongitudeDir,
    Speed,
    TrackHeading,
    GPSHeading = 10,
    GPSHeadingDir
};

int initialiseDevice(IN std::string device_name, int *serial_port);
int getInfo(char read_buf[SIZE_MSG_BUFFER], rover_control_msgs::gps* gps_msg);
int strToInt(IN const char *str, IN uint8_t size);
int findSizesDegreesMinutes(IN const char *str, OUT uint8_t *size_degrees, OUT uint8_t *size_minutes, IN uint8_t max = 100);
float strToFloat(IN const char *str, IN uint8_t skip, IN uint8_t size);
float strToFloat(IN const char *str, IN uint8_t size);
int getLatLong(IN const char elements[SIZE_NB_ELEMENTS][SIZE_VALUES], IN int data_type, OUT float *data);
int getElement(IN const char elements[SIZE_NB_ELEMENTS][SIZE_VALUES], IN int data_type, OUT int8_t *data);
int getElement(IN const char elements[SIZE_NB_ELEMENTS][SIZE_VALUES], IN int data_type, OUT float *data);
void clearBuffer(char *buf, uint16_t size);

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "gps_node");
    ros::NodeHandle nh;

    int serial_port = 0;
    if (initialiseDevice(nh.param<std::string>(ros::this_node::getName() + "/port",
                                               "/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_D-if00-port0"),
                         &serial_port) != 1)
    {
        ROS_ERROR("%s:main() Error in serial device init, exiting", ros::this_node::getName().c_str());
        return ERROR(ErrorCodes::Failure);
    }

    rover_control_msgs::gps gps_msg;
    ros::Publisher pub_gps = nh.advertise<rover_control_msgs::gps>("gps_data", 1);

    while (!ros::isShuttingDown())
    {
        char read_buf[SIZE_MSG_BUFFER];
        int n = read(serial_port, &read_buf, sizeof(read_buf));

        if (n == 0)
        {
            ROS_ERROR("%s:main() Some error occured while reading serial", ros::this_node::getName().c_str());
        }

        // ROS_INFO("%s", read_buf);

        int res = getInfo(read_buf, &gps_msg);
        if (res != 1)
        {
            if (res == ERROR(ErrorCodes::EmptyMessage))
                ROS_DEBUG("%s:main() Empty message, dropping message", ros::this_node::getName().c_str());
            else if (res == ERROR(ErrorCodes::SyntaxError))
                ROS_ERROR("%s:main() Error \"%d\" in buffer, dropping message", ros::this_node::getName().c_str(), res);
            else if (res == ERROR(ErrorCodes::InvalidPosition))
                ROS_ERROR("%s:main() Error Position is invalid, dropping message", ros::this_node::getName().c_str());
        }
        else
        {
            pub_gps.publish(gps_msg);
        }

        clearBuffer(read_buf, sizeof(read_buf));
    }

    return 0;
}

int initialiseDevice(IN std::string device_name, int *serial_port)
{
    *serial_port = open(device_name.c_str(), O_RDWR);
    ROS_INFO("%s::main() Selected device: %s", ros::this_node::getName().c_str(), device_name.c_str());
    struct termios tty;

    if (tcgetattr(*serial_port, &tty) != 0)
    {
        ROS_ERROR("Error on device %s: Failed to get attr: %s\n", device_name.c_str(), strerror(errno));
        return ERROR(ErrorCodes::DeviceError);
    }

    // Baudrate
    cfsetispeed(&tty, B4800);

    if (tcsetattr(*serial_port, TCSANOW, &tty) != 0)
    {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }

    if (serial_port < 0)
    {
        ROS_ERROR("Error on device %s: %s", device_name.c_str(), strerror(errno));
        return ERROR(ErrorCodes::Failure);
    }
    else
    {
        return ERROR(ErrorCodes::Success);
    }
}

int getInfo(char zs_read_buf[SIZE_MSG_BUFFER], rover_control_msgs::gps* gps_msg)
{
    if (zs_read_buf[0] == '\n' || zs_read_buf[0] == '\0')
        return ERROR(ErrorCodes::EmptyMessage);
    else if (zs_read_buf[0] != '$')
        return ERROR(ErrorCodes::SyntaxError);

    int res = ERROR(ErrorCodes::Success);
    char elements[SIZE_NB_ELEMENTS][SIZE_VALUES] = {0};
    uint8_t index_element = 0;
    uint8_t index_character = 0;
    uint16_t cursor = 0;

    for (; zs_read_buf[cursor] != '\n' && cursor < SIZE_MSG_BUFFER && index_element < SIZE_NB_ELEMENTS; cursor++)
    {
        if (zs_read_buf[cursor] == ',')
        {
            elements[index_element][index_character + 1] = '\0';
            index_character = 0;
            index_element++;
        }
        else
        {
            elements[index_element][index_character] = zs_read_buf[cursor];
            index_character++;
        }
    }
    uint8_t size_elements = index_element + 1;

    if (strncmp(elements[0], "$GPGGA", sizeof("$GPGGA") - 1UL) == 0)
    {
        float latitude = -69.0f;
        float longitude = -69.0f;
        int8_t fix_quality = static_cast<int8_t>(GPSFixQuality::Invalid);
        int8_t nb_satellite = -69;
        float height = -69.0f;

        res = GET_WORST_OF(res, getLatLong(elements, ERROR(GGAElements::Latitude), &latitude));
        res = GET_WORST_OF(res, getLatLong(elements, ERROR(GGAElements::Longitude), &longitude));
        res = GET_WORST_OF(res, getElement(elements, ERROR(GGAElements::GPSFixQuality), &fix_quality));
        res = GET_WORST_OF(res, getElement(elements, ERROR(GGAElements::Satellite), &nb_satellite));
        res = GET_WORST_OF(res, getElement(elements, ERROR(GGAElements::Height), &height));

        if (res != ERROR(ErrorCodes::Success))
        {
            ROS_WARN("%s: Error parsing message - dropping", ros::this_node::getName().c_str());
            return res;
        }

        gps_msg->latitude = latitude;
        gps_msg->longitude = longitude;
        gps_msg->fix_quality = fix_quality;
        gps_msg->satellite = nb_satellite;
        gps_msg->height = height;
    }
    else if (strncmp(elements[0], "$GPRMC", sizeof("$GPRMC") - 1UL) == 0)
    {
        if (!(strncmp(elements[ERROR(RMCElements::PositionStatus)], "A", 1) == 0))
        {
            ROS_WARN("%s", elements[ERROR(RMCElements::PositionStatus)]);
            return ERROR(ErrorCodes::InvalidPosition);
        }
        else
        {
            float latitude = -69.0f;
            float longitude = -69.0f;
            float speed = -69.0f;
            float track_heading = 400.0f;
            float gps_heading = 400.f;

            res = GET_WORST_OF(res, getLatLong(elements, static_cast<int>(RMCElements::Latitude), &latitude));
            res = GET_WORST_OF(res, getLatLong(elements, static_cast<int>(RMCElements::Longitude), &longitude));
            res = GET_WORST_OF(res, getElement(elements, static_cast<int>(RMCElements::Speed), &speed));
            res = GET_WORST_OF(res, getElement(elements, static_cast<int>(RMCElements::TrackHeading), &track_heading));
            res = GET_WORST_OF(res, getElement(elements, static_cast<int>(RMCElements::GPSHeading), &gps_heading));

            if (elements[static_cast<int>(RMCElements::GPSHeadingDir)] == "W")
            {
                gps_heading =  -gps_heading + 360.0f;
            }

            gps_msg->latitude = latitude;
            gps_msg->longitude = longitude;
            gps_msg->speed = speed;
            gps_msg->heading_gps = gps_heading;
            gps_msg->heading_track = track_heading;
        }
    }
    else
    {
        return ERROR(ErrorCodes::NotImplemented);
    }

    return res;
}

int strToInt(IN const char *str, IN uint8_t size)
{
    char buffer[size + 1] = {'\0'};

    for (uint8_t i = 0; i < size; i++, str++)
    {
        buffer[i] = *str;
    }

    return atoi(buffer);
}

float strToFloat(IN const char *str, IN uint8_t size)
{
    char buffer[size + 1] = {'\0'};

    for (uint8_t i = 0; i < size; i++, str++)
    {
        buffer[i] = *str;
    }

    return atof(buffer);
}

float strToFloat(IN const char *str, IN uint8_t skip, IN uint8_t size)
{
    char buffer[size + 1] = {'\0'};
    str += skip;
    for (uint8_t i = 0; i < size; i++, str++)
    {
        buffer[i] = *str;
    }

    return atof(buffer);
}

int findSizesDegreesMinutes(IN const char *str, OUT uint8_t *size_degrees, OUT uint8_t *size_minutes, IN uint8_t max)
{
    uint8_t i = 0;
    for (; (i < max && *str != '\0'); i++, str++)
    {
        if (*str == '.')
        {
            *size_degrees = i - 2;
            break;
        }
    }

    if (i == max)
    {
        return ERROR(ErrorCodes::Drop);
    }

    for (; (*str != '\0' && i < max); i++, str++)
    {
    }

    if (i == max)
    {
        return ERROR(ErrorCodes::Drop);
    }

    *size_minutes = i - *size_degrees;

    return ERROR(ErrorCodes::Success);
}

int getLatLong(IN const char elements[SIZE_NB_ELEMENTS][SIZE_VALUES], IN int data_type, OUT float *data)
{
    if (data == NULL || elements == NULL && (data_type != static_cast<int>(GGAElements::Longitude) &&
                                             data_type != static_cast<int>(GGAElements::Latitude) &&
                                             data_type != static_cast<int>(RMCElements::Longitude) &&
                                             data_type != static_cast<int>(RMCElements::Latitude)))
    {
        return ERROR(ErrorCodes::InvalidArguments);
    }

    *data = 0;
    uint8_t size_degrees = 0;
    uint8_t size_minutes = 0;
    if (findSizesDegreesMinutes(elements[data_type], &size_degrees, &size_minutes) != ERROR(ErrorCodes::Drop))
    {
        float deg = static_cast<float>(strToInt(elements[data_type], size_degrees));
        float minutes = strToFloat(elements[data_type], size_degrees, size_minutes);

        *data = deg + (minutes / 60.0f);
    }
    else
    {
        return ERROR(ErrorCodes::Drop);
    }

    if (strncmp(elements[data_type + 1], "N", 1) != 0 && strncmp(elements[data_type + 1], "E", 1) != 0)
    {
        *data *= -1;
    }

    return ERROR(ErrorCodes::Success);
}

int getElement(IN const char elements[SIZE_NB_ELEMENTS][SIZE_VALUES], IN int data_type, OUT int8_t *data)
{
    if (elements[data_type][0] == '\0')
    {
        return ERROR(ErrorCodes::EmptyMessage);
    }

    *data = atoi(elements[data_type]);
    return ERROR(ErrorCodes::Success);
}

int getElement(IN const char elements[SIZE_NB_ELEMENTS][SIZE_VALUES], IN int data_type, OUT float *data)
{
    if (elements[data_type][0] == '\0')
    {
        return ERROR(ErrorCodes::EmptyMessage);
    }

    *data = strToFloat(elements[data_type], 0, sizeof(elements[data_type]));
    return ERROR(ErrorCodes::Success);
}

void clearBuffer(char *buffer, uint16_t size)
{
    for (int i = 0; i < size; i++)
    {
        buffer[i] = '\0';
    }
}
