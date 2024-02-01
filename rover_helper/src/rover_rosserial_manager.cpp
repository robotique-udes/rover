#include <stdio.h>
#include <string.h>
#include <stdint.h>

// Linux headers
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()

#include "rover_ros_serial.hpp"

enum eHeaderCode : uint8_t
{
    BEGIN = 128,
    publisher = 130,
    subscriber = 150,
    config = 170,
    eLast
};

void clearBuffer(char *buffer, uint16_t size);

int main(/*int argc, char *argv[]*/ void)
{
    int serial_port = open("/dev/serial/by-id/usb-1a86_USB_Single_Serial_5573016028-if00", O_RDWR);

    if (serial_port < 0)
    {
        printf("Error %i from open: %s\n", errno, strerror(errno));
    }

    struct termios tty;
    if (tcgetattr(serial_port, &tty) != 0)
    {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }

    cfsetispeed(&tty, B115200); // In baud rate
    cfsetospeed(&tty, B115200); // Out baud rate

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0)
    {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }

    for (;;)
    {
        uint8_t __start = 0;
        uint8_t __msg[1000] = {0};

        for (;;)
        {
            read(serial_port, &__start, sizeof(__start));
            // printf("__start?: %u\n", __start);

            if (__start == RoverRosSerial::Constant::BEGIN)
            {
                printf("Start of a msg detected!\n");
                break;
            }
        }

        RoverRosSerial::Constant::uHeader packetHeader;
        read(serial_port, &packetHeader, sizeof(packetHeader));
        printf("Type: %u | Length: %u\n", packetHeader.header.type, packetHeader.header.length);

        if (packetHeader.header.length < sizeof(__msg))
        {
            RoverRosSerial::MsgLogger packetLogger;
            // if (packetLogger.getSerializedDataSize() != packetHeader.header.length)
            // {
            //     printf("Wrong length for msg");
            //     break;
            // }

            read(serial_port, &packetLogger.uData.packetData, packetHeader.header.length);
            printf("__msg: %s\n", packetLogger.uData.packetMsg.msg);
        }
        else
        {
            printf("Dropping msg too long\n");
        }
    }

    return 0;
}
