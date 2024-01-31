#include <stdio.h>
#include <string.h>
#include <stdint.h>

// Linux headers
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()

typedef struct rover_msgs__msg__AntennaCmd
{
    uint8_t header;
    bool status;
    float speed;
    uint8_t ofl;
} rover_msgs__msg__AntennaCmd;

union rover_msgs__msg__AntennaCmd__packet
{
    rover_msgs__msg__AntennaCmd msg;
    uint8_t data[sizeof(rover_msgs__msg__AntennaCmd)];
};

enum eHeaderCode : uint8_t
{
    BEGIN = 128,
    publisher = 130,
    subscriber = 150,
    config = 170,
    eLast
};

typedef struct msgs__Logging
{
    uint8_t header;
    char msg[255];
} msgs__Logging;

union msgs__Logging__packet
{
    msgs__Logging msg;
    uint8_t data[sizeof(msgs__Logging)];
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

    rover_msgs__msg__AntennaCmd__packet packetPub;
    msgs__Logging__packet packetLog;
    uint8_t buffer[255];
    for (;;)
    {
        // Wait for begin sequence (0x69)
        uint8_t __start = 0;
        uint8_t __length = 0;
        uint8_t __msg[255] = {0};
        for(;;)
        {
            read(serial_port, &__start, sizeof(__start));
            if (__start != 0)
            {
                printf("Code is: %i\n", __start);

                if (__start == 128)
                {
                    printf("Start of a msg detected!\n");
                    break;
                }
            }
        }

        read(serial_port, &__length, sizeof(__length));
        printf("__length: %i", __length);

        read(serial_port, &__msg, __length);

        printf("__msg: %s\n", __msg);

        // int length = read(serial_port, &buffer, sizeof(buffer));
        // buffer[length] = '\0';

        // if (length < 0)
        // {
        //     printf("Error\n");
        // }
        // else if (length == 0)
        // {
        //     // printf("No data, skipping...\n");
        //     // No data, skipping;
        // }
        // else
        // {
        //     // Checking header
        //     printf("---\nheader: %u\n", (uint8_t)buffer[0]);

        //     // if ((uint8_t)buffer[0] == (uint8_t)eHeaderCode::config)
        //     // {
        //     //     memcpy(&packetLog.data, buffer, sizeof(packetLog.data));
        //     //     printf("---\n");
        //     //     printf("msg: %s\n", packetLog.msg.msg);
        //     // }
        //     // else if ((uint8_t)buffer[0] == (uint8_t)eHeaderCode::publisher)
        //     // {
        //     //     memcpy(&packetPub.data, buffer, sizeof(packetPub.data));
        //     //     printf("---\n");
        //     //     printf("speed: %f\n", packetPub.msg.speed);
        //     //     printf("status: %i\n", packetPub.msg.status);
        //     // }
        //     // else if ((uint8_t)buffer[0] == (uint8_t)eHeaderCode::publisher + 1u)
        //     // {
        //     //     memcpy(&packetPub.data, buffer, sizeof(packetPub.data));
        //     //     printf("---\n");
        //     //     printf("speed2: %f\n", packetPub.msg.speed);
        //     //     printf("status2: %i\n", packetPub.msg.status);
        //     // }
        //     // else
        //     // {
        //     //     printf("Unknown header, dropping\n");
        //     // }
        // }
    }

    return 0;
}
