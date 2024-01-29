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
    bool status;
    float speed;
} rover_msgs__msg__AntennaCmd;

union rover_msgs__msg__AntennaCmd__packet
{
    rover_msgs__msg__AntennaCmd msg;
    uint8_t data[sizeof(rover_msgs__msg__AntennaCmd)];
};

void clearBuffer(char *buffer, uint16_t size);

int main(/*int argc, char *argv[]*/ void)
{
    int serial_port = open("/dev/ttyACM0", O_RDWR);

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

    rover_msgs__msg__AntennaCmd__packet packet;
    for (;;)
    {
        int length = read(serial_port, &packet.data, sizeof(packet.data));

        if (length < 0)
        {
            printf("Error\n");
        }
        else if (length == 0)
        {
            // printf("No data\n");
        }
        else
        {
            printf("speed: %f\n", packet.msg.speed);
            printf("status: %i\n", packet.msg.status);
        }
    }

    return 0;
}

void clearBuffer(char *buffer, uint16_t size)
{
    for (int i = 0; i < size; i++)
    {
        buffer[i] = '\0';
    }
}
