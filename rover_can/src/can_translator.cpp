#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

int main(void)
{
    int s;
    sockaddr_can addr;
    ifreq ifr;
    can_frame frame;

    printf("CAN Sockets Demo\r\n");

    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        perror("Socket");
        return 1;
    }

    strcpy(ifr.ifr_name, "can0");
    ioctl(s, SIOCGIFINDEX, &ifr);

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("Bind");
        return 1;
    }

    frame.can_id = 0x21;
    frame.can_dlc = 4;
    frame.data[0] = 0xFA;
    frame.data[1] = 0xFA;
    frame.data[2] = 0xBE;
    frame.data[3] = 0xBE;

    while (true)
    {
        if (write(s, &frame, sizeof(frame)) != sizeof(frame))
        {
            perror("Write");
            return 1;
        }
        else
        {
            printf("packet sent");
        }

        sleep(1);
    }

    if (close(s) < 0)
    {
        perror("Close");
        return 1;
    }

    return 0;
}
