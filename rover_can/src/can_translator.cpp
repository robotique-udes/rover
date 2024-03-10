#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <cstdint>
#include <chrono>
#include <thread>

#include "rover_can_lib/rover_can_lib.hpp"

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

    strcpy(ifr.ifr_name, "canRovus");
    ioctl(s, SIOCGIFINDEX, &ifr);

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("Bind");
        return 1;
    }

    frame.can_id = 0x101;
    frame.can_dlc = 5;
    frame.data[0] = 0x02;

    RoverCanLib::UnionDefinition::FloatUnion test;
    for (float value = -100.0f; value <= 100.0f; value += 0.1f)
    {
        test.data = value;

        for (uint8_t i = 0; i < sizeof(test); i++)
        {
            frame.data[i + 1] = test.dataBytes[i];
        }

        write(s, &frame, sizeof(frame)) != sizeof(frame);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

    }
    if (close(s) < 0)
    {
        perror("Close");
        return 1;
    }

    return 0;
}
