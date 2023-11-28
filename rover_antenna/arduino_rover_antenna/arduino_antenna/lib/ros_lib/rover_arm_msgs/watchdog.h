#ifndef _ROS_rover_arm_msgs_watchdog_h
#define _ROS_rover_arm_msgs_watchdog_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rover_arm_msgs
{

  class watchdog : public ros::Msg
  {
    public:
      typedef bool _connected_type;
      _connected_type connected;

    watchdog():
      connected(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_connected;
      u_connected.real = this->connected;
      *(outbuffer + offset + 0) = (u_connected.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->connected);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_connected;
      u_connected.base = 0;
      u_connected.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->connected = u_connected.real;
      offset += sizeof(this->connected);
     return offset;
    }

    virtual const char * getType() override { return "rover_arm_msgs/watchdog"; };
    virtual const char * getMD5() override { return "e0cdaf65159c7f3563426650fb8d3f64"; };

  };

}
#endif
