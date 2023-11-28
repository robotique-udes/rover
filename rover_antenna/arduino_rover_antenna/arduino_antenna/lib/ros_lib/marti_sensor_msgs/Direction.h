#ifndef _ROS_marti_sensor_msgs_Direction_h
#define _ROS_marti_sensor_msgs_Direction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace marti_sensor_msgs
{

  class Direction : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int8_t _direction_type;
      _direction_type direction;
      enum { BACKWARD = -1 };
      enum { ZERO = 0 };
      enum { FORWARD = 1 };
      enum { UNKNOWN = 127          };

    Direction():
      header(),
      direction(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int8_t real;
        uint8_t base;
      } u_direction;
      u_direction.real = this->direction;
      *(outbuffer + offset + 0) = (u_direction.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->direction);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int8_t real;
        uint8_t base;
      } u_direction;
      u_direction.base = 0;
      u_direction.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->direction = u_direction.real;
      offset += sizeof(this->direction);
     return offset;
    }

    virtual const char * getType() override { return "marti_sensor_msgs/Direction"; };
    virtual const char * getMD5() override { return "f4fcbfc7e6f35aa53be850d25834349c"; };

  };

}
#endif
