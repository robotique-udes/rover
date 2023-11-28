#ifndef _ROS_robotnik_msgs_MotorHeadingOffset_h
#define _ROS_robotnik_msgs_MotorHeadingOffset_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robotnik_msgs
{

  class MotorHeadingOffset : public ros::Msg
  {
    public:
      typedef int32_t _motor_type;
      _motor_type motor;
      typedef float _value_type;
      _value_type value;

    MotorHeadingOffset():
      motor(0),
      value(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_motor;
      u_motor.real = this->motor;
      *(outbuffer + offset + 0) = (u_motor.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_motor.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_motor.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_motor.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->motor);
      offset += serializeAvrFloat64(outbuffer + offset, this->value);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_motor;
      u_motor.base = 0;
      u_motor.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_motor.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_motor.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_motor.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->motor = u_motor.real;
      offset += sizeof(this->motor);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->value));
     return offset;
    }

    virtual const char * getType() override { return "robotnik_msgs/MotorHeadingOffset"; };
    virtual const char * getMD5() override { return "8f9a9c9e1eb9b64236a3a4e805aa730d"; };

  };

}
#endif
