#ifndef _ROS_SERVICE_joint_calib_h
#define _ROS_SERVICE_joint_calib_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rover_arm_msgs
{

static const char JOINT_CALIB[] = "rover_arm_msgs/joint_calib";

  class joint_calibRequest : public ros::Msg
  {
    public:
      typedef float _shift_type;
      _shift_type shift;

    joint_calibRequest():
      shift(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_shift;
      u_shift.real = this->shift;
      *(outbuffer + offset + 0) = (u_shift.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_shift.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_shift.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_shift.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->shift);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_shift;
      u_shift.base = 0;
      u_shift.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_shift.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_shift.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_shift.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->shift = u_shift.real;
      offset += sizeof(this->shift);
     return offset;
    }

    virtual const char * getType() override { return JOINT_CALIB; };
    virtual const char * getMD5() override { return "2086ad0abcb8534c147077515b2b2211"; };

  };

  class joint_calibResponse : public ros::Msg
  {
    public:

    joint_calibResponse()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
     return offset;
    }

    virtual const char * getType() override { return JOINT_CALIB; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class joint_calib {
    public:
    typedef joint_calibRequest Request;
    typedef joint_calibResponse Response;
  };

}
#endif
