#ifndef _ROS_rover_arm_msgs_PID_h
#define _ROS_rover_arm_msgs_PID_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rover_arm_msgs
{

  class PID : public ros::Msg
  {
    public:
      typedef float _kP_type;
      _kP_type kP;
      typedef float _kI_type;
      _kI_type kI;
      typedef float _kD_type;
      _kD_type kD;

    PID():
      kP(0),
      kI(0),
      kD(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_kP;
      u_kP.real = this->kP;
      *(outbuffer + offset + 0) = (u_kP.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kP.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kP.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kP.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->kP);
      union {
        float real;
        uint32_t base;
      } u_kI;
      u_kI.real = this->kI;
      *(outbuffer + offset + 0) = (u_kI.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kI.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kI.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kI.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->kI);
      union {
        float real;
        uint32_t base;
      } u_kD;
      u_kD.real = this->kD;
      *(outbuffer + offset + 0) = (u_kD.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kD.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kD.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kD.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->kD);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_kP;
      u_kP.base = 0;
      u_kP.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_kP.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_kP.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_kP.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->kP = u_kP.real;
      offset += sizeof(this->kP);
      union {
        float real;
        uint32_t base;
      } u_kI;
      u_kI.base = 0;
      u_kI.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_kI.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_kI.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_kI.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->kI = u_kI.real;
      offset += sizeof(this->kI);
      union {
        float real;
        uint32_t base;
      } u_kD;
      u_kD.base = 0;
      u_kD.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_kD.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_kD.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_kD.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->kD = u_kD.real;
      offset += sizeof(this->kD);
     return offset;
    }

    virtual const char * getType() override { return "rover_arm_msgs/PID"; };
    virtual const char * getMD5() override { return "f75ee6d037776c524707680ad5e639ed"; };

  };

}
#endif
