#ifndef _ROS_SERVICE_SetPID_h
#define _ROS_SERVICE_SetPID_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ros_talon
{

static const char SETPID[] = "ros_talon/SetPID";

  class SetPIDRequest : public ros::Msg
  {
    public:
      typedef float _kp_type;
      _kp_type kp;
      typedef float _ki_type;
      _ki_type ki;
      typedef float _kd_type;
      _kd_type kd;
      typedef float _kf_type;
      _kf_type kf;

    SetPIDRequest():
      kp(0),
      ki(0),
      kd(0),
      kf(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_kp;
      u_kp.real = this->kp;
      *(outbuffer + offset + 0) = (u_kp.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kp.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kp.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kp.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->kp);
      union {
        float real;
        uint32_t base;
      } u_ki;
      u_ki.real = this->ki;
      *(outbuffer + offset + 0) = (u_ki.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ki.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ki.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ki.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ki);
      union {
        float real;
        uint32_t base;
      } u_kd;
      u_kd.real = this->kd;
      *(outbuffer + offset + 0) = (u_kd.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kd.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kd.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kd.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->kd);
      union {
        float real;
        uint32_t base;
      } u_kf;
      u_kf.real = this->kf;
      *(outbuffer + offset + 0) = (u_kf.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kf.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kf.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kf.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->kf);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_kp;
      u_kp.base = 0;
      u_kp.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_kp.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_kp.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_kp.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->kp = u_kp.real;
      offset += sizeof(this->kp);
      union {
        float real;
        uint32_t base;
      } u_ki;
      u_ki.base = 0;
      u_ki.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ki.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ki.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ki.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ki = u_ki.real;
      offset += sizeof(this->ki);
      union {
        float real;
        uint32_t base;
      } u_kd;
      u_kd.base = 0;
      u_kd.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_kd.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_kd.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_kd.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->kd = u_kd.real;
      offset += sizeof(this->kd);
      union {
        float real;
        uint32_t base;
      } u_kf;
      u_kf.base = 0;
      u_kf.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_kf.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_kf.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_kf.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->kf = u_kf.real;
      offset += sizeof(this->kf);
     return offset;
    }

    virtual const char * getType() override { return SETPID; };
    virtual const char * getMD5() override { return "3e9f6b0a285b7a51add516f834dff068"; };

  };

  class SetPIDResponse : public ros::Msg
  {
    public:

    SetPIDResponse()
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

    virtual const char * getType() override { return SETPID; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class SetPID {
    public:
    typedef SetPIDRequest Request;
    typedef SetPIDResponse Response;
  };

}
#endif
