#ifndef _ROS_SERVICE_SetExposure_h
#define _ROS_SERVICE_SetExposure_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace marti_sensor_msgs
{

static const char SETEXPOSURE[] = "marti_sensor_msgs/SetExposure";

  class SetExposureRequest : public ros::Msg
  {
    public:
      typedef bool _auto_exposure_type;
      _auto_exposure_type auto_exposure;
      typedef int64_t _time_type;
      _time_type time;

    SetExposureRequest():
      auto_exposure(0),
      time(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_auto_exposure;
      u_auto_exposure.real = this->auto_exposure;
      *(outbuffer + offset + 0) = (u_auto_exposure.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->auto_exposure);
      union {
        int64_t real;
        uint64_t base;
      } u_time;
      u_time.real = this->time;
      *(outbuffer + offset + 0) = (u_time.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_time.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_time.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_time.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_time.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_time.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_time.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_time.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->time);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_auto_exposure;
      u_auto_exposure.base = 0;
      u_auto_exposure.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->auto_exposure = u_auto_exposure.real;
      offset += sizeof(this->auto_exposure);
      union {
        int64_t real;
        uint64_t base;
      } u_time;
      u_time.base = 0;
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->time = u_time.real;
      offset += sizeof(this->time);
     return offset;
    }

    virtual const char * getType() override { return SETEXPOSURE; };
    virtual const char * getMD5() override { return "a226e84ef4a44363d3b289536b8589a5"; };

  };

  class SetExposureResponse : public ros::Msg
  {
    public:
      typedef bool _auto_exposure_type;
      _auto_exposure_type auto_exposure;
      typedef int64_t _time_type;
      _time_type time;

    SetExposureResponse():
      auto_exposure(0),
      time(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_auto_exposure;
      u_auto_exposure.real = this->auto_exposure;
      *(outbuffer + offset + 0) = (u_auto_exposure.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->auto_exposure);
      union {
        int64_t real;
        uint64_t base;
      } u_time;
      u_time.real = this->time;
      *(outbuffer + offset + 0) = (u_time.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_time.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_time.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_time.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_time.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_time.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_time.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_time.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->time);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_auto_exposure;
      u_auto_exposure.base = 0;
      u_auto_exposure.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->auto_exposure = u_auto_exposure.real;
      offset += sizeof(this->auto_exposure);
      union {
        int64_t real;
        uint64_t base;
      } u_time;
      u_time.base = 0;
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->time = u_time.real;
      offset += sizeof(this->time);
     return offset;
    }

    virtual const char * getType() override { return SETEXPOSURE; };
    virtual const char * getMD5() override { return "a226e84ef4a44363d3b289536b8589a5"; };

  };

  class SetExposure {
    public:
    typedef SetExposureRequest Request;
    typedef SetExposureResponse Response;
  };

}
#endif
