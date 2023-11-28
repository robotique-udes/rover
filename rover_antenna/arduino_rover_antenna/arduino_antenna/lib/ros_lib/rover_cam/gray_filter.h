#ifndef _ROS_SERVICE_gray_filter_h
#define _ROS_SERVICE_gray_filter_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rover_cam
{

static const char GRAY_FILTER[] = "rover_cam/gray_filter";

  class gray_filterRequest : public ros::Msg
  {
    public:
      typedef bool _active_type;
      _active_type active;

    gray_filterRequest():
      active(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_active;
      u_active.real = this->active;
      *(outbuffer + offset + 0) = (u_active.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->active);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_active;
      u_active.base = 0;
      u_active.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->active = u_active.real;
      offset += sizeof(this->active);
     return offset;
    }

    virtual const char * getType() override { return GRAY_FILTER; };
    virtual const char * getMD5() override { return "2c5cfb0a2565df41de6873994aee57d2"; };

  };

  class gray_filterResponse : public ros::Msg
  {
    public:
      typedef bool _result_type;
      _result_type result;

    gray_filterResponse():
      result(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_result;
      u_result.real = this->result;
      *(outbuffer + offset + 0) = (u_result.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->result);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_result;
      u_result.base = 0;
      u_result.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->result = u_result.real;
      offset += sizeof(this->result);
     return offset;
    }

    virtual const char * getType() override { return GRAY_FILTER; };
    virtual const char * getMD5() override { return "eb13ac1f1354ccecb7941ee8fa2192e8"; };

  };

  class gray_filter {
    public:
    typedef gray_filterRequest Request;
    typedef gray_filterResponse Response;
  };

}
#endif
