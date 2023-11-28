#ifndef _ROS_SERVICE_fps_filter_h
#define _ROS_SERVICE_fps_filter_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rover_cam
{

static const char FPS_FILTER[] = "rover_cam/fps_filter";

  class fps_filterRequest : public ros::Msg
  {
    public:
      typedef float _fps_type;
      _fps_type fps;
      typedef bool _active_type;
      _active_type active;

    fps_filterRequest():
      fps(0),
      active(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_fps;
      u_fps.real = this->fps;
      *(outbuffer + offset + 0) = (u_fps.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_fps.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_fps.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_fps.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fps);
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
        float real;
        uint32_t base;
      } u_fps;
      u_fps.base = 0;
      u_fps.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_fps.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_fps.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_fps.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->fps = u_fps.real;
      offset += sizeof(this->fps);
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

    virtual const char * getType() override { return FPS_FILTER; };
    virtual const char * getMD5() override { return "13e371eaa35eba0f744d81246c682453"; };

  };

  class fps_filterResponse : public ros::Msg
  {
    public:
      typedef bool _result_type;
      _result_type result;

    fps_filterResponse():
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

    virtual const char * getType() override { return FPS_FILTER; };
    virtual const char * getMD5() override { return "eb13ac1f1354ccecb7941ee8fa2192e8"; };

  };

  class fps_filter {
    public:
    typedef fps_filterRequest Request;
    typedef fps_filterResponse Response;
  };

}
#endif
