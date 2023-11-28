#ifndef _ROS_SERVICE_set_ptz_h
#define _ROS_SERVICE_set_ptz_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robotnik_msgs
{

static const char SET_PTZ[] = "robotnik_msgs/set_ptz";

  class set_ptzRequest : public ros::Msg
  {
    public:
      typedef float _pan_type;
      _pan_type pan;
      typedef float _tilt_type;
      _tilt_type tilt;
      typedef float _zoom_type;
      _zoom_type zoom;
      typedef bool _relative_type;
      _relative_type relative;
      typedef const char* _mode_type;
      _mode_type mode;

    set_ptzRequest():
      pan(0),
      tilt(0),
      zoom(0),
      relative(0),
      mode("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_pan;
      u_pan.real = this->pan;
      *(outbuffer + offset + 0) = (u_pan.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pan.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pan.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pan.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pan);
      union {
        float real;
        uint32_t base;
      } u_tilt;
      u_tilt.real = this->tilt;
      *(outbuffer + offset + 0) = (u_tilt.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_tilt.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_tilt.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_tilt.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tilt);
      union {
        float real;
        uint32_t base;
      } u_zoom;
      u_zoom.real = this->zoom;
      *(outbuffer + offset + 0) = (u_zoom.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_zoom.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_zoom.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_zoom.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->zoom);
      union {
        bool real;
        uint8_t base;
      } u_relative;
      u_relative.real = this->relative;
      *(outbuffer + offset + 0) = (u_relative.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->relative);
      uint32_t length_mode = strlen(this->mode);
      varToArr(outbuffer + offset, length_mode);
      offset += 4;
      memcpy(outbuffer + offset, this->mode, length_mode);
      offset += length_mode;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_pan;
      u_pan.base = 0;
      u_pan.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pan.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pan.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pan.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pan = u_pan.real;
      offset += sizeof(this->pan);
      union {
        float real;
        uint32_t base;
      } u_tilt;
      u_tilt.base = 0;
      u_tilt.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_tilt.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_tilt.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_tilt.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->tilt = u_tilt.real;
      offset += sizeof(this->tilt);
      union {
        float real;
        uint32_t base;
      } u_zoom;
      u_zoom.base = 0;
      u_zoom.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_zoom.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_zoom.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_zoom.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->zoom = u_zoom.real;
      offset += sizeof(this->zoom);
      union {
        bool real;
        uint8_t base;
      } u_relative;
      u_relative.base = 0;
      u_relative.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->relative = u_relative.real;
      offset += sizeof(this->relative);
      uint32_t length_mode;
      arrToVar(length_mode, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_mode; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_mode-1]=0;
      this->mode = (char *)(inbuffer + offset-1);
      offset += length_mode;
     return offset;
    }

    virtual const char * getType() override { return SET_PTZ; };
    virtual const char * getMD5() override { return "bdbbdf55b26c69882a03baf1effe8bc4"; };

  };

  class set_ptzResponse : public ros::Msg
  {
    public:
      typedef bool _ret_type;
      _ret_type ret;

    set_ptzResponse():
      ret(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_ret;
      u_ret.real = this->ret;
      *(outbuffer + offset + 0) = (u_ret.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ret);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_ret;
      u_ret.base = 0;
      u_ret.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ret = u_ret.real;
      offset += sizeof(this->ret);
     return offset;
    }

    virtual const char * getType() override { return SET_PTZ; };
    virtual const char * getMD5() override { return "e2cc9e9d8c464550830df49c160979ad"; };

  };

  class set_ptz {
    public:
    typedef set_ptzRequest Request;
    typedef set_ptzResponse Response;
  };

}
#endif
