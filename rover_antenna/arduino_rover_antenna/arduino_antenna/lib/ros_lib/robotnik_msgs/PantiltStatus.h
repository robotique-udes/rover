#ifndef _ROS_robotnik_msgs_PantiltStatus_h
#define _ROS_robotnik_msgs_PantiltStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robotnik_msgs
{

  class PantiltStatus : public ros::Msg
  {
    public:
      typedef float _pan_pos_type;
      _pan_pos_type pan_pos;
      typedef float _tilt_pos_type;
      _tilt_pos_type tilt_pos;
      typedef float _pan_speed_type;
      _pan_speed_type pan_speed;
      typedef float _tilt_speed_type;
      _tilt_speed_type tilt_speed;

    PantiltStatus():
      pan_pos(0),
      tilt_pos(0),
      pan_speed(0),
      tilt_speed(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_pan_pos;
      u_pan_pos.real = this->pan_pos;
      *(outbuffer + offset + 0) = (u_pan_pos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pan_pos.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pan_pos.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pan_pos.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pan_pos);
      union {
        float real;
        uint32_t base;
      } u_tilt_pos;
      u_tilt_pos.real = this->tilt_pos;
      *(outbuffer + offset + 0) = (u_tilt_pos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_tilt_pos.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_tilt_pos.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_tilt_pos.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tilt_pos);
      union {
        float real;
        uint32_t base;
      } u_pan_speed;
      u_pan_speed.real = this->pan_speed;
      *(outbuffer + offset + 0) = (u_pan_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pan_speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pan_speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pan_speed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pan_speed);
      union {
        float real;
        uint32_t base;
      } u_tilt_speed;
      u_tilt_speed.real = this->tilt_speed;
      *(outbuffer + offset + 0) = (u_tilt_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_tilt_speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_tilt_speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_tilt_speed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tilt_speed);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_pan_pos;
      u_pan_pos.base = 0;
      u_pan_pos.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pan_pos.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pan_pos.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pan_pos.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pan_pos = u_pan_pos.real;
      offset += sizeof(this->pan_pos);
      union {
        float real;
        uint32_t base;
      } u_tilt_pos;
      u_tilt_pos.base = 0;
      u_tilt_pos.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_tilt_pos.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_tilt_pos.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_tilt_pos.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->tilt_pos = u_tilt_pos.real;
      offset += sizeof(this->tilt_pos);
      union {
        float real;
        uint32_t base;
      } u_pan_speed;
      u_pan_speed.base = 0;
      u_pan_speed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pan_speed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pan_speed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pan_speed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pan_speed = u_pan_speed.real;
      offset += sizeof(this->pan_speed);
      union {
        float real;
        uint32_t base;
      } u_tilt_speed;
      u_tilt_speed.base = 0;
      u_tilt_speed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_tilt_speed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_tilt_speed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_tilt_speed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->tilt_speed = u_tilt_speed.real;
      offset += sizeof(this->tilt_speed);
     return offset;
    }

    virtual const char * getType() override { return "robotnik_msgs/PantiltStatus"; };
    virtual const char * getMD5() override { return "b9addaf9ff55168a01a8a6a9e689829f"; };

  };

}
#endif
