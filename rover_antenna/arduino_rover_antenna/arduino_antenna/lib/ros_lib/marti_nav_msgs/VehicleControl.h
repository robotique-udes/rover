#ifndef _ROS_marti_nav_msgs_VehicleControl_h
#define _ROS_marti_nav_msgs_VehicleControl_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace marti_nav_msgs
{

  class VehicleControl : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int32_t _engine_type;
      _engine_type engine;
      typedef int32_t _gear_type;
      _gear_type gear;
      typedef float _steering_type;
      _steering_type steering;
      typedef float _throttle_type;
      _throttle_type throttle;
      typedef float _brake_type;
      _brake_type brake;
      typedef int16_t _steering_position_type;
      _steering_position_type steering_position;
      typedef int16_t _gb_position_type;
      _gb_position_type gb_position;

    VehicleControl():
      header(),
      engine(0),
      gear(0),
      steering(0),
      throttle(0),
      brake(0),
      steering_position(0),
      gb_position(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_engine;
      u_engine.real = this->engine;
      *(outbuffer + offset + 0) = (u_engine.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_engine.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_engine.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_engine.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->engine);
      union {
        int32_t real;
        uint32_t base;
      } u_gear;
      u_gear.real = this->gear;
      *(outbuffer + offset + 0) = (u_gear.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gear.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gear.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gear.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gear);
      offset += serializeAvrFloat64(outbuffer + offset, this->steering);
      offset += serializeAvrFloat64(outbuffer + offset, this->throttle);
      offset += serializeAvrFloat64(outbuffer + offset, this->brake);
      union {
        int16_t real;
        uint16_t base;
      } u_steering_position;
      u_steering_position.real = this->steering_position;
      *(outbuffer + offset + 0) = (u_steering_position.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_steering_position.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->steering_position);
      union {
        int16_t real;
        uint16_t base;
      } u_gb_position;
      u_gb_position.real = this->gb_position;
      *(outbuffer + offset + 0) = (u_gb_position.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gb_position.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->gb_position);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_engine;
      u_engine.base = 0;
      u_engine.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_engine.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_engine.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_engine.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->engine = u_engine.real;
      offset += sizeof(this->engine);
      union {
        int32_t real;
        uint32_t base;
      } u_gear;
      u_gear.base = 0;
      u_gear.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gear.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_gear.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_gear.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->gear = u_gear.real;
      offset += sizeof(this->gear);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->steering));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->throttle));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->brake));
      union {
        int16_t real;
        uint16_t base;
      } u_steering_position;
      u_steering_position.base = 0;
      u_steering_position.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_steering_position.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->steering_position = u_steering_position.real;
      offset += sizeof(this->steering_position);
      union {
        int16_t real;
        uint16_t base;
      } u_gb_position;
      u_gb_position.base = 0;
      u_gb_position.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gb_position.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->gb_position = u_gb_position.real;
      offset += sizeof(this->gb_position);
     return offset;
    }

    virtual const char * getType() override { return "marti_nav_msgs/VehicleControl"; };
    virtual const char * getMD5() override { return "afc6b0b97f62f744f4b35119ffb6e337"; };

  };

}
#endif
