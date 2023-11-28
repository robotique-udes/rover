#ifndef _ROS_marti_nav_msgs_VehicleParameters_h
#define _ROS_marti_nav_msgs_VehicleParameters_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace marti_nav_msgs
{

  class VehicleParameters : public ros::Msg
  {
    public:
      typedef float _max_curvature_type;
      _max_curvature_type max_curvature;
      typedef float _min_curvature_type;
      _min_curvature_type min_curvature;
      typedef float _max_lateral_acceleration_type;
      _max_lateral_acceleration_type max_lateral_acceleration;
      typedef float _target_lateral_acceleration_type;
      _target_lateral_acceleration_type target_lateral_acceleration;
      typedef float _max_acceleration_type;
      _max_acceleration_type max_acceleration;
      typedef float _target_acceleration_type;
      _target_acceleration_type target_acceleration;
      typedef float _max_deceleration_type;
      _max_deceleration_type max_deceleration;
      typedef float _target_deceleration_type;
      _target_deceleration_type target_deceleration;
      typedef float _stop_deceleration_type;
      _stop_deceleration_type stop_deceleration;

    VehicleParameters():
      max_curvature(0),
      min_curvature(0),
      max_lateral_acceleration(0),
      target_lateral_acceleration(0),
      max_acceleration(0),
      target_acceleration(0),
      max_deceleration(0),
      target_deceleration(0),
      stop_deceleration(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_max_curvature;
      u_max_curvature.real = this->max_curvature;
      *(outbuffer + offset + 0) = (u_max_curvature.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_curvature.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_curvature.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_curvature.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max_curvature);
      union {
        float real;
        uint32_t base;
      } u_min_curvature;
      u_min_curvature.real = this->min_curvature;
      *(outbuffer + offset + 0) = (u_min_curvature.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_min_curvature.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_min_curvature.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_min_curvature.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->min_curvature);
      union {
        float real;
        uint32_t base;
      } u_max_lateral_acceleration;
      u_max_lateral_acceleration.real = this->max_lateral_acceleration;
      *(outbuffer + offset + 0) = (u_max_lateral_acceleration.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_lateral_acceleration.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_lateral_acceleration.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_lateral_acceleration.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max_lateral_acceleration);
      union {
        float real;
        uint32_t base;
      } u_target_lateral_acceleration;
      u_target_lateral_acceleration.real = this->target_lateral_acceleration;
      *(outbuffer + offset + 0) = (u_target_lateral_acceleration.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_target_lateral_acceleration.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_target_lateral_acceleration.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_target_lateral_acceleration.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->target_lateral_acceleration);
      union {
        float real;
        uint32_t base;
      } u_max_acceleration;
      u_max_acceleration.real = this->max_acceleration;
      *(outbuffer + offset + 0) = (u_max_acceleration.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_acceleration.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_acceleration.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_acceleration.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max_acceleration);
      union {
        float real;
        uint32_t base;
      } u_target_acceleration;
      u_target_acceleration.real = this->target_acceleration;
      *(outbuffer + offset + 0) = (u_target_acceleration.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_target_acceleration.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_target_acceleration.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_target_acceleration.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->target_acceleration);
      union {
        float real;
        uint32_t base;
      } u_max_deceleration;
      u_max_deceleration.real = this->max_deceleration;
      *(outbuffer + offset + 0) = (u_max_deceleration.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_deceleration.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_deceleration.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_deceleration.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max_deceleration);
      union {
        float real;
        uint32_t base;
      } u_target_deceleration;
      u_target_deceleration.real = this->target_deceleration;
      *(outbuffer + offset + 0) = (u_target_deceleration.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_target_deceleration.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_target_deceleration.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_target_deceleration.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->target_deceleration);
      union {
        float real;
        uint32_t base;
      } u_stop_deceleration;
      u_stop_deceleration.real = this->stop_deceleration;
      *(outbuffer + offset + 0) = (u_stop_deceleration.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_stop_deceleration.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_stop_deceleration.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_stop_deceleration.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stop_deceleration);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_max_curvature;
      u_max_curvature.base = 0;
      u_max_curvature.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_curvature.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_curvature.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_curvature.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->max_curvature = u_max_curvature.real;
      offset += sizeof(this->max_curvature);
      union {
        float real;
        uint32_t base;
      } u_min_curvature;
      u_min_curvature.base = 0;
      u_min_curvature.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_min_curvature.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_min_curvature.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_min_curvature.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->min_curvature = u_min_curvature.real;
      offset += sizeof(this->min_curvature);
      union {
        float real;
        uint32_t base;
      } u_max_lateral_acceleration;
      u_max_lateral_acceleration.base = 0;
      u_max_lateral_acceleration.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_lateral_acceleration.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_lateral_acceleration.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_lateral_acceleration.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->max_lateral_acceleration = u_max_lateral_acceleration.real;
      offset += sizeof(this->max_lateral_acceleration);
      union {
        float real;
        uint32_t base;
      } u_target_lateral_acceleration;
      u_target_lateral_acceleration.base = 0;
      u_target_lateral_acceleration.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_target_lateral_acceleration.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_target_lateral_acceleration.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_target_lateral_acceleration.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->target_lateral_acceleration = u_target_lateral_acceleration.real;
      offset += sizeof(this->target_lateral_acceleration);
      union {
        float real;
        uint32_t base;
      } u_max_acceleration;
      u_max_acceleration.base = 0;
      u_max_acceleration.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_acceleration.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_acceleration.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_acceleration.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->max_acceleration = u_max_acceleration.real;
      offset += sizeof(this->max_acceleration);
      union {
        float real;
        uint32_t base;
      } u_target_acceleration;
      u_target_acceleration.base = 0;
      u_target_acceleration.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_target_acceleration.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_target_acceleration.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_target_acceleration.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->target_acceleration = u_target_acceleration.real;
      offset += sizeof(this->target_acceleration);
      union {
        float real;
        uint32_t base;
      } u_max_deceleration;
      u_max_deceleration.base = 0;
      u_max_deceleration.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_deceleration.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_deceleration.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_deceleration.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->max_deceleration = u_max_deceleration.real;
      offset += sizeof(this->max_deceleration);
      union {
        float real;
        uint32_t base;
      } u_target_deceleration;
      u_target_deceleration.base = 0;
      u_target_deceleration.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_target_deceleration.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_target_deceleration.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_target_deceleration.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->target_deceleration = u_target_deceleration.real;
      offset += sizeof(this->target_deceleration);
      union {
        float real;
        uint32_t base;
      } u_stop_deceleration;
      u_stop_deceleration.base = 0;
      u_stop_deceleration.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_stop_deceleration.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_stop_deceleration.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_stop_deceleration.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->stop_deceleration = u_stop_deceleration.real;
      offset += sizeof(this->stop_deceleration);
     return offset;
    }

    virtual const char * getType() override { return "marti_nav_msgs/VehicleParameters"; };
    virtual const char * getMD5() override { return "f32b5011f4c0796a9b5689959dcf384a"; };

  };

}
#endif
