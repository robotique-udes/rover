#ifndef _ROS_robotnik_msgs_OdomCalibrationStatus_h
#define _ROS_robotnik_msgs_OdomCalibrationStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robotnik_msgs
{

  class OdomCalibrationStatus : public ros::Msg
  {
    public:
      typedef bool _is_calculating_type;
      _is_calculating_type is_calculating;
      typedef float _remaining_time_type;
      _remaining_time_type remaining_time;
      typedef float _left_radius_type;
      _left_radius_type left_radius;
      typedef float _right_radius_type;
      _right_radius_type right_radius;
      typedef float _wheels_distance_type;
      _wheels_distance_type wheels_distance;

    OdomCalibrationStatus():
      is_calculating(0),
      remaining_time(0),
      left_radius(0),
      right_radius(0),
      wheels_distance(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_is_calculating;
      u_is_calculating.real = this->is_calculating;
      *(outbuffer + offset + 0) = (u_is_calculating.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_calculating);
      union {
        float real;
        uint32_t base;
      } u_remaining_time;
      u_remaining_time.real = this->remaining_time;
      *(outbuffer + offset + 0) = (u_remaining_time.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_remaining_time.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_remaining_time.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_remaining_time.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->remaining_time);
      union {
        float real;
        uint32_t base;
      } u_left_radius;
      u_left_radius.real = this->left_radius;
      *(outbuffer + offset + 0) = (u_left_radius.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_radius.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_radius.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_radius.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_radius);
      union {
        float real;
        uint32_t base;
      } u_right_radius;
      u_right_radius.real = this->right_radius;
      *(outbuffer + offset + 0) = (u_right_radius.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_radius.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_radius.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_radius.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_radius);
      union {
        float real;
        uint32_t base;
      } u_wheels_distance;
      u_wheels_distance.real = this->wheels_distance;
      *(outbuffer + offset + 0) = (u_wheels_distance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_wheels_distance.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_wheels_distance.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_wheels_distance.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->wheels_distance);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_is_calculating;
      u_is_calculating.base = 0;
      u_is_calculating.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_calculating = u_is_calculating.real;
      offset += sizeof(this->is_calculating);
      union {
        float real;
        uint32_t base;
      } u_remaining_time;
      u_remaining_time.base = 0;
      u_remaining_time.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_remaining_time.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_remaining_time.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_remaining_time.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->remaining_time = u_remaining_time.real;
      offset += sizeof(this->remaining_time);
      union {
        float real;
        uint32_t base;
      } u_left_radius;
      u_left_radius.base = 0;
      u_left_radius.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_radius.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_radius.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_radius.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_radius = u_left_radius.real;
      offset += sizeof(this->left_radius);
      union {
        float real;
        uint32_t base;
      } u_right_radius;
      u_right_radius.base = 0;
      u_right_radius.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_radius.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_radius.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_radius.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_radius = u_right_radius.real;
      offset += sizeof(this->right_radius);
      union {
        float real;
        uint32_t base;
      } u_wheels_distance;
      u_wheels_distance.base = 0;
      u_wheels_distance.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_wheels_distance.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_wheels_distance.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_wheels_distance.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->wheels_distance = u_wheels_distance.real;
      offset += sizeof(this->wheels_distance);
     return offset;
    }

    virtual const char * getType() override { return "robotnik_msgs/OdomCalibrationStatus"; };
    virtual const char * getMD5() override { return "d03e7379e89cf43254443e594855bbde"; };

  };

}
#endif
