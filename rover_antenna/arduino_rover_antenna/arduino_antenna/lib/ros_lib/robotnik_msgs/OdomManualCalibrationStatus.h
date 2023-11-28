#ifndef _ROS_robotnik_msgs_OdomManualCalibrationStatus_h
#define _ROS_robotnik_msgs_OdomManualCalibrationStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose.h"

namespace robotnik_msgs
{

  class OdomManualCalibrationStatus : public ros::Msg
  {
    public:
      typedef geometry_msgs::Pose _odom_increment_type;
      _odom_increment_type odom_increment;
      typedef geometry_msgs::Pose _real_increment_type;
      _real_increment_type real_increment;
      typedef float _linear_error_type;
      _linear_error_type linear_error;
      typedef float _angular_error_type;
      _angular_error_type angular_error;

    OdomManualCalibrationStatus():
      odom_increment(),
      real_increment(),
      linear_error(0),
      angular_error(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->odom_increment.serialize(outbuffer + offset);
      offset += this->real_increment.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_linear_error;
      u_linear_error.real = this->linear_error;
      *(outbuffer + offset + 0) = (u_linear_error.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_linear_error.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_linear_error.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_linear_error.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->linear_error);
      union {
        float real;
        uint32_t base;
      } u_angular_error;
      u_angular_error.real = this->angular_error;
      *(outbuffer + offset + 0) = (u_angular_error.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angular_error.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_angular_error.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_angular_error.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angular_error);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->odom_increment.deserialize(inbuffer + offset);
      offset += this->real_increment.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_linear_error;
      u_linear_error.base = 0;
      u_linear_error.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_linear_error.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_linear_error.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_linear_error.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->linear_error = u_linear_error.real;
      offset += sizeof(this->linear_error);
      union {
        float real;
        uint32_t base;
      } u_angular_error;
      u_angular_error.base = 0;
      u_angular_error.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angular_error.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_angular_error.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_angular_error.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->angular_error = u_angular_error.real;
      offset += sizeof(this->angular_error);
     return offset;
    }

    virtual const char * getType() override { return "robotnik_msgs/OdomManualCalibrationStatus"; };
    virtual const char * getMD5() override { return "f77436bc07e4185538807c200cd1cc07"; };

  };

}
#endif
