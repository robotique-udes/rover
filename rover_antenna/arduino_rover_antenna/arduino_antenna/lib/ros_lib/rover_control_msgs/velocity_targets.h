#ifndef _ROS_rover_control_msgs_velocity_targets_h
#define _ROS_rover_control_msgs_velocity_targets_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rover_control_msgs
{

  class velocity_targets : public ros::Msg
  {
    public:
      typedef float _left_wheel_vel_target_type;
      _left_wheel_vel_target_type left_wheel_vel_target;
      typedef float _right_wheel_vel_target_type;
      _right_wheel_vel_target_type right_wheel_vel_target;

    velocity_targets():
      left_wheel_vel_target(0),
      right_wheel_vel_target(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_left_wheel_vel_target;
      u_left_wheel_vel_target.real = this->left_wheel_vel_target;
      *(outbuffer + offset + 0) = (u_left_wheel_vel_target.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_wheel_vel_target.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_wheel_vel_target.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_wheel_vel_target.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_wheel_vel_target);
      union {
        float real;
        uint32_t base;
      } u_right_wheel_vel_target;
      u_right_wheel_vel_target.real = this->right_wheel_vel_target;
      *(outbuffer + offset + 0) = (u_right_wheel_vel_target.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_wheel_vel_target.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_wheel_vel_target.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_wheel_vel_target.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_wheel_vel_target);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_left_wheel_vel_target;
      u_left_wheel_vel_target.base = 0;
      u_left_wheel_vel_target.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_wheel_vel_target.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_wheel_vel_target.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_wheel_vel_target.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_wheel_vel_target = u_left_wheel_vel_target.real;
      offset += sizeof(this->left_wheel_vel_target);
      union {
        float real;
        uint32_t base;
      } u_right_wheel_vel_target;
      u_right_wheel_vel_target.base = 0;
      u_right_wheel_vel_target.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_wheel_vel_target.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_wheel_vel_target.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_wheel_vel_target.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_wheel_vel_target = u_right_wheel_vel_target.real;
      offset += sizeof(this->right_wheel_vel_target);
     return offset;
    }

    virtual const char * getType() override { return "rover_control_msgs/velocity_targets"; };
    virtual const char * getMD5() override { return "8ac043b27f8aed2b973ececde8d248f9"; };

  };

}
#endif
