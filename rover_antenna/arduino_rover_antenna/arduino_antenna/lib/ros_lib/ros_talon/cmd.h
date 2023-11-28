#ifndef _ROS_ros_talon_cmd_h
#define _ROS_ros_talon_cmd_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ros_talon
{

  class cmd : public ros::Msg
  {
    public:
      typedef int32_t _cmd_mode_type;
      _cmd_mode_type cmd_mode;
      typedef float _cmd_type;
      _cmd_type cmd;

    cmd():
      cmd_mode(0),
      cmd(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_cmd_mode;
      u_cmd_mode.real = this->cmd_mode;
      *(outbuffer + offset + 0) = (u_cmd_mode.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cmd_mode.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cmd_mode.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cmd_mode.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cmd_mode);
      union {
        float real;
        uint32_t base;
      } u_cmd;
      u_cmd.real = this->cmd;
      *(outbuffer + offset + 0) = (u_cmd.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cmd.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cmd.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cmd.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cmd);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_cmd_mode;
      u_cmd_mode.base = 0;
      u_cmd_mode.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cmd_mode.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cmd_mode.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cmd_mode.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cmd_mode = u_cmd_mode.real;
      offset += sizeof(this->cmd_mode);
      union {
        float real;
        uint32_t base;
      } u_cmd;
      u_cmd.base = 0;
      u_cmd.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cmd.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cmd.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cmd.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cmd = u_cmd.real;
      offset += sizeof(this->cmd);
     return offset;
    }

    virtual const char * getType() override { return "ros_talon/cmd"; };
    virtual const char * getMD5() override { return "456022181ca8095f3679b61aaa920211"; };

  };

}
#endif
