#ifndef _ROS_rover_arm_msgs_arm_gui_cmd_h
#define _ROS_rover_arm_msgs_arm_gui_cmd_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rover_arm_msgs
{

  class arm_gui_cmd : public ros::Msg
  {
    public:
      bool enable[4];
      typedef uint8_t _state_type;
      _state_type state;
      typedef bool _jog_is_cartesian_type;
      _jog_is_cartesian_type jog_is_cartesian;

    arm_gui_cmd():
      enable(),
      state(0),
      jog_is_cartesian(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 4; i++){
      union {
        bool real;
        uint8_t base;
      } u_enablei;
      u_enablei.real = this->enable[i];
      *(outbuffer + offset + 0) = (u_enablei.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->enable[i]);
      }
      *(outbuffer + offset + 0) = (this->state >> (8 * 0)) & 0xFF;
      offset += sizeof(this->state);
      union {
        bool real;
        uint8_t base;
      } u_jog_is_cartesian;
      u_jog_is_cartesian.real = this->jog_is_cartesian;
      *(outbuffer + offset + 0) = (u_jog_is_cartesian.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->jog_is_cartesian);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 4; i++){
      union {
        bool real;
        uint8_t base;
      } u_enablei;
      u_enablei.base = 0;
      u_enablei.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->enable[i] = u_enablei.real;
      offset += sizeof(this->enable[i]);
      }
      this->state =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->state);
      union {
        bool real;
        uint8_t base;
      } u_jog_is_cartesian;
      u_jog_is_cartesian.base = 0;
      u_jog_is_cartesian.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->jog_is_cartesian = u_jog_is_cartesian.real;
      offset += sizeof(this->jog_is_cartesian);
     return offset;
    }

    virtual const char * getType() override { return "rover_arm_msgs/arm_gui_cmd"; };
    virtual const char * getMD5() override { return "a775524dacc88feef94b826cd8fd9597"; };

  };

}
#endif
