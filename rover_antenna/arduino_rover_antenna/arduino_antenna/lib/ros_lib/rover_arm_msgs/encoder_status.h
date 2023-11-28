#ifndef _ROS_rover_arm_msgs_encoder_status_h
#define _ROS_rover_arm_msgs_encoder_status_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rover_arm_msgs
{

  class encoder_status : public ros::Msg
  {
    public:
      typedef int8_t _state_type;
      _state_type state;
      enum { SIGNAL_TOO_LOW =  -1 };
      enum { SIGNAL_OK =  0 };
      enum { SIGNAL_TOO_HIGH =  1 };

    encoder_status():
      state(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_state;
      u_state.real = this->state;
      *(outbuffer + offset + 0) = (u_state.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->state);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_state;
      u_state.base = 0;
      u_state.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->state = u_state.real;
      offset += sizeof(this->state);
     return offset;
    }

    virtual const char * getType() override { return "rover_arm_msgs/encoder_status"; };
    virtual const char * getMD5() override { return "967ef243f44d2c37a0f23b92054c6a45"; };

  };

}
#endif
