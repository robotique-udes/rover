#ifndef _ROS_robotnik_msgs_ElevatorStatus_h
#define _ROS_robotnik_msgs_ElevatorStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robotnik_msgs
{

  class ElevatorStatus : public ros::Msg
  {
    public:
      typedef const char* _state_type;
      _state_type state;
      typedef const char* _position_type;
      _position_type position;
      typedef float _height_type;
      _height_type height;
      enum { RAISING = raising };
      enum { LOWERING = lowering };
      enum { IDLE = idle };
      enum { ERROR_G_IO = error_getting_io };
      enum { ERROR_S_IO = error_setting_io };
      enum { ERROR_TIMEOUT = error_timeout_in_action };
      enum { UP = up };
      enum { DOWN = down };
      enum { UNKNOWN = unknown };

    ElevatorStatus():
      state(""),
      position(""),
      height(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_state = strlen(this->state);
      varToArr(outbuffer + offset, length_state);
      offset += 4;
      memcpy(outbuffer + offset, this->state, length_state);
      offset += length_state;
      uint32_t length_position = strlen(this->position);
      varToArr(outbuffer + offset, length_position);
      offset += 4;
      memcpy(outbuffer + offset, this->position, length_position);
      offset += length_position;
      union {
        float real;
        uint32_t base;
      } u_height;
      u_height.real = this->height;
      *(outbuffer + offset + 0) = (u_height.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_height.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_height.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_height.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->height);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_state;
      arrToVar(length_state, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_state; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_state-1]=0;
      this->state = (char *)(inbuffer + offset-1);
      offset += length_state;
      uint32_t length_position;
      arrToVar(length_position, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_position; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_position-1]=0;
      this->position = (char *)(inbuffer + offset-1);
      offset += length_position;
      union {
        float real;
        uint32_t base;
      } u_height;
      u_height.base = 0;
      u_height.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_height.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_height.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_height.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->height = u_height.real;
      offset += sizeof(this->height);
     return offset;
    }

    virtual const char * getType() override { return "robotnik_msgs/ElevatorStatus"; };
    virtual const char * getMD5() override { return "4718bff9866c729031e595e6565fdb82"; };

  };

}
#endif
