#ifndef _ROS_robotnik_msgs_ArmStatus_h
#define _ROS_robotnik_msgs_ArmStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robotnik_msgs
{

  class ArmStatus : public ros::Msg
  {
    public:
      typedef const char* _position_type;
      _position_type position;

    ArmStatus():
      position("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_position = strlen(this->position);
      varToArr(outbuffer + offset, length_position);
      offset += 4;
      memcpy(outbuffer + offset, this->position, length_position);
      offset += length_position;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_position;
      arrToVar(length_position, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_position; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_position-1]=0;
      this->position = (char *)(inbuffer + offset-1);
      offset += length_position;
     return offset;
    }

    virtual const char * getType() override { return "robotnik_msgs/ArmStatus"; };
    virtual const char * getMD5() override { return "d6afdd327a64a50f94a7d3a2de5435e3"; };

  };

}
#endif
