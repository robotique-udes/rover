#ifndef _ROS_robotnik_msgs_WatchdogStatus_h
#define _ROS_robotnik_msgs_WatchdogStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robotnik_msgs
{

  class WatchdogStatus : public ros::Msg
  {
    public:
      typedef const char* _id_type;
      _id_type id;
      typedef bool _timed_out_type;
      _timed_out_type timed_out;
      typedef const char* _description_type;
      _description_type description;

    WatchdogStatus():
      id(""),
      timed_out(0),
      description("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_id = strlen(this->id);
      varToArr(outbuffer + offset, length_id);
      offset += 4;
      memcpy(outbuffer + offset, this->id, length_id);
      offset += length_id;
      union {
        bool real;
        uint8_t base;
      } u_timed_out;
      u_timed_out.real = this->timed_out;
      *(outbuffer + offset + 0) = (u_timed_out.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->timed_out);
      uint32_t length_description = strlen(this->description);
      varToArr(outbuffer + offset, length_description);
      offset += 4;
      memcpy(outbuffer + offset, this->description, length_description);
      offset += length_description;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_id;
      arrToVar(length_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_id-1]=0;
      this->id = (char *)(inbuffer + offset-1);
      offset += length_id;
      union {
        bool real;
        uint8_t base;
      } u_timed_out;
      u_timed_out.base = 0;
      u_timed_out.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->timed_out = u_timed_out.real;
      offset += sizeof(this->timed_out);
      uint32_t length_description;
      arrToVar(length_description, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_description; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_description-1]=0;
      this->description = (char *)(inbuffer + offset-1);
      offset += length_description;
     return offset;
    }

    virtual const char * getType() override { return "robotnik_msgs/WatchdogStatus"; };
    virtual const char * getMD5() override { return "6eac41b9a2552a59aabed4f2aef62b65"; };

  };

}
#endif
