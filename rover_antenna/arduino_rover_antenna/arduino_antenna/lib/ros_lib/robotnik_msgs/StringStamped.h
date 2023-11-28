#ifndef _ROS_robotnik_msgs_StringStamped_h
#define _ROS_robotnik_msgs_StringStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace robotnik_msgs
{

  class StringStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _string_type;
      _string_type string;

    StringStamped():
      header(),
      string("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_string = strlen(this->string);
      varToArr(outbuffer + offset, length_string);
      offset += 4;
      memcpy(outbuffer + offset, this->string, length_string);
      offset += length_string;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_string;
      arrToVar(length_string, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_string; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_string-1]=0;
      this->string = (char *)(inbuffer + offset-1);
      offset += length_string;
     return offset;
    }

    virtual const char * getType() override { return "robotnik_msgs/StringStamped"; };
    virtual const char * getMD5() override { return "5e3e46086181199270f1ac3a28a5977f"; };

  };

}
#endif
