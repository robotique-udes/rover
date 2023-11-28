#ifndef _ROS_marti_common_msgs_StringArrayStamped_h
#define _ROS_marti_common_msgs_StringArrayStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace marti_common_msgs
{

  class StringArrayStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t strings_length;
      typedef char* _strings_type;
      _strings_type st_strings;
      _strings_type * strings;

    StringArrayStamped():
      header(),
      strings_length(0), st_strings(), strings(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->strings_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->strings_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->strings_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->strings_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->strings_length);
      for( uint32_t i = 0; i < strings_length; i++){
      uint32_t length_stringsi = strlen(this->strings[i]);
      varToArr(outbuffer + offset, length_stringsi);
      offset += 4;
      memcpy(outbuffer + offset, this->strings[i], length_stringsi);
      offset += length_stringsi;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t strings_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      strings_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      strings_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      strings_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->strings_length);
      if(strings_lengthT > strings_length)
        this->strings = (char**)realloc(this->strings, strings_lengthT * sizeof(char*));
      strings_length = strings_lengthT;
      for( uint32_t i = 0; i < strings_length; i++){
      uint32_t length_st_strings;
      arrToVar(length_st_strings, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_strings; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_strings-1]=0;
      this->st_strings = (char *)(inbuffer + offset-1);
      offset += length_st_strings;
        memcpy( &(this->strings[i]), &(this->st_strings), sizeof(char*));
      }
     return offset;
    }

    virtual const char * getType() override { return "marti_common_msgs/StringArrayStamped"; };
    virtual const char * getMD5() override { return "e8d1c28bfef374ffc40e1645c020607f"; };

  };

}
#endif
