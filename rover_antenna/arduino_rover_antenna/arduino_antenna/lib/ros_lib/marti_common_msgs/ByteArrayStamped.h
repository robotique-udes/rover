#ifndef _ROS_marti_common_msgs_ByteArrayStamped_h
#define _ROS_marti_common_msgs_ByteArrayStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace marti_common_msgs
{

  class ByteArrayStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t value_length;
      typedef int8_t _value_type;
      _value_type st_value;
      _value_type * value;

    ByteArrayStamped():
      header(),
      value_length(0), st_value(), value(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->value_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->value_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->value_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->value_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->value_length);
      for( uint32_t i = 0; i < value_length; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_valuei;
      u_valuei.real = this->value[i];
      *(outbuffer + offset + 0) = (u_valuei.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->value[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t value_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      value_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      value_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      value_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->value_length);
      if(value_lengthT > value_length)
        this->value = (int8_t*)realloc(this->value, value_lengthT * sizeof(int8_t));
      value_length = value_lengthT;
      for( uint32_t i = 0; i < value_length; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_st_value;
      u_st_value.base = 0;
      u_st_value.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->st_value = u_st_value.real;
      offset += sizeof(this->st_value);
        memcpy( &(this->value[i]), &(this->st_value), sizeof(int8_t));
      }
     return offset;
    }

    virtual const char * getType() override { return "marti_common_msgs/ByteArrayStamped"; };
    virtual const char * getMD5() override { return "375ed7aa29ecfbdffa16b36b36760a28"; };

  };

}
#endif
