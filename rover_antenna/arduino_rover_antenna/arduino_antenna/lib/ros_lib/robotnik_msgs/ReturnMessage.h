#ifndef _ROS_robotnik_msgs_ReturnMessage_h
#define _ROS_robotnik_msgs_ReturnMessage_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robotnik_msgs
{

  class ReturnMessage : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;
      typedef const char* _message_type;
      _message_type message;
      typedef int16_t _code_type;
      _code_type code;

    ReturnMessage():
      success(0),
      message(""),
      code(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      uint32_t length_message = strlen(this->message);
      varToArr(outbuffer + offset, length_message);
      offset += 4;
      memcpy(outbuffer + offset, this->message, length_message);
      offset += length_message;
      union {
        int16_t real;
        uint16_t base;
      } u_code;
      u_code.real = this->code;
      *(outbuffer + offset + 0) = (u_code.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_code.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->code);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
      uint32_t length_message;
      arrToVar(length_message, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_message; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_message-1]=0;
      this->message = (char *)(inbuffer + offset-1);
      offset += length_message;
      union {
        int16_t real;
        uint16_t base;
      } u_code;
      u_code.base = 0;
      u_code.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_code.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->code = u_code.real;
      offset += sizeof(this->code);
     return offset;
    }

    virtual const char * getType() override { return "robotnik_msgs/ReturnMessage"; };
    virtual const char * getMD5() override { return "797637cd4b4f3860dff5d9fefb9b58f4"; };

  };

}
#endif
