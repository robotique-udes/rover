#ifndef _ROS_SERVICE_SetEncoderTurns_h
#define _ROS_SERVICE_SetEncoderTurns_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "robotnik_msgs/MotorHeadingOffset.h"

namespace robotnik_msgs
{

static const char SETENCODERTURNS[] = "robotnik_msgs/SetEncoderTurns";

  class SetEncoderTurnsRequest : public ros::Msg
  {
    public:
      typedef robotnik_msgs::MotorHeadingOffset _encoder_turns_type;
      _encoder_turns_type encoder_turns;

    SetEncoderTurnsRequest():
      encoder_turns()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->encoder_turns.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->encoder_turns.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return SETENCODERTURNS; };
    virtual const char * getMD5() override { return "34cf70b52bbe3c3e3567eb0d481c62de"; };

  };

  class SetEncoderTurnsResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;
      typedef const char* _message_type;
      _message_type message;

    SetEncoderTurnsResponse():
      success(0),
      message("")
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
     return offset;
    }

    virtual const char * getType() override { return SETENCODERTURNS; };
    virtual const char * getMD5() override { return "937c9679a518e3a18d831e57125ea522"; };

  };

  class SetEncoderTurns {
    public:
    typedef SetEncoderTurnsRequest Request;
    typedef SetEncoderTurnsResponse Response;
  };

}
#endif
