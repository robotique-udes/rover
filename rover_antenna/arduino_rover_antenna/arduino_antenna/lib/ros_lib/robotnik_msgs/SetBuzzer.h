#ifndef _ROS_SERVICE_SetBuzzer_h
#define _ROS_SERVICE_SetBuzzer_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robotnik_msgs
{

static const char SETBUZZER[] = "robotnik_msgs/SetBuzzer";

  class SetBuzzerRequest : public ros::Msg
  {
    public:
      typedef bool _enable_type;
      _enable_type enable;
      typedef float _beep_freq_type;
      _beep_freq_type beep_freq;
      typedef float _time_enabled_type;
      _time_enabled_type time_enabled;

    SetBuzzerRequest():
      enable(0),
      beep_freq(0),
      time_enabled(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_enable;
      u_enable.real = this->enable;
      *(outbuffer + offset + 0) = (u_enable.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->enable);
      offset += serializeAvrFloat64(outbuffer + offset, this->beep_freq);
      offset += serializeAvrFloat64(outbuffer + offset, this->time_enabled);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_enable;
      u_enable.base = 0;
      u_enable.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->enable = u_enable.real;
      offset += sizeof(this->enable);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->beep_freq));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->time_enabled));
     return offset;
    }

    virtual const char * getType() override { return SETBUZZER; };
    virtual const char * getMD5() override { return "7cb4f8ecf7ad9c3ff741117e0298b359"; };

  };

  class SetBuzzerResponse : public ros::Msg
  {
    public:
      typedef const char* _msg_type;
      _msg_type msg;
      typedef bool _ret_type;
      _ret_type ret;

    SetBuzzerResponse():
      msg(""),
      ret(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_msg = strlen(this->msg);
      varToArr(outbuffer + offset, length_msg);
      offset += 4;
      memcpy(outbuffer + offset, this->msg, length_msg);
      offset += length_msg;
      union {
        bool real;
        uint8_t base;
      } u_ret;
      u_ret.real = this->ret;
      *(outbuffer + offset + 0) = (u_ret.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ret);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_msg;
      arrToVar(length_msg, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_msg; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_msg-1]=0;
      this->msg = (char *)(inbuffer + offset-1);
      offset += length_msg;
      union {
        bool real;
        uint8_t base;
      } u_ret;
      u_ret.base = 0;
      u_ret.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ret = u_ret.real;
      offset += sizeof(this->ret);
     return offset;
    }

    virtual const char * getType() override { return SETBUZZER; };
    virtual const char * getMD5() override { return "61cb52ead9791e6a2cd0753b71d52c54"; };

  };

  class SetBuzzer {
    public:
    typedef SetBuzzerRequest Request;
    typedef SetBuzzerResponse Response;
  };

}
#endif
