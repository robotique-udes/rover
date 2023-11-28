#ifndef _ROS_SERVICE_SetRelayStatus_h
#define _ROS_SERVICE_SetRelayStatus_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rly_08
{

static const char SETRELAYSTATUS[] = "rly_08/SetRelayStatus";

  class SetRelayStatusRequest : public ros::Msg
  {
    public:
      typedef uint32_t _iRelay_type;
      _iRelay_type iRelay;
      typedef uint32_t _value_type;
      _value_type value;

    SetRelayStatusRequest():
      iRelay(0),
      value(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->iRelay >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->iRelay >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->iRelay >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->iRelay >> (8 * 3)) & 0xFF;
      offset += sizeof(this->iRelay);
      *(outbuffer + offset + 0) = (this->value >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->value >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->value >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->value >> (8 * 3)) & 0xFF;
      offset += sizeof(this->value);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->iRelay =  ((uint32_t) (*(inbuffer + offset)));
      this->iRelay |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->iRelay |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->iRelay |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->iRelay);
      this->value =  ((uint32_t) (*(inbuffer + offset)));
      this->value |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->value |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->value |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->value);
     return offset;
    }

    virtual const char * getType() override { return SETRELAYSTATUS; };
    virtual const char * getMD5() override { return "03222b9aafbf74903616ff5938f3efd0"; };

  };

  class SetRelayStatusResponse : public ros::Msg
  {
    public:
      typedef bool _result_type;
      _result_type result;

    SetRelayStatusResponse():
      result(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_result;
      u_result.real = this->result;
      *(outbuffer + offset + 0) = (u_result.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->result);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_result;
      u_result.base = 0;
      u_result.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->result = u_result.real;
      offset += sizeof(this->result);
     return offset;
    }

    virtual const char * getType() override { return SETRELAYSTATUS; };
    virtual const char * getMD5() override { return "eb13ac1f1354ccecb7941ee8fa2192e8"; };

  };

  class SetRelayStatus {
    public:
    typedef SetRelayStatusRequest Request;
    typedef SetRelayStatusResponse Response;
  };

}
#endif
