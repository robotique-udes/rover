#ifndef _ROS_SERVICE_GetRelayStatus_h
#define _ROS_SERVICE_GetRelayStatus_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rly_08
{

static const char GETRELAYSTATUS[] = "rly_08/GetRelayStatus";

  class GetRelayStatusRequest : public ros::Msg
  {
    public:
      typedef uint32_t _iRelay_type;
      _iRelay_type iRelay;

    GetRelayStatusRequest():
      iRelay(0)
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
     return offset;
    }

    virtual const char * getType() override { return GETRELAYSTATUS; };
    virtual const char * getMD5() override { return "11ca886dab5904e49c4cce124c21dfaa"; };

  };

  class GetRelayStatusResponse : public ros::Msg
  {
    public:
      typedef int32_t _status_type;
      _status_type status;

    GetRelayStatusResponse():
      status(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_status;
      u_status.real = this->status;
      *(outbuffer + offset + 0) = (u_status.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_status.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_status.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_status.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->status);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_status;
      u_status.base = 0;
      u_status.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_status.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_status.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_status.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->status = u_status.real;
      offset += sizeof(this->status);
     return offset;
    }

    virtual const char * getType() override { return GETRELAYSTATUS; };
    virtual const char * getMD5() override { return "86791dcf1de997ec7de5a0de7e4dcfcc"; };

  };

  class GetRelayStatus {
    public:
    typedef GetRelayStatusRequest Request;
    typedef GetRelayStatusResponse Response;
  };

}
#endif
