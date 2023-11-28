#ifndef _ROS_SERVICE_GetSwVersion_h
#define _ROS_SERVICE_GetSwVersion_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rly_08
{

static const char GETSWVERSION[] = "rly_08/GetSwVersion";

  class GetSwVersionRequest : public ros::Msg
  {
    public:

    GetSwVersionRequest()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
     return offset;
    }

    virtual const char * getType() override { return GETSWVERSION; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetSwVersionResponse : public ros::Msg
  {
    public:
      typedef int32_t _module_id_type;
      _module_id_type module_id;
      typedef int32_t _sw_version_type;
      _sw_version_type sw_version;

    GetSwVersionResponse():
      module_id(0),
      sw_version(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_module_id;
      u_module_id.real = this->module_id;
      *(outbuffer + offset + 0) = (u_module_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_module_id.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_module_id.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_module_id.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->module_id);
      union {
        int32_t real;
        uint32_t base;
      } u_sw_version;
      u_sw_version.real = this->sw_version;
      *(outbuffer + offset + 0) = (u_sw_version.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sw_version.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sw_version.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sw_version.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sw_version);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_module_id;
      u_module_id.base = 0;
      u_module_id.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_module_id.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_module_id.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_module_id.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->module_id = u_module_id.real;
      offset += sizeof(this->module_id);
      union {
        int32_t real;
        uint32_t base;
      } u_sw_version;
      u_sw_version.base = 0;
      u_sw_version.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_sw_version.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_sw_version.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_sw_version.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->sw_version = u_sw_version.real;
      offset += sizeof(this->sw_version);
     return offset;
    }

    virtual const char * getType() override { return GETSWVERSION; };
    virtual const char * getMD5() override { return "d913ea691de29aee2b42df700f5f4b04"; };

  };

  class GetSwVersion {
    public:
    typedef GetSwVersionRequest Request;
    typedef GetSwVersionResponse Response;
  };

}
#endif
