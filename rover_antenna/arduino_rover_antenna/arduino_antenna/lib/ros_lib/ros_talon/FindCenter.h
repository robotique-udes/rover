#ifndef _ROS_SERVICE_FindCenter_h
#define _ROS_SERVICE_FindCenter_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ros_talon
{

static const char FINDCENTER[] = "ros_talon/FindCenter";

  class FindCenterRequest : public ros::Msg
  {
    public:

    FindCenterRequest()
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

    virtual const char * getType() override { return FINDCENTER; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class FindCenterResponse : public ros::Msg
  {
    public:

    FindCenterResponse()
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

    virtual const char * getType() override { return FINDCENTER; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class FindCenter {
    public:
    typedef FindCenterRequest Request;
    typedef FindCenterResponse Response;
  };

}
#endif
