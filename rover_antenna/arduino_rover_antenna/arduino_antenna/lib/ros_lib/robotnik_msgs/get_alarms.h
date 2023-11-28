#ifndef _ROS_SERVICE_get_alarms_h
#define _ROS_SERVICE_get_alarms_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "robotnik_msgs/alarmsmonitor.h"

namespace robotnik_msgs
{

static const char GET_ALARMS[] = "robotnik_msgs/get_alarms";

  class get_alarmsRequest : public ros::Msg
  {
    public:

    get_alarmsRequest()
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

    virtual const char * getType() override { return GET_ALARMS; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class get_alarmsResponse : public ros::Msg
  {
    public:
      typedef robotnik_msgs::alarmsmonitor _alarms_type;
      _alarms_type alarms;

    get_alarmsResponse():
      alarms()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->alarms.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->alarms.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return GET_ALARMS; };
    virtual const char * getMD5() override { return "4550354b6bec7529a93833858063eaed"; };

  };

  class get_alarms {
    public:
    typedef get_alarmsRequest Request;
    typedef get_alarmsResponse Response;
  };

}
#endif
