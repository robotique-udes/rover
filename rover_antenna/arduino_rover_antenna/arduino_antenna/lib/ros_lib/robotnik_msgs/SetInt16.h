#ifndef _ROS_SERVICE_SetInt16_h
#define _ROS_SERVICE_SetInt16_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "robotnik_msgs/ReturnMessage.h"
#include "std_msgs/Int16.h"

namespace robotnik_msgs
{

static const char SETINT16[] = "robotnik_msgs/SetInt16";

  class SetInt16Request : public ros::Msg
  {
    public:
      typedef std_msgs::Int16 _data_type;
      _data_type data;

    SetInt16Request():
      data()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->data.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->data.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return SETINT16; };
    virtual const char * getMD5() override { return "0f9585f47779b9607f760bef85a254c0"; };

  };

  class SetInt16Response : public ros::Msg
  {
    public:
      typedef robotnik_msgs::ReturnMessage _ret_type;
      _ret_type ret;

    SetInt16Response():
      ret()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->ret.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->ret.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return SETINT16; };
    virtual const char * getMD5() override { return "1cc59476b1732f75af5b4512acb5adbe"; };

  };

  class SetInt16 {
    public:
    typedef SetInt16Request Request;
    typedef SetInt16Response Response;
  };

}
#endif
