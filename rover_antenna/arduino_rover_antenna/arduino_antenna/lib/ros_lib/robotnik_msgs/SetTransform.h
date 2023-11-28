#ifndef _ROS_SERVICE_SetTransform_h
#define _ROS_SERVICE_SetTransform_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Transform.h"
#include "robotnik_msgs/ReturnMessage.h"

namespace robotnik_msgs
{

static const char SETTRANSFORM[] = "robotnik_msgs/SetTransform";

  class SetTransformRequest : public ros::Msg
  {
    public:
      typedef geometry_msgs::Transform _tf_type;
      _tf_type tf;

    SetTransformRequest():
      tf()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->tf.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->tf.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return SETTRANSFORM; };
    virtual const char * getMD5() override { return "470d1b6b1231a033d37ede5826ea44d4"; };

  };

  class SetTransformResponse : public ros::Msg
  {
    public:
      typedef robotnik_msgs::ReturnMessage _ret_type;
      _ret_type ret;

    SetTransformResponse():
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

    virtual const char * getType() override { return SETTRANSFORM; };
    virtual const char * getMD5() override { return "1cc59476b1732f75af5b4512acb5adbe"; };

  };

  class SetTransform {
    public:
    typedef SetTransformRequest Request;
    typedef SetTransformResponse Response;
  };

}
#endif
