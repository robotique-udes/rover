#ifndef _ROS_SERVICE_SetString_h
#define _ROS_SERVICE_SetString_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "robotnik_msgs/ReturnMessage.h"

namespace robotnik_msgs
{

static const char SETSTRING[] = "robotnik_msgs/SetString";

  class SetStringRequest : public ros::Msg
  {
    public:
      typedef const char* _data_type;
      _data_type data;

    SetStringRequest():
      data("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_data = strlen(this->data);
      varToArr(outbuffer + offset, length_data);
      offset += 4;
      memcpy(outbuffer + offset, this->data, length_data);
      offset += length_data;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_data;
      arrToVar(length_data, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_data; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_data-1]=0;
      this->data = (char *)(inbuffer + offset-1);
      offset += length_data;
     return offset;
    }

    virtual const char * getType() override { return SETSTRING; };
    virtual const char * getMD5() override { return "992ce8a1687cec8c8bd883ec73ca41d1"; };

  };

  class SetStringResponse : public ros::Msg
  {
    public:
      typedef robotnik_msgs::ReturnMessage _ret_type;
      _ret_type ret;

    SetStringResponse():
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

    virtual const char * getType() override { return SETSTRING; };
    virtual const char * getMD5() override { return "1cc59476b1732f75af5b4512acb5adbe"; };

  };

  class SetString {
    public:
    typedef SetStringRequest Request;
    typedef SetStringResponse Response;
  };

}
#endif
