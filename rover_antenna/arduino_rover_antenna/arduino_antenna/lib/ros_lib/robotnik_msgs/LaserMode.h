#ifndef _ROS_robotnik_msgs_LaserMode_h
#define _ROS_robotnik_msgs_LaserMode_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robotnik_msgs
{

  class LaserMode : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      enum { STANDARD = standard };
      enum { DOCKING_STATION = docking_station };
      enum { CART = cart };
      enum { CART_DOCKING_CART = cart_docking_cart };
      enum { DOCKING_CART = docking_cart };
      enum { REDUCED = reduced };
      enum { INVALID = invalid };

    LaserMode():
      name("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
     return offset;
    }

    virtual const char * getType() override { return "robotnik_msgs/LaserMode"; };
    virtual const char * getMD5() override { return "04d69d26faf3f5fdd3d172ee0e35f8ea"; };

  };

}
#endif
