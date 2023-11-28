#ifndef _ROS_marti_common_msgs_Float64Stamped_h
#define _ROS_marti_common_msgs_Float64Stamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace marti_common_msgs
{

  class Float64Stamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _value_type;
      _value_type value;

    Float64Stamped():
      header(),
      value(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->value);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->value));
     return offset;
    }

    virtual const char * getType() override { return "marti_common_msgs/Float64Stamped"; };
    virtual const char * getMD5() override { return "d053817de0764f9ee90dbc89c4cdd751"; };

  };

}
#endif
