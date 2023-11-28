#ifndef _ROS_marti_common_msgs_UInt32Stamped_h
#define _ROS_marti_common_msgs_UInt32Stamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace marti_common_msgs
{

  class UInt32Stamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint32_t _value_type;
      _value_type value;

    UInt32Stamped():
      header(),
      value(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
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
      offset += this->header.deserialize(inbuffer + offset);
      this->value =  ((uint32_t) (*(inbuffer + offset)));
      this->value |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->value |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->value |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->value);
     return offset;
    }

    virtual const char * getType() override { return "marti_common_msgs/UInt32Stamped"; };
    virtual const char * getMD5() override { return "438dd455da526a293dd9eb06485f8a89"; };

  };

}
#endif
