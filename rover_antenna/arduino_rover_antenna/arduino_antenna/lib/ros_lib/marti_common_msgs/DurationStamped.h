#ifndef _ROS_marti_common_msgs_DurationStamped_h
#define _ROS_marti_common_msgs_DurationStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "ros/duration.h"

namespace marti_common_msgs
{

  class DurationStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef ros::Duration _value_type;
      _value_type value;

    DurationStamped():
      header(),
      value()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->value.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->value.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->value.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->value.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->value.sec);
      *(outbuffer + offset + 0) = (this->value.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->value.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->value.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->value.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->value.nsec);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->value.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->value.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->value.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->value.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->value.sec);
      this->value.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->value.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->value.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->value.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->value.nsec);
     return offset;
    }

    virtual const char * getType() override { return "marti_common_msgs/DurationStamped"; };
    virtual const char * getMD5() override { return "abd101ec65114034e9bd0e187b790140"; };

  };

}
#endif
