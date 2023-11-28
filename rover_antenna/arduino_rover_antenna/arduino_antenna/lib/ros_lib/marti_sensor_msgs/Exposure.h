#ifndef _ROS_marti_sensor_msgs_Exposure_h
#define _ROS_marti_sensor_msgs_Exposure_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace marti_sensor_msgs
{

  class Exposure : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint64_t _value_type;
      _value_type value;

    Exposure():
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
      *(outbuffer + offset + 4) = (this->value >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (this->value >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (this->value >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (this->value >> (8 * 7)) & 0xFF;
      offset += sizeof(this->value);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->value =  ((uint64_t) (*(inbuffer + offset)));
      this->value |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->value |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->value |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->value |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      this->value |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      this->value |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      this->value |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      offset += sizeof(this->value);
     return offset;
    }

    virtual const char * getType() override { return "marti_sensor_msgs/Exposure"; };
    virtual const char * getMD5() override { return "85b556f2af6a79c3e57c029d50b2ad45"; };

  };

}
#endif
