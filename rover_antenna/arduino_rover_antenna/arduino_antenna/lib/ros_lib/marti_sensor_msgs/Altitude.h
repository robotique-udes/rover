#ifndef _ROS_marti_sensor_msgs_Altitude_h
#define _ROS_marti_sensor_msgs_Altitude_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace marti_sensor_msgs
{

  class Altitude : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _altitude_type;
      _altitude_type altitude;
      typedef float _sigma_type;
      _sigma_type sigma;

    Altitude():
      header(),
      altitude(0),
      sigma(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->altitude);
      offset += serializeAvrFloat64(outbuffer + offset, this->sigma);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->altitude));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->sigma));
     return offset;
    }

    virtual const char * getType() override { return "marti_sensor_msgs/Altitude"; };
    virtual const char * getMD5() override { return "c3156c79592b8b17259917f4804f4420"; };

  };

}
#endif
