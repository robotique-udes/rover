#ifndef _ROS_marti_sensor_msgs_Velocity_h
#define _ROS_marti_sensor_msgs_Velocity_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace marti_sensor_msgs
{

  class Velocity : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _velocity_type;
      _velocity_type velocity;
      typedef float _variance_type;
      _variance_type variance;

    Velocity():
      header(),
      velocity(0),
      variance(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->velocity);
      offset += serializeAvrFloat64(outbuffer + offset, this->variance);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->velocity));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->variance));
     return offset;
    }

    virtual const char * getType() override { return "marti_sensor_msgs/Velocity"; };
    virtual const char * getMD5() override { return "d9f90fa9bf91df1f2554f047ca0b59f2"; };

  };

}
#endif
