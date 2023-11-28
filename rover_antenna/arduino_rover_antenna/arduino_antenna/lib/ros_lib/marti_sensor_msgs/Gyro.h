#ifndef _ROS_marti_sensor_msgs_Gyro_h
#define _ROS_marti_sensor_msgs_Gyro_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace marti_sensor_msgs
{

  class Gyro : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _angular_rate_type;
      _angular_rate_type angular_rate;
      typedef float _variance_type;
      _variance_type variance;

    Gyro():
      header(),
      angular_rate(0),
      variance(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->angular_rate);
      offset += serializeAvrFloat64(outbuffer + offset, this->variance);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->angular_rate));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->variance));
     return offset;
    }

    virtual const char * getType() override { return "marti_sensor_msgs/Gyro"; };
    virtual const char * getMD5() override { return "f2ad455b27f86b992b2cad5af4af11ba"; };

  };

}
#endif
