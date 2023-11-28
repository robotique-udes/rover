#ifndef _ROS_robotnik_msgs_BatteryStatusStamped_h
#define _ROS_robotnik_msgs_BatteryStatusStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "robotnik_msgs/BatteryStatus.h"

namespace robotnik_msgs
{

  class BatteryStatusStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef robotnik_msgs::BatteryStatus _status_type;
      _status_type status;

    BatteryStatusStamped():
      header(),
      status()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->status.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->status.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "robotnik_msgs/BatteryStatusStamped"; };
    virtual const char * getMD5() override { return "ebfafe84a4dbed599e77635223eefcd0"; };

  };

}
#endif
