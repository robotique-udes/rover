#ifndef _ROS_robotnik_msgs_BatteryDockingStatusStamped_h
#define _ROS_robotnik_msgs_BatteryDockingStatusStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "robotnik_msgs/BatteryDockingStatus.h"

namespace robotnik_msgs
{

  class BatteryDockingStatusStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef robotnik_msgs::BatteryDockingStatus _status_type;
      _status_type status;

    BatteryDockingStatusStamped():
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

    virtual const char * getType() override { return "robotnik_msgs/BatteryDockingStatusStamped"; };
    virtual const char * getMD5() override { return "f9b376e82e9d778484349573af188b1d"; };

  };

}
#endif
