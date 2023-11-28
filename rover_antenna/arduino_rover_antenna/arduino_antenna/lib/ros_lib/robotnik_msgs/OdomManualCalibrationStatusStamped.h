#ifndef _ROS_robotnik_msgs_OdomManualCalibrationStatusStamped_h
#define _ROS_robotnik_msgs_OdomManualCalibrationStatusStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "robotnik_msgs/OdomManualCalibrationStatus.h"

namespace robotnik_msgs
{

  class OdomManualCalibrationStatusStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef robotnik_msgs::OdomManualCalibrationStatus _status_type;
      _status_type status;

    OdomManualCalibrationStatusStamped():
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

    virtual const char * getType() override { return "robotnik_msgs/OdomManualCalibrationStatusStamped"; };
    virtual const char * getMD5() override { return "040580a8054e03204d303892affeca19"; };

  };

}
#endif
