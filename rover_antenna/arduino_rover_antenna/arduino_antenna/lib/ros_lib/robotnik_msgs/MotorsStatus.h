#ifndef _ROS_robotnik_msgs_MotorsStatus_h
#define _ROS_robotnik_msgs_MotorsStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "robotnik_msgs/MotorStatus.h"

namespace robotnik_msgs
{

  class MotorsStatus : public ros::Msg
  {
    public:
      typedef robotnik_msgs::MotorStatus _flwStatus_type;
      _flwStatus_type flwStatus;
      typedef robotnik_msgs::MotorStatus _blwStatus_type;
      _blwStatus_type blwStatus;
      typedef robotnik_msgs::MotorStatus _frwStatus_type;
      _frwStatus_type frwStatus;
      typedef robotnik_msgs::MotorStatus _brwStatus_type;
      _brwStatus_type brwStatus;

    MotorsStatus():
      flwStatus(),
      blwStatus(),
      frwStatus(),
      brwStatus()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->flwStatus.serialize(outbuffer + offset);
      offset += this->blwStatus.serialize(outbuffer + offset);
      offset += this->frwStatus.serialize(outbuffer + offset);
      offset += this->brwStatus.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->flwStatus.deserialize(inbuffer + offset);
      offset += this->blwStatus.deserialize(inbuffer + offset);
      offset += this->frwStatus.deserialize(inbuffer + offset);
      offset += this->brwStatus.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "robotnik_msgs/MotorsStatus"; };
    virtual const char * getMD5() override { return "8737e2a027c7f183f76ac545960670cc"; };

  };

}
#endif
