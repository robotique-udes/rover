#ifndef _ROS_robotnik_msgs_MotorsStatusDifferential_h
#define _ROS_robotnik_msgs_MotorsStatusDifferential_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "robotnik_msgs/MotorStatus.h"

namespace robotnik_msgs
{

  class MotorsStatusDifferential : public ros::Msg
  {
    public:
      typedef robotnik_msgs::MotorStatus _lwStatus_type;
      _lwStatus_type lwStatus;
      typedef robotnik_msgs::MotorStatus _rwStatus_type;
      _rwStatus_type rwStatus;

    MotorsStatusDifferential():
      lwStatus(),
      rwStatus()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->lwStatus.serialize(outbuffer + offset);
      offset += this->rwStatus.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->lwStatus.deserialize(inbuffer + offset);
      offset += this->rwStatus.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "robotnik_msgs/MotorsStatusDifferential"; };
    virtual const char * getMD5() override { return "54c939ea4e1227a032aeca24b67584ad"; };

  };

}
#endif
