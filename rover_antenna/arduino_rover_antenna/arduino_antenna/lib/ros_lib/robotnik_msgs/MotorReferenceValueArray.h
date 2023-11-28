#ifndef _ROS_robotnik_msgs_MotorReferenceValueArray_h
#define _ROS_robotnik_msgs_MotorReferenceValueArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "robotnik_msgs/MotorReferenceValue.h"

namespace robotnik_msgs
{

  class MotorReferenceValueArray : public ros::Msg
  {
    public:
      typedef robotnik_msgs::MotorReferenceValue _velocity_type;
      _velocity_type velocity;
      typedef robotnik_msgs::MotorReferenceValue _position_type;
      _position_type position;
      typedef robotnik_msgs::MotorReferenceValue _torque_type;
      _torque_type torque;

    MotorReferenceValueArray():
      velocity(),
      position(),
      torque()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->velocity.serialize(outbuffer + offset);
      offset += this->position.serialize(outbuffer + offset);
      offset += this->torque.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->velocity.deserialize(inbuffer + offset);
      offset += this->position.deserialize(inbuffer + offset);
      offset += this->torque.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "robotnik_msgs/MotorReferenceValueArray"; };
    virtual const char * getMD5() override { return "6729745f687db56799272568ebe51000"; };

  };

}
#endif
