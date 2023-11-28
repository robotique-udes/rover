#ifndef _ROS_robotnik_msgs_RobotnikMotorsStatus_h
#define _ROS_robotnik_msgs_RobotnikMotorsStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "robotnik_msgs/MotorStatus.h"

namespace robotnik_msgs
{

  class RobotnikMotorsStatus : public ros::Msg
  {
    public:
      typedef const char* _state_type;
      _state_type state;
      uint32_t motor_status_length;
      typedef robotnik_msgs::MotorStatus _motor_status_type;
      _motor_status_type st_motor_status;
      _motor_status_type * motor_status;

    RobotnikMotorsStatus():
      state(""),
      motor_status_length(0), st_motor_status(), motor_status(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_state = strlen(this->state);
      varToArr(outbuffer + offset, length_state);
      offset += 4;
      memcpy(outbuffer + offset, this->state, length_state);
      offset += length_state;
      *(outbuffer + offset + 0) = (this->motor_status_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->motor_status_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->motor_status_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->motor_status_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->motor_status_length);
      for( uint32_t i = 0; i < motor_status_length; i++){
      offset += this->motor_status[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_state;
      arrToVar(length_state, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_state; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_state-1]=0;
      this->state = (char *)(inbuffer + offset-1);
      offset += length_state;
      uint32_t motor_status_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      motor_status_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      motor_status_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      motor_status_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->motor_status_length);
      if(motor_status_lengthT > motor_status_length)
        this->motor_status = (robotnik_msgs::MotorStatus*)realloc(this->motor_status, motor_status_lengthT * sizeof(robotnik_msgs::MotorStatus));
      motor_status_length = motor_status_lengthT;
      for( uint32_t i = 0; i < motor_status_length; i++){
      offset += this->st_motor_status.deserialize(inbuffer + offset);
        memcpy( &(this->motor_status[i]), &(this->st_motor_status), sizeof(robotnik_msgs::MotorStatus));
      }
     return offset;
    }

    virtual const char * getType() override { return "robotnik_msgs/RobotnikMotorsStatus"; };
    virtual const char * getMD5() override { return "a6f76b778bb6c93074bf081a98ca69ac"; };

  };

}
#endif
