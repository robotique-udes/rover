#ifndef _ROS_robotnik_msgs_PresenceSensorArray_h
#define _ROS_robotnik_msgs_PresenceSensorArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "robotnik_msgs/PresenceSensor.h"

namespace robotnik_msgs
{

  class PresenceSensorArray : public ros::Msg
  {
    public:
      uint32_t sensors_length;
      typedef robotnik_msgs::PresenceSensor _sensors_type;
      _sensors_type st_sensors;
      _sensors_type * sensors;

    PresenceSensorArray():
      sensors_length(0), st_sensors(), sensors(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->sensors_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sensors_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sensors_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sensors_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sensors_length);
      for( uint32_t i = 0; i < sensors_length; i++){
      offset += this->sensors[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t sensors_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      sensors_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      sensors_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      sensors_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->sensors_length);
      if(sensors_lengthT > sensors_length)
        this->sensors = (robotnik_msgs::PresenceSensor*)realloc(this->sensors, sensors_lengthT * sizeof(robotnik_msgs::PresenceSensor));
      sensors_length = sensors_lengthT;
      for( uint32_t i = 0; i < sensors_length; i++){
      offset += this->st_sensors.deserialize(inbuffer + offset);
        memcpy( &(this->sensors[i]), &(this->st_sensors), sizeof(robotnik_msgs::PresenceSensor));
      }
     return offset;
    }

    virtual const char * getType() override { return "robotnik_msgs/PresenceSensorArray"; };
    virtual const char * getMD5() override { return "eb0dd527b36bdf6dd7986227d34224a6"; };

  };

}
#endif
