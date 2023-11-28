#ifndef _ROS_marti_sensor_msgs_WheelEncoderSet_h
#define _ROS_marti_sensor_msgs_WheelEncoderSet_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "marti_sensor_msgs/WheelEncoder.h"

namespace marti_sensor_msgs
{

  class WheelEncoderSet : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t encoders_length;
      typedef marti_sensor_msgs::WheelEncoder _encoders_type;
      _encoders_type st_encoders;
      _encoders_type * encoders;

    WheelEncoderSet():
      header(),
      encoders_length(0), st_encoders(), encoders(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->encoders_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->encoders_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->encoders_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->encoders_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->encoders_length);
      for( uint32_t i = 0; i < encoders_length; i++){
      offset += this->encoders[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t encoders_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      encoders_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      encoders_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      encoders_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->encoders_length);
      if(encoders_lengthT > encoders_length)
        this->encoders = (marti_sensor_msgs::WheelEncoder*)realloc(this->encoders, encoders_lengthT * sizeof(marti_sensor_msgs::WheelEncoder));
      encoders_length = encoders_lengthT;
      for( uint32_t i = 0; i < encoders_length; i++){
      offset += this->st_encoders.deserialize(inbuffer + offset);
        memcpy( &(this->encoders[i]), &(this->st_encoders), sizeof(marti_sensor_msgs::WheelEncoder));
      }
     return offset;
    }

    virtual const char * getType() override { return "marti_sensor_msgs/WheelEncoderSet"; };
    virtual const char * getMD5() override { return "a1169b74ddf14d2e1ad1aa65311182d9"; };

  };

}
#endif
