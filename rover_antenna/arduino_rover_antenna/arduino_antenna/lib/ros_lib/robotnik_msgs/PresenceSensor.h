#ifndef _ROS_robotnik_msgs_PresenceSensor_h
#define _ROS_robotnik_msgs_PresenceSensor_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"

namespace robotnik_msgs
{

  class PresenceSensor : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef geometry_msgs::Pose _pose_type;
      _pose_type pose;
      typedef const char* _sensor_id_type;
      _sensor_id_type sensor_id;
      typedef const char* _sensor_type_type;
      _sensor_type_type sensor_type;
      typedef const char* _detected_id_type;
      _detected_id_type detected_id;
      typedef const char* _zone_type;
      _zone_type zone;
      typedef bool _enabled_type;
      _enabled_type enabled;
      typedef bool _value_type;
      _value_type value;
      enum { CAMERA = camera };
      enum { PHOTOCELL = photocell };

    PresenceSensor():
      header(),
      pose(),
      sensor_id(""),
      sensor_type(""),
      detected_id(""),
      zone(""),
      enabled(0),
      value(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->pose.serialize(outbuffer + offset);
      uint32_t length_sensor_id = strlen(this->sensor_id);
      varToArr(outbuffer + offset, length_sensor_id);
      offset += 4;
      memcpy(outbuffer + offset, this->sensor_id, length_sensor_id);
      offset += length_sensor_id;
      uint32_t length_sensor_type = strlen(this->sensor_type);
      varToArr(outbuffer + offset, length_sensor_type);
      offset += 4;
      memcpy(outbuffer + offset, this->sensor_type, length_sensor_type);
      offset += length_sensor_type;
      uint32_t length_detected_id = strlen(this->detected_id);
      varToArr(outbuffer + offset, length_detected_id);
      offset += 4;
      memcpy(outbuffer + offset, this->detected_id, length_detected_id);
      offset += length_detected_id;
      uint32_t length_zone = strlen(this->zone);
      varToArr(outbuffer + offset, length_zone);
      offset += 4;
      memcpy(outbuffer + offset, this->zone, length_zone);
      offset += length_zone;
      union {
        bool real;
        uint8_t base;
      } u_enabled;
      u_enabled.real = this->enabled;
      *(outbuffer + offset + 0) = (u_enabled.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->enabled);
      union {
        bool real;
        uint8_t base;
      } u_value;
      u_value.real = this->value;
      *(outbuffer + offset + 0) = (u_value.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->value);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->pose.deserialize(inbuffer + offset);
      uint32_t length_sensor_id;
      arrToVar(length_sensor_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_sensor_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_sensor_id-1]=0;
      this->sensor_id = (char *)(inbuffer + offset-1);
      offset += length_sensor_id;
      uint32_t length_sensor_type;
      arrToVar(length_sensor_type, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_sensor_type; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_sensor_type-1]=0;
      this->sensor_type = (char *)(inbuffer + offset-1);
      offset += length_sensor_type;
      uint32_t length_detected_id;
      arrToVar(length_detected_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_detected_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_detected_id-1]=0;
      this->detected_id = (char *)(inbuffer + offset-1);
      offset += length_detected_id;
      uint32_t length_zone;
      arrToVar(length_zone, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_zone; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_zone-1]=0;
      this->zone = (char *)(inbuffer + offset-1);
      offset += length_zone;
      union {
        bool real;
        uint8_t base;
      } u_enabled;
      u_enabled.base = 0;
      u_enabled.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->enabled = u_enabled.real;
      offset += sizeof(this->enabled);
      union {
        bool real;
        uint8_t base;
      } u_value;
      u_value.base = 0;
      u_value.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->value = u_value.real;
      offset += sizeof(this->value);
     return offset;
    }

    virtual const char * getType() override { return "robotnik_msgs/PresenceSensor"; };
    virtual const char * getMD5() override { return "d655196d2d8c595af378ef8e462eca4a"; };

  };

}
#endif
