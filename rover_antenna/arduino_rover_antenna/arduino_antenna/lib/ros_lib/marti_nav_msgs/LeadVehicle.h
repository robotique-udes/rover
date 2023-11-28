#ifndef _ROS_marti_nav_msgs_LeadVehicle_h
#define _ROS_marti_nav_msgs_LeadVehicle_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace marti_nav_msgs
{

  class LeadVehicle : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _headwayDistance_type;
      _headwayDistance_type headwayDistance;
      typedef float _speed_type;
      _speed_type speed;
      typedef float _heading_type;
      _heading_type heading;
      typedef float _xPos_type;
      _xPos_type xPos;
      typedef float _yPos_type;
      _yPos_type yPos;
      typedef int8_t _classification_type;
      _classification_type classification;
      typedef int8_t _type_type;
      _type_type type;

    LeadVehicle():
      header(),
      headwayDistance(0),
      speed(0),
      heading(0),
      xPos(0),
      yPos(0),
      classification(0),
      type(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_headwayDistance;
      u_headwayDistance.real = this->headwayDistance;
      *(outbuffer + offset + 0) = (u_headwayDistance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_headwayDistance.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_headwayDistance.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_headwayDistance.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->headwayDistance);
      union {
        float real;
        uint32_t base;
      } u_speed;
      u_speed.real = this->speed;
      *(outbuffer + offset + 0) = (u_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->speed);
      union {
        float real;
        uint32_t base;
      } u_heading;
      u_heading.real = this->heading;
      *(outbuffer + offset + 0) = (u_heading.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_heading.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_heading.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_heading.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->heading);
      union {
        float real;
        uint32_t base;
      } u_xPos;
      u_xPos.real = this->xPos;
      *(outbuffer + offset + 0) = (u_xPos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_xPos.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_xPos.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_xPos.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->xPos);
      union {
        float real;
        uint32_t base;
      } u_yPos;
      u_yPos.real = this->yPos;
      *(outbuffer + offset + 0) = (u_yPos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_yPos.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_yPos.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_yPos.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->yPos);
      union {
        int8_t real;
        uint8_t base;
      } u_classification;
      u_classification.real = this->classification;
      *(outbuffer + offset + 0) = (u_classification.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->classification);
      union {
        int8_t real;
        uint8_t base;
      } u_type;
      u_type.real = this->type;
      *(outbuffer + offset + 0) = (u_type.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->type);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_headwayDistance;
      u_headwayDistance.base = 0;
      u_headwayDistance.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_headwayDistance.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_headwayDistance.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_headwayDistance.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->headwayDistance = u_headwayDistance.real;
      offset += sizeof(this->headwayDistance);
      union {
        float real;
        uint32_t base;
      } u_speed;
      u_speed.base = 0;
      u_speed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->speed = u_speed.real;
      offset += sizeof(this->speed);
      union {
        float real;
        uint32_t base;
      } u_heading;
      u_heading.base = 0;
      u_heading.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_heading.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_heading.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_heading.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->heading = u_heading.real;
      offset += sizeof(this->heading);
      union {
        float real;
        uint32_t base;
      } u_xPos;
      u_xPos.base = 0;
      u_xPos.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_xPos.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_xPos.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_xPos.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->xPos = u_xPos.real;
      offset += sizeof(this->xPos);
      union {
        float real;
        uint32_t base;
      } u_yPos;
      u_yPos.base = 0;
      u_yPos.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_yPos.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_yPos.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_yPos.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->yPos = u_yPos.real;
      offset += sizeof(this->yPos);
      union {
        int8_t real;
        uint8_t base;
      } u_classification;
      u_classification.base = 0;
      u_classification.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->classification = u_classification.real;
      offset += sizeof(this->classification);
      union {
        int8_t real;
        uint8_t base;
      } u_type;
      u_type.base = 0;
      u_type.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->type = u_type.real;
      offset += sizeof(this->type);
     return offset;
    }

    virtual const char * getType() override { return "marti_nav_msgs/LeadVehicle"; };
    virtual const char * getMD5() override { return "123a04e085bfddd727efd0e68809c765"; };

  };

}
#endif
