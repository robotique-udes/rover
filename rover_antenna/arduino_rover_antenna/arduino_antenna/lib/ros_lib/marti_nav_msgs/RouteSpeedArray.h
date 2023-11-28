#ifndef _ROS_marti_nav_msgs_RouteSpeedArray_h
#define _ROS_marti_nav_msgs_RouteSpeedArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "marti_nav_msgs/RouteSpeed.h"

namespace marti_nav_msgs
{

  class RouteSpeedArray : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t speeds_length;
      typedef marti_nav_msgs::RouteSpeed _speeds_type;
      _speeds_type st_speeds;
      _speeds_type * speeds;

    RouteSpeedArray():
      header(),
      speeds_length(0), st_speeds(), speeds(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->speeds_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->speeds_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->speeds_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->speeds_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->speeds_length);
      for( uint32_t i = 0; i < speeds_length; i++){
      offset += this->speeds[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t speeds_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      speeds_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      speeds_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      speeds_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->speeds_length);
      if(speeds_lengthT > speeds_length)
        this->speeds = (marti_nav_msgs::RouteSpeed*)realloc(this->speeds, speeds_lengthT * sizeof(marti_nav_msgs::RouteSpeed));
      speeds_length = speeds_lengthT;
      for( uint32_t i = 0; i < speeds_length; i++){
      offset += this->st_speeds.deserialize(inbuffer + offset);
        memcpy( &(this->speeds[i]), &(this->st_speeds), sizeof(marti_nav_msgs::RouteSpeed));
      }
     return offset;
    }

    virtual const char * getType() override { return "marti_nav_msgs/RouteSpeedArray"; };
    virtual const char * getMD5() override { return "c5b2e8db78eaab7eafdb3ecf8d4e017f"; };

  };

}
#endif
