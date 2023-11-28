#ifndef _ROS_marti_nav_msgs_Command_h
#define _ROS_marti_nav_msgs_Command_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace marti_nav_msgs
{

  class Command : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t startstop_length;
      typedef int32_t _startstop_type;
      _startstop_type st_startstop;
      _startstop_type * startstop;

    Command():
      header(),
      startstop_length(0), st_startstop(), startstop(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->startstop_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->startstop_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->startstop_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->startstop_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->startstop_length);
      for( uint32_t i = 0; i < startstop_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_startstopi;
      u_startstopi.real = this->startstop[i];
      *(outbuffer + offset + 0) = (u_startstopi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_startstopi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_startstopi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_startstopi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->startstop[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t startstop_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      startstop_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      startstop_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      startstop_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->startstop_length);
      if(startstop_lengthT > startstop_length)
        this->startstop = (int32_t*)realloc(this->startstop, startstop_lengthT * sizeof(int32_t));
      startstop_length = startstop_lengthT;
      for( uint32_t i = 0; i < startstop_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_startstop;
      u_st_startstop.base = 0;
      u_st_startstop.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_startstop.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_startstop.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_startstop.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_startstop = u_st_startstop.real;
      offset += sizeof(this->st_startstop);
        memcpy( &(this->startstop[i]), &(this->st_startstop), sizeof(int32_t));
      }
     return offset;
    }

    virtual const char * getType() override { return "marti_nav_msgs/Command"; };
    virtual const char * getMD5() override { return "252763f31a786611d807254ade7c1dc3"; };

  };

}
#endif
