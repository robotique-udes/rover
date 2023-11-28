#ifndef _ROS_marti_nav_msgs_Wgs84Sample_h
#define _ROS_marti_nav_msgs_Wgs84Sample_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Point.h"

namespace marti_nav_msgs
{

  class Wgs84Sample : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef geometry_msgs::Point _odom_type;
      _odom_type odom;
      typedef geometry_msgs::Point _wgs84_type;
      _wgs84_type wgs84;
      float wgs84_covariance[9];

    Wgs84Sample():
      header(),
      odom(),
      wgs84(),
      wgs84_covariance()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->odom.serialize(outbuffer + offset);
      offset += this->wgs84.serialize(outbuffer + offset);
      for( uint32_t i = 0; i < 9; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->wgs84_covariance[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->odom.deserialize(inbuffer + offset);
      offset += this->wgs84.deserialize(inbuffer + offset);
      for( uint32_t i = 0; i < 9; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->wgs84_covariance[i]));
      }
     return offset;
    }

    virtual const char * getType() override { return "marti_nav_msgs/Wgs84Sample"; };
    virtual const char * getMD5() override { return "5241cb1ac52fc7a58f710c9a9774badc"; };

  };

}
#endif
