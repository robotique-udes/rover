#ifndef _ROS_marti_nav_msgs_Path_h
#define _ROS_marti_nav_msgs_Path_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "marti_nav_msgs/PathPoint.h"

namespace marti_nav_msgs
{

  class Path : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t points_length;
      typedef marti_nav_msgs::PathPoint _points_type;
      _points_type st_points;
      _points_type * points;
      typedef bool _in_reverse_type;
      _in_reverse_type in_reverse;

    Path():
      header(),
      points_length(0), st_points(), points(nullptr),
      in_reverse(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->points_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->points_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->points_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->points_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->points_length);
      for( uint32_t i = 0; i < points_length; i++){
      offset += this->points[i].serialize(outbuffer + offset);
      }
      union {
        bool real;
        uint8_t base;
      } u_in_reverse;
      u_in_reverse.real = this->in_reverse;
      *(outbuffer + offset + 0) = (u_in_reverse.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->in_reverse);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t points_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      points_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      points_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      points_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->points_length);
      if(points_lengthT > points_length)
        this->points = (marti_nav_msgs::PathPoint*)realloc(this->points, points_lengthT * sizeof(marti_nav_msgs::PathPoint));
      points_length = points_lengthT;
      for( uint32_t i = 0; i < points_length; i++){
      offset += this->st_points.deserialize(inbuffer + offset);
        memcpy( &(this->points[i]), &(this->st_points), sizeof(marti_nav_msgs::PathPoint));
      }
      union {
        bool real;
        uint8_t base;
      } u_in_reverse;
      u_in_reverse.base = 0;
      u_in_reverse.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->in_reverse = u_in_reverse.real;
      offset += sizeof(this->in_reverse);
     return offset;
    }

    virtual const char * getType() override { return "marti_nav_msgs/Path"; };
    virtual const char * getMD5() override { return "da4d7292371593c2128396b6c7229d46"; };

  };

}
#endif
