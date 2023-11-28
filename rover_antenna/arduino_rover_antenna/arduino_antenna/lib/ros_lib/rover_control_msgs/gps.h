#ifndef _ROS_rover_control_msgs_gps_h
#define _ROS_rover_control_msgs_gps_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rover_control_msgs
{

  class gps : public ros::Msg
  {
    public:
      typedef float _latitude_type;
      _latitude_type latitude;
      typedef float _longitude_type;
      _longitude_type longitude;
      typedef float _heading_track_type;
      _heading_track_type heading_track;
      typedef float _height_type;
      _height_type height;
      typedef uint8_t _fix_quality_type;
      _fix_quality_type fix_quality;
      typedef uint8_t _satellite_type;
      _satellite_type satellite;
      typedef float _heading_gps_type;
      _heading_gps_type heading_gps;
      typedef float _speed_type;
      _speed_type speed;
      enum { FIX_INVALID =  0 };
      enum { FIX_STANDALONE =  1 };
      enum { FIX_DGPS =  2 };
      enum { FIX_RTK_FIXED =  4 };
      enum { FIX_RTK_FLOAT =  5 };

    gps():
      latitude(0),
      longitude(0),
      heading_track(0),
      height(0),
      fix_quality(0),
      satellite(0),
      heading_gps(0),
      speed(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_latitude;
      u_latitude.real = this->latitude;
      *(outbuffer + offset + 0) = (u_latitude.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_latitude.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_latitude.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_latitude.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->latitude);
      union {
        float real;
        uint32_t base;
      } u_longitude;
      u_longitude.real = this->longitude;
      *(outbuffer + offset + 0) = (u_longitude.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_longitude.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_longitude.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_longitude.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->longitude);
      union {
        float real;
        uint32_t base;
      } u_heading_track;
      u_heading_track.real = this->heading_track;
      *(outbuffer + offset + 0) = (u_heading_track.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_heading_track.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_heading_track.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_heading_track.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->heading_track);
      union {
        float real;
        uint32_t base;
      } u_height;
      u_height.real = this->height;
      *(outbuffer + offset + 0) = (u_height.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_height.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_height.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_height.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->height);
      *(outbuffer + offset + 0) = (this->fix_quality >> (8 * 0)) & 0xFF;
      offset += sizeof(this->fix_quality);
      *(outbuffer + offset + 0) = (this->satellite >> (8 * 0)) & 0xFF;
      offset += sizeof(this->satellite);
      union {
        float real;
        uint32_t base;
      } u_heading_gps;
      u_heading_gps.real = this->heading_gps;
      *(outbuffer + offset + 0) = (u_heading_gps.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_heading_gps.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_heading_gps.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_heading_gps.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->heading_gps);
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
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_latitude;
      u_latitude.base = 0;
      u_latitude.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_latitude.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_latitude.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_latitude.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->latitude = u_latitude.real;
      offset += sizeof(this->latitude);
      union {
        float real;
        uint32_t base;
      } u_longitude;
      u_longitude.base = 0;
      u_longitude.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_longitude.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_longitude.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_longitude.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->longitude = u_longitude.real;
      offset += sizeof(this->longitude);
      union {
        float real;
        uint32_t base;
      } u_heading_track;
      u_heading_track.base = 0;
      u_heading_track.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_heading_track.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_heading_track.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_heading_track.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->heading_track = u_heading_track.real;
      offset += sizeof(this->heading_track);
      union {
        float real;
        uint32_t base;
      } u_height;
      u_height.base = 0;
      u_height.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_height.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_height.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_height.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->height = u_height.real;
      offset += sizeof(this->height);
      this->fix_quality =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->fix_quality);
      this->satellite =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->satellite);
      union {
        float real;
        uint32_t base;
      } u_heading_gps;
      u_heading_gps.base = 0;
      u_heading_gps.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_heading_gps.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_heading_gps.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_heading_gps.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->heading_gps = u_heading_gps.real;
      offset += sizeof(this->heading_gps);
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
     return offset;
    }

    virtual const char * getType() override { return "rover_control_msgs/gps"; };
    virtual const char * getMD5() override { return "4f73e2b2ca603b127e3cff57ff942a84"; };

  };

}
#endif
