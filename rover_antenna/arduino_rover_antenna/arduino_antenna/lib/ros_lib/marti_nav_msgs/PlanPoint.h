#ifndef _ROS_marti_nav_msgs_PlanPoint_h
#define _ROS_marti_nav_msgs_PlanPoint_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace marti_nav_msgs
{

  class PlanPoint : public ros::Msg
  {
    public:
      typedef float _x_type;
      _x_type x;
      typedef float _y_type;
      _y_type y;
      typedef float _z_type;
      _z_type z;
      typedef float _yaw_type;
      _yaw_type yaw;
      typedef float _speed_type;
      _speed_type speed;
      typedef int64_t _lane_id_type;
      _lane_id_type lane_id;
      typedef int64_t _segment_id_type;
      _segment_id_type segment_id;
      typedef int32_t _flags_type;
      _flags_type flags;
      enum { FLAG_REVERSE =  1 };
      enum { FLAG_STOP_POINT =  2 };

    PlanPoint():
      x(0),
      y(0),
      z(0),
      yaw(0),
      speed(0),
      lane_id(0),
      segment_id(0),
      flags(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->x);
      offset += serializeAvrFloat64(outbuffer + offset, this->y);
      offset += serializeAvrFloat64(outbuffer + offset, this->z);
      offset += serializeAvrFloat64(outbuffer + offset, this->yaw);
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
        int64_t real;
        uint64_t base;
      } u_lane_id;
      u_lane_id.real = this->lane_id;
      *(outbuffer + offset + 0) = (u_lane_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_lane_id.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_lane_id.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_lane_id.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_lane_id.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_lane_id.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_lane_id.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_lane_id.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->lane_id);
      union {
        int64_t real;
        uint64_t base;
      } u_segment_id;
      u_segment_id.real = this->segment_id;
      *(outbuffer + offset + 0) = (u_segment_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_segment_id.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_segment_id.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_segment_id.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_segment_id.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_segment_id.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_segment_id.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_segment_id.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->segment_id);
      union {
        int32_t real;
        uint32_t base;
      } u_flags;
      u_flags.real = this->flags;
      *(outbuffer + offset + 0) = (u_flags.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_flags.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_flags.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_flags.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->flags);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->x));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->y));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->z));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->yaw));
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
        int64_t real;
        uint64_t base;
      } u_lane_id;
      u_lane_id.base = 0;
      u_lane_id.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_lane_id.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_lane_id.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_lane_id.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_lane_id.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_lane_id.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_lane_id.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_lane_id.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->lane_id = u_lane_id.real;
      offset += sizeof(this->lane_id);
      union {
        int64_t real;
        uint64_t base;
      } u_segment_id;
      u_segment_id.base = 0;
      u_segment_id.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_segment_id.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_segment_id.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_segment_id.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_segment_id.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_segment_id.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_segment_id.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_segment_id.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->segment_id = u_segment_id.real;
      offset += sizeof(this->segment_id);
      union {
        int32_t real;
        uint32_t base;
      } u_flags;
      u_flags.base = 0;
      u_flags.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_flags.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_flags.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_flags.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->flags = u_flags.real;
      offset += sizeof(this->flags);
     return offset;
    }

    virtual const char * getType() override { return "marti_nav_msgs/PlanPoint"; };
    virtual const char * getMD5() override { return "d755606289437b8371d55cbf2789ea2d"; };

  };

}
#endif
