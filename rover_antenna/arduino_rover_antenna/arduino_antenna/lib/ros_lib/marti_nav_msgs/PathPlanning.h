#ifndef _ROS_marti_nav_msgs_PathPlanning_h
#define _ROS_marti_nav_msgs_PathPlanning_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace marti_nav_msgs
{

  class PathPlanning : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int32_t _segment_type_type;
      _segment_type_type segment_type;
      typedef float _length_type;
      _length_type length;
      typedef float _start_speed_type;
      _start_speed_type start_speed;
      typedef float _end_speed_type;
      _end_speed_type end_speed;
      typedef float _startx_type;
      _startx_type startx;
      typedef float _starty_type;
      _starty_type starty;
      typedef float _endx_type;
      _endx_type endx;
      typedef float _endy_type;
      _endy_type endy;
      typedef float _theta0_type;
      _theta0_type theta0;
      typedef float _a1_type;
      _a1_type a1;
      typedef float _a2_type;
      _a2_type a2;
      typedef float _k0_type;
      _k0_type k0;
      typedef float _c1_type;
      _c1_type c1;
      typedef float _c2_type;
      _c2_type c2;
      typedef int32_t _behavior_type;
      _behavior_type behavior;
      typedef int32_t _creep_type;
      _creep_type creep;
      typedef int32_t _acc_type;
      _acc_type acc;
      typedef int32_t _reverse_type;
      _reverse_type reverse;
      typedef int32_t _vehicle_track_type;
      _vehicle_track_type vehicle_track;
      typedef bool _transmitted_type;
      _transmitted_type transmitted;
      typedef bool _auxTransmitted_type;
      _auxTransmitted_type auxTransmitted;
      typedef float _theta_end_type;
      _theta_end_type theta_end;
      typedef float _k_end_type;
      _k_end_type k_end;
      typedef int32_t _seg_len_type;
      _seg_len_type seg_len;
      typedef float _speed_limit_type;
      _speed_limit_type speed_limit;
      typedef float _max_error_type;
      _max_error_type max_error;
      typedef float _max_smooth_type;
      _max_smooth_type max_smooth;
      typedef float _max_curv_type;
      _max_curv_type max_curv;
      typedef int32_t _possible_points_type;
      _possible_points_type possible_points;
      typedef bool _bExitSegment_type;
      _bExitSegment_type bExitSegment;

    PathPlanning():
      header(),
      segment_type(0),
      length(0),
      start_speed(0),
      end_speed(0),
      startx(0),
      starty(0),
      endx(0),
      endy(0),
      theta0(0),
      a1(0),
      a2(0),
      k0(0),
      c1(0),
      c2(0),
      behavior(0),
      creep(0),
      acc(0),
      reverse(0),
      vehicle_track(0),
      transmitted(0),
      auxTransmitted(0),
      theta_end(0),
      k_end(0),
      seg_len(0),
      speed_limit(0),
      max_error(0),
      max_smooth(0),
      max_curv(0),
      possible_points(0),
      bExitSegment(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_segment_type;
      u_segment_type.real = this->segment_type;
      *(outbuffer + offset + 0) = (u_segment_type.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_segment_type.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_segment_type.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_segment_type.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->segment_type);
      union {
        float real;
        uint32_t base;
      } u_length;
      u_length.real = this->length;
      *(outbuffer + offset + 0) = (u_length.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_length.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_length.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_length.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->length);
      union {
        float real;
        uint32_t base;
      } u_start_speed;
      u_start_speed.real = this->start_speed;
      *(outbuffer + offset + 0) = (u_start_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_start_speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_start_speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_start_speed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->start_speed);
      union {
        float real;
        uint32_t base;
      } u_end_speed;
      u_end_speed.real = this->end_speed;
      *(outbuffer + offset + 0) = (u_end_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_end_speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_end_speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_end_speed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->end_speed);
      union {
        float real;
        uint32_t base;
      } u_startx;
      u_startx.real = this->startx;
      *(outbuffer + offset + 0) = (u_startx.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_startx.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_startx.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_startx.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->startx);
      union {
        float real;
        uint32_t base;
      } u_starty;
      u_starty.real = this->starty;
      *(outbuffer + offset + 0) = (u_starty.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_starty.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_starty.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_starty.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->starty);
      union {
        float real;
        uint32_t base;
      } u_endx;
      u_endx.real = this->endx;
      *(outbuffer + offset + 0) = (u_endx.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_endx.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_endx.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_endx.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->endx);
      union {
        float real;
        uint32_t base;
      } u_endy;
      u_endy.real = this->endy;
      *(outbuffer + offset + 0) = (u_endy.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_endy.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_endy.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_endy.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->endy);
      union {
        float real;
        uint32_t base;
      } u_theta0;
      u_theta0.real = this->theta0;
      *(outbuffer + offset + 0) = (u_theta0.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_theta0.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_theta0.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_theta0.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->theta0);
      union {
        float real;
        uint32_t base;
      } u_a1;
      u_a1.real = this->a1;
      *(outbuffer + offset + 0) = (u_a1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_a1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_a1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_a1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->a1);
      union {
        float real;
        uint32_t base;
      } u_a2;
      u_a2.real = this->a2;
      *(outbuffer + offset + 0) = (u_a2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_a2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_a2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_a2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->a2);
      union {
        float real;
        uint32_t base;
      } u_k0;
      u_k0.real = this->k0;
      *(outbuffer + offset + 0) = (u_k0.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_k0.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_k0.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_k0.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->k0);
      union {
        float real;
        uint32_t base;
      } u_c1;
      u_c1.real = this->c1;
      *(outbuffer + offset + 0) = (u_c1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_c1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_c1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_c1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->c1);
      union {
        float real;
        uint32_t base;
      } u_c2;
      u_c2.real = this->c2;
      *(outbuffer + offset + 0) = (u_c2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_c2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_c2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_c2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->c2);
      union {
        int32_t real;
        uint32_t base;
      } u_behavior;
      u_behavior.real = this->behavior;
      *(outbuffer + offset + 0) = (u_behavior.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_behavior.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_behavior.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_behavior.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->behavior);
      union {
        int32_t real;
        uint32_t base;
      } u_creep;
      u_creep.real = this->creep;
      *(outbuffer + offset + 0) = (u_creep.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_creep.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_creep.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_creep.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->creep);
      union {
        int32_t real;
        uint32_t base;
      } u_acc;
      u_acc.real = this->acc;
      *(outbuffer + offset + 0) = (u_acc.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_acc.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_acc.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_acc.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->acc);
      union {
        int32_t real;
        uint32_t base;
      } u_reverse;
      u_reverse.real = this->reverse;
      *(outbuffer + offset + 0) = (u_reverse.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_reverse.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_reverse.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_reverse.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->reverse);
      union {
        int32_t real;
        uint32_t base;
      } u_vehicle_track;
      u_vehicle_track.real = this->vehicle_track;
      *(outbuffer + offset + 0) = (u_vehicle_track.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_vehicle_track.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_vehicle_track.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_vehicle_track.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vehicle_track);
      union {
        bool real;
        uint8_t base;
      } u_transmitted;
      u_transmitted.real = this->transmitted;
      *(outbuffer + offset + 0) = (u_transmitted.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->transmitted);
      union {
        bool real;
        uint8_t base;
      } u_auxTransmitted;
      u_auxTransmitted.real = this->auxTransmitted;
      *(outbuffer + offset + 0) = (u_auxTransmitted.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->auxTransmitted);
      union {
        float real;
        uint32_t base;
      } u_theta_end;
      u_theta_end.real = this->theta_end;
      *(outbuffer + offset + 0) = (u_theta_end.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_theta_end.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_theta_end.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_theta_end.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->theta_end);
      union {
        float real;
        uint32_t base;
      } u_k_end;
      u_k_end.real = this->k_end;
      *(outbuffer + offset + 0) = (u_k_end.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_k_end.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_k_end.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_k_end.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->k_end);
      union {
        int32_t real;
        uint32_t base;
      } u_seg_len;
      u_seg_len.real = this->seg_len;
      *(outbuffer + offset + 0) = (u_seg_len.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_seg_len.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_seg_len.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_seg_len.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->seg_len);
      union {
        float real;
        uint32_t base;
      } u_speed_limit;
      u_speed_limit.real = this->speed_limit;
      *(outbuffer + offset + 0) = (u_speed_limit.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed_limit.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speed_limit.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speed_limit.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->speed_limit);
      union {
        float real;
        uint32_t base;
      } u_max_error;
      u_max_error.real = this->max_error;
      *(outbuffer + offset + 0) = (u_max_error.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_error.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_error.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_error.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max_error);
      union {
        float real;
        uint32_t base;
      } u_max_smooth;
      u_max_smooth.real = this->max_smooth;
      *(outbuffer + offset + 0) = (u_max_smooth.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_smooth.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_smooth.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_smooth.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max_smooth);
      union {
        float real;
        uint32_t base;
      } u_max_curv;
      u_max_curv.real = this->max_curv;
      *(outbuffer + offset + 0) = (u_max_curv.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_curv.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_curv.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_curv.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max_curv);
      union {
        int32_t real;
        uint32_t base;
      } u_possible_points;
      u_possible_points.real = this->possible_points;
      *(outbuffer + offset + 0) = (u_possible_points.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_possible_points.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_possible_points.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_possible_points.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->possible_points);
      union {
        bool real;
        uint8_t base;
      } u_bExitSegment;
      u_bExitSegment.real = this->bExitSegment;
      *(outbuffer + offset + 0) = (u_bExitSegment.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->bExitSegment);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_segment_type;
      u_segment_type.base = 0;
      u_segment_type.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_segment_type.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_segment_type.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_segment_type.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->segment_type = u_segment_type.real;
      offset += sizeof(this->segment_type);
      union {
        float real;
        uint32_t base;
      } u_length;
      u_length.base = 0;
      u_length.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_length.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_length.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_length.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->length = u_length.real;
      offset += sizeof(this->length);
      union {
        float real;
        uint32_t base;
      } u_start_speed;
      u_start_speed.base = 0;
      u_start_speed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_start_speed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_start_speed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_start_speed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->start_speed = u_start_speed.real;
      offset += sizeof(this->start_speed);
      union {
        float real;
        uint32_t base;
      } u_end_speed;
      u_end_speed.base = 0;
      u_end_speed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_end_speed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_end_speed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_end_speed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->end_speed = u_end_speed.real;
      offset += sizeof(this->end_speed);
      union {
        float real;
        uint32_t base;
      } u_startx;
      u_startx.base = 0;
      u_startx.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_startx.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_startx.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_startx.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->startx = u_startx.real;
      offset += sizeof(this->startx);
      union {
        float real;
        uint32_t base;
      } u_starty;
      u_starty.base = 0;
      u_starty.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_starty.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_starty.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_starty.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->starty = u_starty.real;
      offset += sizeof(this->starty);
      union {
        float real;
        uint32_t base;
      } u_endx;
      u_endx.base = 0;
      u_endx.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_endx.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_endx.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_endx.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->endx = u_endx.real;
      offset += sizeof(this->endx);
      union {
        float real;
        uint32_t base;
      } u_endy;
      u_endy.base = 0;
      u_endy.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_endy.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_endy.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_endy.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->endy = u_endy.real;
      offset += sizeof(this->endy);
      union {
        float real;
        uint32_t base;
      } u_theta0;
      u_theta0.base = 0;
      u_theta0.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_theta0.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_theta0.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_theta0.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->theta0 = u_theta0.real;
      offset += sizeof(this->theta0);
      union {
        float real;
        uint32_t base;
      } u_a1;
      u_a1.base = 0;
      u_a1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_a1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_a1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_a1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->a1 = u_a1.real;
      offset += sizeof(this->a1);
      union {
        float real;
        uint32_t base;
      } u_a2;
      u_a2.base = 0;
      u_a2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_a2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_a2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_a2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->a2 = u_a2.real;
      offset += sizeof(this->a2);
      union {
        float real;
        uint32_t base;
      } u_k0;
      u_k0.base = 0;
      u_k0.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_k0.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_k0.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_k0.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->k0 = u_k0.real;
      offset += sizeof(this->k0);
      union {
        float real;
        uint32_t base;
      } u_c1;
      u_c1.base = 0;
      u_c1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_c1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_c1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_c1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->c1 = u_c1.real;
      offset += sizeof(this->c1);
      union {
        float real;
        uint32_t base;
      } u_c2;
      u_c2.base = 0;
      u_c2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_c2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_c2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_c2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->c2 = u_c2.real;
      offset += sizeof(this->c2);
      union {
        int32_t real;
        uint32_t base;
      } u_behavior;
      u_behavior.base = 0;
      u_behavior.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_behavior.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_behavior.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_behavior.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->behavior = u_behavior.real;
      offset += sizeof(this->behavior);
      union {
        int32_t real;
        uint32_t base;
      } u_creep;
      u_creep.base = 0;
      u_creep.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_creep.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_creep.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_creep.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->creep = u_creep.real;
      offset += sizeof(this->creep);
      union {
        int32_t real;
        uint32_t base;
      } u_acc;
      u_acc.base = 0;
      u_acc.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_acc.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_acc.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_acc.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->acc = u_acc.real;
      offset += sizeof(this->acc);
      union {
        int32_t real;
        uint32_t base;
      } u_reverse;
      u_reverse.base = 0;
      u_reverse.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_reverse.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_reverse.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_reverse.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->reverse = u_reverse.real;
      offset += sizeof(this->reverse);
      union {
        int32_t real;
        uint32_t base;
      } u_vehicle_track;
      u_vehicle_track.base = 0;
      u_vehicle_track.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_vehicle_track.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_vehicle_track.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_vehicle_track.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->vehicle_track = u_vehicle_track.real;
      offset += sizeof(this->vehicle_track);
      union {
        bool real;
        uint8_t base;
      } u_transmitted;
      u_transmitted.base = 0;
      u_transmitted.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->transmitted = u_transmitted.real;
      offset += sizeof(this->transmitted);
      union {
        bool real;
        uint8_t base;
      } u_auxTransmitted;
      u_auxTransmitted.base = 0;
      u_auxTransmitted.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->auxTransmitted = u_auxTransmitted.real;
      offset += sizeof(this->auxTransmitted);
      union {
        float real;
        uint32_t base;
      } u_theta_end;
      u_theta_end.base = 0;
      u_theta_end.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_theta_end.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_theta_end.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_theta_end.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->theta_end = u_theta_end.real;
      offset += sizeof(this->theta_end);
      union {
        float real;
        uint32_t base;
      } u_k_end;
      u_k_end.base = 0;
      u_k_end.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_k_end.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_k_end.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_k_end.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->k_end = u_k_end.real;
      offset += sizeof(this->k_end);
      union {
        int32_t real;
        uint32_t base;
      } u_seg_len;
      u_seg_len.base = 0;
      u_seg_len.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_seg_len.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_seg_len.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_seg_len.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->seg_len = u_seg_len.real;
      offset += sizeof(this->seg_len);
      union {
        float real;
        uint32_t base;
      } u_speed_limit;
      u_speed_limit.base = 0;
      u_speed_limit.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed_limit.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speed_limit.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speed_limit.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->speed_limit = u_speed_limit.real;
      offset += sizeof(this->speed_limit);
      union {
        float real;
        uint32_t base;
      } u_max_error;
      u_max_error.base = 0;
      u_max_error.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_error.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_error.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_error.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->max_error = u_max_error.real;
      offset += sizeof(this->max_error);
      union {
        float real;
        uint32_t base;
      } u_max_smooth;
      u_max_smooth.base = 0;
      u_max_smooth.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_smooth.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_smooth.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_smooth.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->max_smooth = u_max_smooth.real;
      offset += sizeof(this->max_smooth);
      union {
        float real;
        uint32_t base;
      } u_max_curv;
      u_max_curv.base = 0;
      u_max_curv.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_curv.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_curv.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_curv.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->max_curv = u_max_curv.real;
      offset += sizeof(this->max_curv);
      union {
        int32_t real;
        uint32_t base;
      } u_possible_points;
      u_possible_points.base = 0;
      u_possible_points.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_possible_points.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_possible_points.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_possible_points.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->possible_points = u_possible_points.real;
      offset += sizeof(this->possible_points);
      union {
        bool real;
        uint8_t base;
      } u_bExitSegment;
      u_bExitSegment.base = 0;
      u_bExitSegment.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->bExitSegment = u_bExitSegment.real;
      offset += sizeof(this->bExitSegment);
     return offset;
    }

    virtual const char * getType() override { return "marti_nav_msgs/PathPlanning"; };
    virtual const char * getMD5() override { return "275fd723a6af7fd7f102a3f07eca0829"; };

  };

}
#endif
