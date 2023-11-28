#ifndef _ROS_marti_nav_msgs_TrackedObject_h
#define _ROS_marti_nav_msgs_TrackedObject_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/TwistWithCovariance.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Point.h"
#include "ros/duration.h"

namespace marti_nav_msgs
{

  class TrackedObject : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint16_t _id_type;
      _id_type id;
      typedef geometry_msgs::PoseWithCovariance _pose_type;
      _pose_type pose;
      typedef geometry_msgs::TwistWithCovariance _velocity_type;
      _velocity_type velocity;
      typedef geometry_msgs::Vector3 _linear_acceleration_type;
      _linear_acceleration_type linear_acceleration;
      float linear_acceleration_covariance[9];
      uint32_t polygon_length;
      typedef geometry_msgs::Point _polygon_type;
      _polygon_type st_polygon;
      _polygon_type * polygon;
      typedef float _length_type;
      _length_type length;
      typedef float _length_quality_type;
      _length_quality_type length_quality;
      typedef float _width_type;
      _width_type width;
      typedef float _width_quality_type;
      _width_quality_type width_quality;
      typedef uint8_t _classification_type;
      _classification_type classification;
      typedef float _classification_quality_type;
      _classification_quality_type classification_quality;
      typedef float _existence_probability_type;
      _existence_probability_type existence_probability;
      typedef ros::Duration _age_duration_type;
      _age_duration_type age_duration;
      typedef ros::Duration _prediction_duration_type;
      _prediction_duration_type prediction_duration;
      typedef bool _active_type;
      _active_type active;
      enum { VEHICLE = 0 };
      enum { PEDESTRIAN = 1 };
      enum { UNKNOWN = 255 };

    TrackedObject():
      header(),
      id(0),
      pose(),
      velocity(),
      linear_acceleration(),
      linear_acceleration_covariance(),
      polygon_length(0), st_polygon(), polygon(nullptr),
      length(0),
      length_quality(0),
      width(0),
      width_quality(0),
      classification(0),
      classification_quality(0),
      existence_probability(0),
      age_duration(),
      prediction_duration(),
      active(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->id >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->id >> (8 * 1)) & 0xFF;
      offset += sizeof(this->id);
      offset += this->pose.serialize(outbuffer + offset);
      offset += this->velocity.serialize(outbuffer + offset);
      offset += this->linear_acceleration.serialize(outbuffer + offset);
      for( uint32_t i = 0; i < 9; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->linear_acceleration_covariance[i]);
      }
      *(outbuffer + offset + 0) = (this->polygon_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->polygon_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->polygon_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->polygon_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->polygon_length);
      for( uint32_t i = 0; i < polygon_length; i++){
      offset += this->polygon[i].serialize(outbuffer + offset);
      }
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
      } u_length_quality;
      u_length_quality.real = this->length_quality;
      *(outbuffer + offset + 0) = (u_length_quality.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_length_quality.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_length_quality.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_length_quality.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->length_quality);
      union {
        float real;
        uint32_t base;
      } u_width;
      u_width.real = this->width;
      *(outbuffer + offset + 0) = (u_width.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_width.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_width.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_width.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->width);
      union {
        float real;
        uint32_t base;
      } u_width_quality;
      u_width_quality.real = this->width_quality;
      *(outbuffer + offset + 0) = (u_width_quality.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_width_quality.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_width_quality.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_width_quality.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->width_quality);
      *(outbuffer + offset + 0) = (this->classification >> (8 * 0)) & 0xFF;
      offset += sizeof(this->classification);
      union {
        float real;
        uint32_t base;
      } u_classification_quality;
      u_classification_quality.real = this->classification_quality;
      *(outbuffer + offset + 0) = (u_classification_quality.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_classification_quality.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_classification_quality.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_classification_quality.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->classification_quality);
      union {
        float real;
        uint32_t base;
      } u_existence_probability;
      u_existence_probability.real = this->existence_probability;
      *(outbuffer + offset + 0) = (u_existence_probability.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_existence_probability.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_existence_probability.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_existence_probability.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->existence_probability);
      *(outbuffer + offset + 0) = (this->age_duration.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->age_duration.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->age_duration.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->age_duration.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->age_duration.sec);
      *(outbuffer + offset + 0) = (this->age_duration.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->age_duration.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->age_duration.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->age_duration.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->age_duration.nsec);
      *(outbuffer + offset + 0) = (this->prediction_duration.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->prediction_duration.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->prediction_duration.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->prediction_duration.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->prediction_duration.sec);
      *(outbuffer + offset + 0) = (this->prediction_duration.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->prediction_duration.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->prediction_duration.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->prediction_duration.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->prediction_duration.nsec);
      union {
        bool real;
        uint8_t base;
      } u_active;
      u_active.real = this->active;
      *(outbuffer + offset + 0) = (u_active.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->active);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->id =  ((uint16_t) (*(inbuffer + offset)));
      this->id |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->id);
      offset += this->pose.deserialize(inbuffer + offset);
      offset += this->velocity.deserialize(inbuffer + offset);
      offset += this->linear_acceleration.deserialize(inbuffer + offset);
      for( uint32_t i = 0; i < 9; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->linear_acceleration_covariance[i]));
      }
      uint32_t polygon_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      polygon_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      polygon_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      polygon_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->polygon_length);
      if(polygon_lengthT > polygon_length)
        this->polygon = (geometry_msgs::Point*)realloc(this->polygon, polygon_lengthT * sizeof(geometry_msgs::Point));
      polygon_length = polygon_lengthT;
      for( uint32_t i = 0; i < polygon_length; i++){
      offset += this->st_polygon.deserialize(inbuffer + offset);
        memcpy( &(this->polygon[i]), &(this->st_polygon), sizeof(geometry_msgs::Point));
      }
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
      } u_length_quality;
      u_length_quality.base = 0;
      u_length_quality.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_length_quality.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_length_quality.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_length_quality.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->length_quality = u_length_quality.real;
      offset += sizeof(this->length_quality);
      union {
        float real;
        uint32_t base;
      } u_width;
      u_width.base = 0;
      u_width.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_width.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_width.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_width.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->width = u_width.real;
      offset += sizeof(this->width);
      union {
        float real;
        uint32_t base;
      } u_width_quality;
      u_width_quality.base = 0;
      u_width_quality.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_width_quality.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_width_quality.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_width_quality.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->width_quality = u_width_quality.real;
      offset += sizeof(this->width_quality);
      this->classification =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->classification);
      union {
        float real;
        uint32_t base;
      } u_classification_quality;
      u_classification_quality.base = 0;
      u_classification_quality.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_classification_quality.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_classification_quality.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_classification_quality.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->classification_quality = u_classification_quality.real;
      offset += sizeof(this->classification_quality);
      union {
        float real;
        uint32_t base;
      } u_existence_probability;
      u_existence_probability.base = 0;
      u_existence_probability.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_existence_probability.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_existence_probability.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_existence_probability.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->existence_probability = u_existence_probability.real;
      offset += sizeof(this->existence_probability);
      this->age_duration.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->age_duration.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->age_duration.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->age_duration.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->age_duration.sec);
      this->age_duration.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->age_duration.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->age_duration.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->age_duration.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->age_duration.nsec);
      this->prediction_duration.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->prediction_duration.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->prediction_duration.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->prediction_duration.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->prediction_duration.sec);
      this->prediction_duration.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->prediction_duration.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->prediction_duration.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->prediction_duration.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->prediction_duration.nsec);
      union {
        bool real;
        uint8_t base;
      } u_active;
      u_active.base = 0;
      u_active.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->active = u_active.real;
      offset += sizeof(this->active);
     return offset;
    }

    virtual const char * getType() override { return "marti_nav_msgs/TrackedObject"; };
    virtual const char * getMD5() override { return "cbb29999f8b089f4f41612b25a8b7540"; };

  };

}
#endif
