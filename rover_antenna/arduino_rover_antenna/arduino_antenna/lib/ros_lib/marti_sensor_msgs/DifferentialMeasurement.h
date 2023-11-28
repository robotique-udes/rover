#ifndef _ROS_marti_sensor_msgs_DifferentialMeasurement_h
#define _ROS_marti_sensor_msgs_DifferentialMeasurement_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Vector3.h"

namespace marti_sensor_msgs
{

  class DifferentialMeasurement : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _base_frame_id_type;
      _base_frame_id_type base_frame_id;
      typedef float _baseline_length_type;
      _baseline_length_type baseline_length;
      typedef float _baseline_length_variance_type;
      _baseline_length_variance_type baseline_length_variance;
      typedef float _heading_type;
      _heading_type heading;
      typedef float _heading_variance_type;
      _heading_variance_type heading_variance;
      typedef float _pitch_type;
      _pitch_type pitch;
      typedef float _pitch_variance_type;
      _pitch_variance_type pitch_variance;
      typedef float _roll_type;
      _roll_type roll;
      typedef float _roll_variance_type;
      _roll_variance_type roll_variance;
      typedef geometry_msgs::Vector3 _position_type;
      _position_type position;
      float position_covariance[9];

    DifferentialMeasurement():
      header(),
      base_frame_id(""),
      baseline_length(0),
      baseline_length_variance(0),
      heading(0),
      heading_variance(0),
      pitch(0),
      pitch_variance(0),
      roll(0),
      roll_variance(0),
      position(),
      position_covariance()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_base_frame_id = strlen(this->base_frame_id);
      varToArr(outbuffer + offset, length_base_frame_id);
      offset += 4;
      memcpy(outbuffer + offset, this->base_frame_id, length_base_frame_id);
      offset += length_base_frame_id;
      offset += serializeAvrFloat64(outbuffer + offset, this->baseline_length);
      offset += serializeAvrFloat64(outbuffer + offset, this->baseline_length_variance);
      offset += serializeAvrFloat64(outbuffer + offset, this->heading);
      offset += serializeAvrFloat64(outbuffer + offset, this->heading_variance);
      offset += serializeAvrFloat64(outbuffer + offset, this->pitch);
      offset += serializeAvrFloat64(outbuffer + offset, this->pitch_variance);
      offset += serializeAvrFloat64(outbuffer + offset, this->roll);
      offset += serializeAvrFloat64(outbuffer + offset, this->roll_variance);
      offset += this->position.serialize(outbuffer + offset);
      for( uint32_t i = 0; i < 9; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->position_covariance[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_base_frame_id;
      arrToVar(length_base_frame_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_base_frame_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_base_frame_id-1]=0;
      this->base_frame_id = (char *)(inbuffer + offset-1);
      offset += length_base_frame_id;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->baseline_length));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->baseline_length_variance));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->heading));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->heading_variance));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->pitch));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->pitch_variance));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->roll));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->roll_variance));
      offset += this->position.deserialize(inbuffer + offset);
      for( uint32_t i = 0; i < 9; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->position_covariance[i]));
      }
     return offset;
    }

    virtual const char * getType() override { return "marti_sensor_msgs/DifferentialMeasurement"; };
    virtual const char * getMD5() override { return "4e59a00b1eeea864eec73f05dd359752"; };

  };

}
#endif
