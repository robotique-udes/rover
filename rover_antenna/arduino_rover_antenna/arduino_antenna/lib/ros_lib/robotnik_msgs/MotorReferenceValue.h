#ifndef _ROS_robotnik_msgs_MotorReferenceValue_h
#define _ROS_robotnik_msgs_MotorReferenceValue_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robotnik_msgs
{

  class MotorReferenceValue : public ros::Msg
  {
    public:
      typedef float _target_value_type;
      _target_value_type target_value;
      typedef int32_t _target_ref_type;
      _target_ref_type target_ref;
      typedef float _actual_value_type;
      _actual_value_type actual_value;
      typedef int32_t _actual_ref_type;
      _actual_ref_type actual_ref;
      typedef float _spin_type;
      _spin_type spin;
      typedef float _offset_type;
      _offset_type offset;

    MotorReferenceValue():
      target_value(0),
      target_ref(0),
      actual_value(0),
      actual_ref(0),
      spin(0),
      offset(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_target_value;
      u_target_value.real = this->target_value;
      *(outbuffer + offset + 0) = (u_target_value.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_target_value.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_target_value.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_target_value.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->target_value);
      union {
        int32_t real;
        uint32_t base;
      } u_target_ref;
      u_target_ref.real = this->target_ref;
      *(outbuffer + offset + 0) = (u_target_ref.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_target_ref.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_target_ref.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_target_ref.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->target_ref);
      union {
        float real;
        uint32_t base;
      } u_actual_value;
      u_actual_value.real = this->actual_value;
      *(outbuffer + offset + 0) = (u_actual_value.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_actual_value.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_actual_value.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_actual_value.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->actual_value);
      union {
        int32_t real;
        uint32_t base;
      } u_actual_ref;
      u_actual_ref.real = this->actual_ref;
      *(outbuffer + offset + 0) = (u_actual_ref.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_actual_ref.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_actual_ref.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_actual_ref.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->actual_ref);
      union {
        float real;
        uint32_t base;
      } u_spin;
      u_spin.real = this->spin;
      *(outbuffer + offset + 0) = (u_spin.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_spin.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_spin.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_spin.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->spin);
      union {
        float real;
        uint32_t base;
      } u_offset;
      u_offset.real = this->offset;
      *(outbuffer + offset + 0) = (u_offset.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_offset.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_offset.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_offset.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_target_value;
      u_target_value.base = 0;
      u_target_value.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_target_value.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_target_value.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_target_value.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->target_value = u_target_value.real;
      offset += sizeof(this->target_value);
      union {
        int32_t real;
        uint32_t base;
      } u_target_ref;
      u_target_ref.base = 0;
      u_target_ref.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_target_ref.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_target_ref.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_target_ref.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->target_ref = u_target_ref.real;
      offset += sizeof(this->target_ref);
      union {
        float real;
        uint32_t base;
      } u_actual_value;
      u_actual_value.base = 0;
      u_actual_value.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_actual_value.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_actual_value.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_actual_value.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->actual_value = u_actual_value.real;
      offset += sizeof(this->actual_value);
      union {
        int32_t real;
        uint32_t base;
      } u_actual_ref;
      u_actual_ref.base = 0;
      u_actual_ref.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_actual_ref.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_actual_ref.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_actual_ref.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->actual_ref = u_actual_ref.real;
      offset += sizeof(this->actual_ref);
      union {
        float real;
        uint32_t base;
      } u_spin;
      u_spin.base = 0;
      u_spin.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_spin.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_spin.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_spin.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->spin = u_spin.real;
      offset += sizeof(this->spin);
      union {
        float real;
        uint32_t base;
      } u_offset;
      u_offset.base = 0;
      u_offset.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_offset.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_offset.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_offset.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->offset = u_offset.real;
      offset += sizeof(this->offset);
     return offset;
    }

    virtual const char * getType() override { return "robotnik_msgs/MotorReferenceValue"; };
    virtual const char * getMD5() override { return "bee092bed901770c06bbd09306ce8598"; };

  };

}
#endif
