#ifndef _ROS_differential_drive_Encoders_h
#define _ROS_differential_drive_Encoders_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace differential_drive
{

  class Encoders : public ros::Msg
  {
    public:
      typedef int32_t _left_encoder_type;
      _left_encoder_type left_encoder;
      typedef int16_t _left_encoder_multiplier_type;
      _left_encoder_multiplier_type left_encoder_multiplier;
      typedef int32_t _right_encoder_type;
      _right_encoder_type right_encoder;
      typedef int16_t _right_encoder_multiplier_type;
      _right_encoder_multiplier_type right_encoder_multiplier;

    Encoders():
      left_encoder(0),
      left_encoder_multiplier(0),
      right_encoder(0),
      right_encoder_multiplier(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_left_encoder;
      u_left_encoder.real = this->left_encoder;
      *(outbuffer + offset + 0) = (u_left_encoder.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_encoder.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_encoder.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_encoder.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_encoder);
      union {
        int16_t real;
        uint16_t base;
      } u_left_encoder_multiplier;
      u_left_encoder_multiplier.real = this->left_encoder_multiplier;
      *(outbuffer + offset + 0) = (u_left_encoder_multiplier.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_encoder_multiplier.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->left_encoder_multiplier);
      union {
        int32_t real;
        uint32_t base;
      } u_right_encoder;
      u_right_encoder.real = this->right_encoder;
      *(outbuffer + offset + 0) = (u_right_encoder.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_encoder.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_encoder.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_encoder.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_encoder);
      union {
        int16_t real;
        uint16_t base;
      } u_right_encoder_multiplier;
      u_right_encoder_multiplier.real = this->right_encoder_multiplier;
      *(outbuffer + offset + 0) = (u_right_encoder_multiplier.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_encoder_multiplier.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->right_encoder_multiplier);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_left_encoder;
      u_left_encoder.base = 0;
      u_left_encoder.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_encoder.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_encoder.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_encoder.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_encoder = u_left_encoder.real;
      offset += sizeof(this->left_encoder);
      union {
        int16_t real;
        uint16_t base;
      } u_left_encoder_multiplier;
      u_left_encoder_multiplier.base = 0;
      u_left_encoder_multiplier.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_encoder_multiplier.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->left_encoder_multiplier = u_left_encoder_multiplier.real;
      offset += sizeof(this->left_encoder_multiplier);
      union {
        int32_t real;
        uint32_t base;
      } u_right_encoder;
      u_right_encoder.base = 0;
      u_right_encoder.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_encoder.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_encoder.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_encoder.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_encoder = u_right_encoder.real;
      offset += sizeof(this->right_encoder);
      union {
        int16_t real;
        uint16_t base;
      } u_right_encoder_multiplier;
      u_right_encoder_multiplier.base = 0;
      u_right_encoder_multiplier.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_encoder_multiplier.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->right_encoder_multiplier = u_right_encoder_multiplier.real;
      offset += sizeof(this->right_encoder_multiplier);
     return offset;
    }

    virtual const char * getType() override { return "differential_drive/Encoders"; };
    virtual const char * getMD5() override { return "75b18dbef2e767d8db3dba7c3689dc5b"; };

  };

}
#endif
