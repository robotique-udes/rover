#ifndef _ROS_rover_arm_msgs_feedback_h
#define _ROS_rover_arm_msgs_feedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rover_arm_msgs
{

  class feedback : public ros::Msg
  {
    public:
      float angles[6];
      typedef bool _singular_matrix_type;
      _singular_matrix_type singular_matrix;
      float vitesses[6];
      bool enable[6];
      typedef bool _ctrl_mode_type;
      _ctrl_mode_type ctrl_mode;
      typedef int8_t _current_joint_type;
      _current_joint_type current_joint;
      typedef float _speed_multiplier_type;
      _speed_multiplier_type speed_multiplier;
      typedef bool _limiteur_type;
      _limiteur_type limiteur;
      typedef bool _calibration_type;
      _calibration_type calibration;
      typedef bool _kinetics_calc_error_type;
      _kinetics_calc_error_type kinetics_calc_error;

    feedback():
      angles(),
      singular_matrix(0),
      vitesses(),
      enable(),
      ctrl_mode(0),
      current_joint(0),
      speed_multiplier(0),
      limiteur(0),
      calibration(0),
      kinetics_calc_error(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 6; i++){
      union {
        float real;
        uint32_t base;
      } u_anglesi;
      u_anglesi.real = this->angles[i];
      *(outbuffer + offset + 0) = (u_anglesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_anglesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_anglesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_anglesi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angles[i]);
      }
      union {
        bool real;
        uint8_t base;
      } u_singular_matrix;
      u_singular_matrix.real = this->singular_matrix;
      *(outbuffer + offset + 0) = (u_singular_matrix.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->singular_matrix);
      for( uint32_t i = 0; i < 6; i++){
      union {
        float real;
        uint32_t base;
      } u_vitessesi;
      u_vitessesi.real = this->vitesses[i];
      *(outbuffer + offset + 0) = (u_vitessesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_vitessesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_vitessesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_vitessesi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vitesses[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      union {
        bool real;
        uint8_t base;
      } u_enablei;
      u_enablei.real = this->enable[i];
      *(outbuffer + offset + 0) = (u_enablei.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->enable[i]);
      }
      union {
        bool real;
        uint8_t base;
      } u_ctrl_mode;
      u_ctrl_mode.real = this->ctrl_mode;
      *(outbuffer + offset + 0) = (u_ctrl_mode.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ctrl_mode);
      union {
        int8_t real;
        uint8_t base;
      } u_current_joint;
      u_current_joint.real = this->current_joint;
      *(outbuffer + offset + 0) = (u_current_joint.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->current_joint);
      union {
        float real;
        uint32_t base;
      } u_speed_multiplier;
      u_speed_multiplier.real = this->speed_multiplier;
      *(outbuffer + offset + 0) = (u_speed_multiplier.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed_multiplier.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speed_multiplier.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speed_multiplier.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->speed_multiplier);
      union {
        bool real;
        uint8_t base;
      } u_limiteur;
      u_limiteur.real = this->limiteur;
      *(outbuffer + offset + 0) = (u_limiteur.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->limiteur);
      union {
        bool real;
        uint8_t base;
      } u_calibration;
      u_calibration.real = this->calibration;
      *(outbuffer + offset + 0) = (u_calibration.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->calibration);
      union {
        bool real;
        uint8_t base;
      } u_kinetics_calc_error;
      u_kinetics_calc_error.real = this->kinetics_calc_error;
      *(outbuffer + offset + 0) = (u_kinetics_calc_error.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->kinetics_calc_error);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 6; i++){
      union {
        float real;
        uint32_t base;
      } u_anglesi;
      u_anglesi.base = 0;
      u_anglesi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_anglesi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_anglesi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_anglesi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->angles[i] = u_anglesi.real;
      offset += sizeof(this->angles[i]);
      }
      union {
        bool real;
        uint8_t base;
      } u_singular_matrix;
      u_singular_matrix.base = 0;
      u_singular_matrix.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->singular_matrix = u_singular_matrix.real;
      offset += sizeof(this->singular_matrix);
      for( uint32_t i = 0; i < 6; i++){
      union {
        float real;
        uint32_t base;
      } u_vitessesi;
      u_vitessesi.base = 0;
      u_vitessesi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_vitessesi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_vitessesi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_vitessesi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->vitesses[i] = u_vitessesi.real;
      offset += sizeof(this->vitesses[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      union {
        bool real;
        uint8_t base;
      } u_enablei;
      u_enablei.base = 0;
      u_enablei.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->enable[i] = u_enablei.real;
      offset += sizeof(this->enable[i]);
      }
      union {
        bool real;
        uint8_t base;
      } u_ctrl_mode;
      u_ctrl_mode.base = 0;
      u_ctrl_mode.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ctrl_mode = u_ctrl_mode.real;
      offset += sizeof(this->ctrl_mode);
      union {
        int8_t real;
        uint8_t base;
      } u_current_joint;
      u_current_joint.base = 0;
      u_current_joint.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->current_joint = u_current_joint.real;
      offset += sizeof(this->current_joint);
      union {
        float real;
        uint32_t base;
      } u_speed_multiplier;
      u_speed_multiplier.base = 0;
      u_speed_multiplier.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed_multiplier.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speed_multiplier.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speed_multiplier.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->speed_multiplier = u_speed_multiplier.real;
      offset += sizeof(this->speed_multiplier);
      union {
        bool real;
        uint8_t base;
      } u_limiteur;
      u_limiteur.base = 0;
      u_limiteur.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->limiteur = u_limiteur.real;
      offset += sizeof(this->limiteur);
      union {
        bool real;
        uint8_t base;
      } u_calibration;
      u_calibration.base = 0;
      u_calibration.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->calibration = u_calibration.real;
      offset += sizeof(this->calibration);
      union {
        bool real;
        uint8_t base;
      } u_kinetics_calc_error;
      u_kinetics_calc_error.base = 0;
      u_kinetics_calc_error.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->kinetics_calc_error = u_kinetics_calc_error.real;
      offset += sizeof(this->kinetics_calc_error);
     return offset;
    }

    virtual const char * getType() override { return "rover_arm_msgs/feedback"; };
    virtual const char * getMD5() override { return "9ca5d0ab24a28a6ff02c7a386bfaacad"; };

  };

}
#endif
