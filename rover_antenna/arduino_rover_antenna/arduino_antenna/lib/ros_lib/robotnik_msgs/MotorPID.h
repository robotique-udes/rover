#ifndef _ROS_robotnik_msgs_MotorPID_h
#define _ROS_robotnik_msgs_MotorPID_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robotnik_msgs
{

  class MotorPID : public ros::Msg
  {
    public:
      uint32_t can_id_length;
      typedef int32_t _can_id_type;
      _can_id_type st_can_id;
      _can_id_type * can_id;
      uint32_t name_length;
      typedef char* _name_type;
      _name_type st_name;
      _name_type * name;
      uint32_t kp_length;
      typedef float _kp_type;
      _kp_type st_kp;
      _kp_type * kp;
      uint32_t ki_length;
      typedef float _ki_type;
      _ki_type st_ki;
      _ki_type * ki;
      uint32_t kd_length;
      typedef float _kd_type;
      _kd_type st_kd;
      _kd_type * kd;

    MotorPID():
      can_id_length(0), st_can_id(), can_id(nullptr),
      name_length(0), st_name(), name(nullptr),
      kp_length(0), st_kp(), kp(nullptr),
      ki_length(0), st_ki(), ki(nullptr),
      kd_length(0), st_kd(), kd(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->can_id_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->can_id_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->can_id_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->can_id_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->can_id_length);
      for( uint32_t i = 0; i < can_id_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_can_idi;
      u_can_idi.real = this->can_id[i];
      *(outbuffer + offset + 0) = (u_can_idi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_can_idi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_can_idi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_can_idi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->can_id[i]);
      }
      *(outbuffer + offset + 0) = (this->name_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->name_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->name_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->name_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->name_length);
      for( uint32_t i = 0; i < name_length; i++){
      uint32_t length_namei = strlen(this->name[i]);
      varToArr(outbuffer + offset, length_namei);
      offset += 4;
      memcpy(outbuffer + offset, this->name[i], length_namei);
      offset += length_namei;
      }
      *(outbuffer + offset + 0) = (this->kp_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->kp_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->kp_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->kp_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->kp_length);
      for( uint32_t i = 0; i < kp_length; i++){
      union {
        float real;
        uint32_t base;
      } u_kpi;
      u_kpi.real = this->kp[i];
      *(outbuffer + offset + 0) = (u_kpi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kpi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kpi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kpi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->kp[i]);
      }
      *(outbuffer + offset + 0) = (this->ki_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->ki_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->ki_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->ki_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ki_length);
      for( uint32_t i = 0; i < ki_length; i++){
      union {
        float real;
        uint32_t base;
      } u_kii;
      u_kii.real = this->ki[i];
      *(outbuffer + offset + 0) = (u_kii.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kii.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kii.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kii.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ki[i]);
      }
      *(outbuffer + offset + 0) = (this->kd_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->kd_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->kd_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->kd_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->kd_length);
      for( uint32_t i = 0; i < kd_length; i++){
      union {
        float real;
        uint32_t base;
      } u_kdi;
      u_kdi.real = this->kd[i];
      *(outbuffer + offset + 0) = (u_kdi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kdi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kdi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kdi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->kd[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t can_id_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      can_id_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      can_id_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      can_id_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->can_id_length);
      if(can_id_lengthT > can_id_length)
        this->can_id = (int32_t*)realloc(this->can_id, can_id_lengthT * sizeof(int32_t));
      can_id_length = can_id_lengthT;
      for( uint32_t i = 0; i < can_id_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_can_id;
      u_st_can_id.base = 0;
      u_st_can_id.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_can_id.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_can_id.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_can_id.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_can_id = u_st_can_id.real;
      offset += sizeof(this->st_can_id);
        memcpy( &(this->can_id[i]), &(this->st_can_id), sizeof(int32_t));
      }
      uint32_t name_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      name_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      name_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      name_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->name_length);
      if(name_lengthT > name_length)
        this->name = (char**)realloc(this->name, name_lengthT * sizeof(char*));
      name_length = name_lengthT;
      for( uint32_t i = 0; i < name_length; i++){
      uint32_t length_st_name;
      arrToVar(length_st_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_name-1]=0;
      this->st_name = (char *)(inbuffer + offset-1);
      offset += length_st_name;
        memcpy( &(this->name[i]), &(this->st_name), sizeof(char*));
      }
      uint32_t kp_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      kp_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      kp_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      kp_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->kp_length);
      if(kp_lengthT > kp_length)
        this->kp = (float*)realloc(this->kp, kp_lengthT * sizeof(float));
      kp_length = kp_lengthT;
      for( uint32_t i = 0; i < kp_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_kp;
      u_st_kp.base = 0;
      u_st_kp.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_kp.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_kp.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_kp.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_kp = u_st_kp.real;
      offset += sizeof(this->st_kp);
        memcpy( &(this->kp[i]), &(this->st_kp), sizeof(float));
      }
      uint32_t ki_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      ki_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      ki_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      ki_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->ki_length);
      if(ki_lengthT > ki_length)
        this->ki = (float*)realloc(this->ki, ki_lengthT * sizeof(float));
      ki_length = ki_lengthT;
      for( uint32_t i = 0; i < ki_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_ki;
      u_st_ki.base = 0;
      u_st_ki.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_ki.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_ki.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_ki.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_ki = u_st_ki.real;
      offset += sizeof(this->st_ki);
        memcpy( &(this->ki[i]), &(this->st_ki), sizeof(float));
      }
      uint32_t kd_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      kd_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      kd_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      kd_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->kd_length);
      if(kd_lengthT > kd_length)
        this->kd = (float*)realloc(this->kd, kd_lengthT * sizeof(float));
      kd_length = kd_lengthT;
      for( uint32_t i = 0; i < kd_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_kd;
      u_st_kd.base = 0;
      u_st_kd.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_kd.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_kd.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_kd.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_kd = u_st_kd.real;
      offset += sizeof(this->st_kd);
        memcpy( &(this->kd[i]), &(this->st_kd), sizeof(float));
      }
     return offset;
    }

    virtual const char * getType() override { return "robotnik_msgs/MotorPID"; };
    virtual const char * getMD5() override { return "a4f1747645e7d598483fc2ed471a485d"; };

  };

}
#endif
