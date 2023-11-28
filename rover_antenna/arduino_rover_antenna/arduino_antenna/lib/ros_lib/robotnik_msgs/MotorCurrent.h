#ifndef _ROS_robotnik_msgs_MotorCurrent_h
#define _ROS_robotnik_msgs_MotorCurrent_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robotnik_msgs
{

  class MotorCurrent : public ros::Msg
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
      uint32_t continuous_current_limit_length;
      typedef float _continuous_current_limit_type;
      _continuous_current_limit_type st_continuous_current_limit;
      _continuous_current_limit_type * continuous_current_limit;
      uint32_t current_peak_time_length;
      typedef float _current_peak_time_type;
      _current_peak_time_type st_current_peak_time;
      _current_peak_time_type * current_peak_time;
      uint32_t current_peak_limit_length;
      typedef float _current_peak_limit_type;
      _current_peak_limit_type st_current_peak_limit;
      _current_peak_limit_type * current_peak_limit;

    MotorCurrent():
      can_id_length(0), st_can_id(), can_id(nullptr),
      name_length(0), st_name(), name(nullptr),
      continuous_current_limit_length(0), st_continuous_current_limit(), continuous_current_limit(nullptr),
      current_peak_time_length(0), st_current_peak_time(), current_peak_time(nullptr),
      current_peak_limit_length(0), st_current_peak_limit(), current_peak_limit(nullptr)
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
      *(outbuffer + offset + 0) = (this->continuous_current_limit_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->continuous_current_limit_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->continuous_current_limit_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->continuous_current_limit_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->continuous_current_limit_length);
      for( uint32_t i = 0; i < continuous_current_limit_length; i++){
      union {
        float real;
        uint32_t base;
      } u_continuous_current_limiti;
      u_continuous_current_limiti.real = this->continuous_current_limit[i];
      *(outbuffer + offset + 0) = (u_continuous_current_limiti.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_continuous_current_limiti.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_continuous_current_limiti.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_continuous_current_limiti.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->continuous_current_limit[i]);
      }
      *(outbuffer + offset + 0) = (this->current_peak_time_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->current_peak_time_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->current_peak_time_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->current_peak_time_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current_peak_time_length);
      for( uint32_t i = 0; i < current_peak_time_length; i++){
      union {
        float real;
        uint32_t base;
      } u_current_peak_timei;
      u_current_peak_timei.real = this->current_peak_time[i];
      *(outbuffer + offset + 0) = (u_current_peak_timei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_current_peak_timei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_current_peak_timei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_current_peak_timei.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current_peak_time[i]);
      }
      *(outbuffer + offset + 0) = (this->current_peak_limit_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->current_peak_limit_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->current_peak_limit_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->current_peak_limit_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current_peak_limit_length);
      for( uint32_t i = 0; i < current_peak_limit_length; i++){
      union {
        float real;
        uint32_t base;
      } u_current_peak_limiti;
      u_current_peak_limiti.real = this->current_peak_limit[i];
      *(outbuffer + offset + 0) = (u_current_peak_limiti.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_current_peak_limiti.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_current_peak_limiti.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_current_peak_limiti.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current_peak_limit[i]);
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
      uint32_t continuous_current_limit_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      continuous_current_limit_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      continuous_current_limit_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      continuous_current_limit_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->continuous_current_limit_length);
      if(continuous_current_limit_lengthT > continuous_current_limit_length)
        this->continuous_current_limit = (float*)realloc(this->continuous_current_limit, continuous_current_limit_lengthT * sizeof(float));
      continuous_current_limit_length = continuous_current_limit_lengthT;
      for( uint32_t i = 0; i < continuous_current_limit_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_continuous_current_limit;
      u_st_continuous_current_limit.base = 0;
      u_st_continuous_current_limit.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_continuous_current_limit.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_continuous_current_limit.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_continuous_current_limit.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_continuous_current_limit = u_st_continuous_current_limit.real;
      offset += sizeof(this->st_continuous_current_limit);
        memcpy( &(this->continuous_current_limit[i]), &(this->st_continuous_current_limit), sizeof(float));
      }
      uint32_t current_peak_time_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      current_peak_time_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      current_peak_time_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      current_peak_time_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->current_peak_time_length);
      if(current_peak_time_lengthT > current_peak_time_length)
        this->current_peak_time = (float*)realloc(this->current_peak_time, current_peak_time_lengthT * sizeof(float));
      current_peak_time_length = current_peak_time_lengthT;
      for( uint32_t i = 0; i < current_peak_time_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_current_peak_time;
      u_st_current_peak_time.base = 0;
      u_st_current_peak_time.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_current_peak_time.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_current_peak_time.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_current_peak_time.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_current_peak_time = u_st_current_peak_time.real;
      offset += sizeof(this->st_current_peak_time);
        memcpy( &(this->current_peak_time[i]), &(this->st_current_peak_time), sizeof(float));
      }
      uint32_t current_peak_limit_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      current_peak_limit_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      current_peak_limit_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      current_peak_limit_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->current_peak_limit_length);
      if(current_peak_limit_lengthT > current_peak_limit_length)
        this->current_peak_limit = (float*)realloc(this->current_peak_limit, current_peak_limit_lengthT * sizeof(float));
      current_peak_limit_length = current_peak_limit_lengthT;
      for( uint32_t i = 0; i < current_peak_limit_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_current_peak_limit;
      u_st_current_peak_limit.base = 0;
      u_st_current_peak_limit.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_current_peak_limit.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_current_peak_limit.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_current_peak_limit.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_current_peak_limit = u_st_current_peak_limit.real;
      offset += sizeof(this->st_current_peak_limit);
        memcpy( &(this->current_peak_limit[i]), &(this->st_current_peak_limit), sizeof(float));
      }
     return offset;
    }

    virtual const char * getType() override { return "robotnik_msgs/MotorCurrent"; };
    virtual const char * getMD5() override { return "683e2e0ea0c1b322bc7f35b2f02088ee"; };

  };

}
#endif
