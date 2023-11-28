#ifndef _ROS_robotnik_msgs_MotorStatus_h
#define _ROS_robotnik_msgs_MotorStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robotnik_msgs
{

  class MotorStatus : public ros::Msg
  {
    public:
      typedef uint8_t _can_id_type;
      _can_id_type can_id;
      typedef const char* _joint_type;
      _joint_type joint;
      typedef const char* _state_type;
      _state_type state;
      typedef const char* _status_type;
      _status_type status;
      typedef const char* _communicationstatus_type;
      _communicationstatus_type communicationstatus;
      typedef const char* _mode_of_operation_type;
      _mode_of_operation_type mode_of_operation;
      uint32_t activestatusword_length;
      typedef char* _activestatusword_type;
      _activestatusword_type st_activestatusword;
      _activestatusword_type * activestatusword;
      uint32_t activedriveflags_length;
      typedef char* _activedriveflags_type;
      _activedriveflags_type st_activedriveflags;
      _activedriveflags_type * activedriveflags;
      uint32_t digitaloutputs_length;
      typedef bool _digitaloutputs_type;
      _digitaloutputs_type st_digitaloutputs;
      _digitaloutputs_type * digitaloutputs;
      uint32_t digitalinputs_length;
      typedef bool _digitalinputs_type;
      _digitalinputs_type st_digitalinputs;
      _digitalinputs_type * digitalinputs;
      typedef float _current_type;
      _current_type current;
      uint32_t analoginputs_length;
      typedef float _analoginputs_type;
      _analoginputs_type st_analoginputs;
      _analoginputs_type * analoginputs;
      typedef const char* _statusword_type;
      _statusword_type statusword;
      typedef const char* _driveflags_type;
      _driveflags_type driveflags;

    MotorStatus():
      can_id(0),
      joint(""),
      state(""),
      status(""),
      communicationstatus(""),
      mode_of_operation(""),
      activestatusword_length(0), st_activestatusword(), activestatusword(nullptr),
      activedriveflags_length(0), st_activedriveflags(), activedriveflags(nullptr),
      digitaloutputs_length(0), st_digitaloutputs(), digitaloutputs(nullptr),
      digitalinputs_length(0), st_digitalinputs(), digitalinputs(nullptr),
      current(0),
      analoginputs_length(0), st_analoginputs(), analoginputs(nullptr),
      statusword(""),
      driveflags("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->can_id >> (8 * 0)) & 0xFF;
      offset += sizeof(this->can_id);
      uint32_t length_joint = strlen(this->joint);
      varToArr(outbuffer + offset, length_joint);
      offset += 4;
      memcpy(outbuffer + offset, this->joint, length_joint);
      offset += length_joint;
      uint32_t length_state = strlen(this->state);
      varToArr(outbuffer + offset, length_state);
      offset += 4;
      memcpy(outbuffer + offset, this->state, length_state);
      offset += length_state;
      uint32_t length_status = strlen(this->status);
      varToArr(outbuffer + offset, length_status);
      offset += 4;
      memcpy(outbuffer + offset, this->status, length_status);
      offset += length_status;
      uint32_t length_communicationstatus = strlen(this->communicationstatus);
      varToArr(outbuffer + offset, length_communicationstatus);
      offset += 4;
      memcpy(outbuffer + offset, this->communicationstatus, length_communicationstatus);
      offset += length_communicationstatus;
      uint32_t length_mode_of_operation = strlen(this->mode_of_operation);
      varToArr(outbuffer + offset, length_mode_of_operation);
      offset += 4;
      memcpy(outbuffer + offset, this->mode_of_operation, length_mode_of_operation);
      offset += length_mode_of_operation;
      *(outbuffer + offset + 0) = (this->activestatusword_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->activestatusword_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->activestatusword_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->activestatusword_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->activestatusword_length);
      for( uint32_t i = 0; i < activestatusword_length; i++){
      uint32_t length_activestatuswordi = strlen(this->activestatusword[i]);
      varToArr(outbuffer + offset, length_activestatuswordi);
      offset += 4;
      memcpy(outbuffer + offset, this->activestatusword[i], length_activestatuswordi);
      offset += length_activestatuswordi;
      }
      *(outbuffer + offset + 0) = (this->activedriveflags_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->activedriveflags_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->activedriveflags_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->activedriveflags_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->activedriveflags_length);
      for( uint32_t i = 0; i < activedriveflags_length; i++){
      uint32_t length_activedriveflagsi = strlen(this->activedriveflags[i]);
      varToArr(outbuffer + offset, length_activedriveflagsi);
      offset += 4;
      memcpy(outbuffer + offset, this->activedriveflags[i], length_activedriveflagsi);
      offset += length_activedriveflagsi;
      }
      *(outbuffer + offset + 0) = (this->digitaloutputs_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->digitaloutputs_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->digitaloutputs_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->digitaloutputs_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->digitaloutputs_length);
      for( uint32_t i = 0; i < digitaloutputs_length; i++){
      union {
        bool real;
        uint8_t base;
      } u_digitaloutputsi;
      u_digitaloutputsi.real = this->digitaloutputs[i];
      *(outbuffer + offset + 0) = (u_digitaloutputsi.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->digitaloutputs[i]);
      }
      *(outbuffer + offset + 0) = (this->digitalinputs_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->digitalinputs_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->digitalinputs_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->digitalinputs_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->digitalinputs_length);
      for( uint32_t i = 0; i < digitalinputs_length; i++){
      union {
        bool real;
        uint8_t base;
      } u_digitalinputsi;
      u_digitalinputsi.real = this->digitalinputs[i];
      *(outbuffer + offset + 0) = (u_digitalinputsi.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->digitalinputs[i]);
      }
      union {
        float real;
        uint32_t base;
      } u_current;
      u_current.real = this->current;
      *(outbuffer + offset + 0) = (u_current.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_current.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_current.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_current.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current);
      *(outbuffer + offset + 0) = (this->analoginputs_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->analoginputs_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->analoginputs_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->analoginputs_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->analoginputs_length);
      for( uint32_t i = 0; i < analoginputs_length; i++){
      union {
        float real;
        uint32_t base;
      } u_analoginputsi;
      u_analoginputsi.real = this->analoginputs[i];
      *(outbuffer + offset + 0) = (u_analoginputsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_analoginputsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_analoginputsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_analoginputsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->analoginputs[i]);
      }
      uint32_t length_statusword = strlen(this->statusword);
      varToArr(outbuffer + offset, length_statusword);
      offset += 4;
      memcpy(outbuffer + offset, this->statusword, length_statusword);
      offset += length_statusword;
      uint32_t length_driveflags = strlen(this->driveflags);
      varToArr(outbuffer + offset, length_driveflags);
      offset += 4;
      memcpy(outbuffer + offset, this->driveflags, length_driveflags);
      offset += length_driveflags;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->can_id =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->can_id);
      uint32_t length_joint;
      arrToVar(length_joint, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_joint; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_joint-1]=0;
      this->joint = (char *)(inbuffer + offset-1);
      offset += length_joint;
      uint32_t length_state;
      arrToVar(length_state, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_state; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_state-1]=0;
      this->state = (char *)(inbuffer + offset-1);
      offset += length_state;
      uint32_t length_status;
      arrToVar(length_status, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_status; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_status-1]=0;
      this->status = (char *)(inbuffer + offset-1);
      offset += length_status;
      uint32_t length_communicationstatus;
      arrToVar(length_communicationstatus, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_communicationstatus; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_communicationstatus-1]=0;
      this->communicationstatus = (char *)(inbuffer + offset-1);
      offset += length_communicationstatus;
      uint32_t length_mode_of_operation;
      arrToVar(length_mode_of_operation, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_mode_of_operation; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_mode_of_operation-1]=0;
      this->mode_of_operation = (char *)(inbuffer + offset-1);
      offset += length_mode_of_operation;
      uint32_t activestatusword_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      activestatusword_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      activestatusword_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      activestatusword_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->activestatusword_length);
      if(activestatusword_lengthT > activestatusword_length)
        this->activestatusword = (char**)realloc(this->activestatusword, activestatusword_lengthT * sizeof(char*));
      activestatusword_length = activestatusword_lengthT;
      for( uint32_t i = 0; i < activestatusword_length; i++){
      uint32_t length_st_activestatusword;
      arrToVar(length_st_activestatusword, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_activestatusword; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_activestatusword-1]=0;
      this->st_activestatusword = (char *)(inbuffer + offset-1);
      offset += length_st_activestatusword;
        memcpy( &(this->activestatusword[i]), &(this->st_activestatusword), sizeof(char*));
      }
      uint32_t activedriveflags_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      activedriveflags_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      activedriveflags_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      activedriveflags_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->activedriveflags_length);
      if(activedriveflags_lengthT > activedriveflags_length)
        this->activedriveflags = (char**)realloc(this->activedriveflags, activedriveflags_lengthT * sizeof(char*));
      activedriveflags_length = activedriveflags_lengthT;
      for( uint32_t i = 0; i < activedriveflags_length; i++){
      uint32_t length_st_activedriveflags;
      arrToVar(length_st_activedriveflags, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_activedriveflags; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_activedriveflags-1]=0;
      this->st_activedriveflags = (char *)(inbuffer + offset-1);
      offset += length_st_activedriveflags;
        memcpy( &(this->activedriveflags[i]), &(this->st_activedriveflags), sizeof(char*));
      }
      uint32_t digitaloutputs_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      digitaloutputs_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      digitaloutputs_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      digitaloutputs_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->digitaloutputs_length);
      if(digitaloutputs_lengthT > digitaloutputs_length)
        this->digitaloutputs = (bool*)realloc(this->digitaloutputs, digitaloutputs_lengthT * sizeof(bool));
      digitaloutputs_length = digitaloutputs_lengthT;
      for( uint32_t i = 0; i < digitaloutputs_length; i++){
      union {
        bool real;
        uint8_t base;
      } u_st_digitaloutputs;
      u_st_digitaloutputs.base = 0;
      u_st_digitaloutputs.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->st_digitaloutputs = u_st_digitaloutputs.real;
      offset += sizeof(this->st_digitaloutputs);
        memcpy( &(this->digitaloutputs[i]), &(this->st_digitaloutputs), sizeof(bool));
      }
      uint32_t digitalinputs_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      digitalinputs_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      digitalinputs_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      digitalinputs_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->digitalinputs_length);
      if(digitalinputs_lengthT > digitalinputs_length)
        this->digitalinputs = (bool*)realloc(this->digitalinputs, digitalinputs_lengthT * sizeof(bool));
      digitalinputs_length = digitalinputs_lengthT;
      for( uint32_t i = 0; i < digitalinputs_length; i++){
      union {
        bool real;
        uint8_t base;
      } u_st_digitalinputs;
      u_st_digitalinputs.base = 0;
      u_st_digitalinputs.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->st_digitalinputs = u_st_digitalinputs.real;
      offset += sizeof(this->st_digitalinputs);
        memcpy( &(this->digitalinputs[i]), &(this->st_digitalinputs), sizeof(bool));
      }
      union {
        float real;
        uint32_t base;
      } u_current;
      u_current.base = 0;
      u_current.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_current.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_current.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_current.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->current = u_current.real;
      offset += sizeof(this->current);
      uint32_t analoginputs_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      analoginputs_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      analoginputs_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      analoginputs_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->analoginputs_length);
      if(analoginputs_lengthT > analoginputs_length)
        this->analoginputs = (float*)realloc(this->analoginputs, analoginputs_lengthT * sizeof(float));
      analoginputs_length = analoginputs_lengthT;
      for( uint32_t i = 0; i < analoginputs_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_analoginputs;
      u_st_analoginputs.base = 0;
      u_st_analoginputs.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_analoginputs.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_analoginputs.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_analoginputs.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_analoginputs = u_st_analoginputs.real;
      offset += sizeof(this->st_analoginputs);
        memcpy( &(this->analoginputs[i]), &(this->st_analoginputs), sizeof(float));
      }
      uint32_t length_statusword;
      arrToVar(length_statusword, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_statusword; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_statusword-1]=0;
      this->statusword = (char *)(inbuffer + offset-1);
      offset += length_statusword;
      uint32_t length_driveflags;
      arrToVar(length_driveflags, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_driveflags; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_driveflags-1]=0;
      this->driveflags = (char *)(inbuffer + offset-1);
      offset += length_driveflags;
     return offset;
    }

    virtual const char * getType() override { return "robotnik_msgs/MotorStatus"; };
    virtual const char * getMD5() override { return "f36d1740e4c2f57c8e2136e05837a2ca"; };

  };

}
#endif
