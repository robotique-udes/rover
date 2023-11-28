#ifndef _ROS_SERVICE_Record_h
#define _ROS_SERVICE_Record_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robotnik_msgs
{

static const char RECORD[] = "robotnik_msgs/Record";

  class RecordRequest : public ros::Msg
  {
    public:
      typedef const char* _action_type;
      _action_type action;
      typedef const char* _file_name_type;
      _file_name_type file_name;
      typedef int32_t _max_time_type;
      _max_time_type max_time;
      enum { ACTION_RECORD = RECORD };
      enum { ACTION_STOP = STOP };
      enum { ACTION_SAVE = SAVE };

    RecordRequest():
      action(""),
      file_name(""),
      max_time(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_action = strlen(this->action);
      varToArr(outbuffer + offset, length_action);
      offset += 4;
      memcpy(outbuffer + offset, this->action, length_action);
      offset += length_action;
      uint32_t length_file_name = strlen(this->file_name);
      varToArr(outbuffer + offset, length_file_name);
      offset += 4;
      memcpy(outbuffer + offset, this->file_name, length_file_name);
      offset += length_file_name;
      union {
        int32_t real;
        uint32_t base;
      } u_max_time;
      u_max_time.real = this->max_time;
      *(outbuffer + offset + 0) = (u_max_time.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_time.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_time.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_time.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max_time);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_action;
      arrToVar(length_action, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_action; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_action-1]=0;
      this->action = (char *)(inbuffer + offset-1);
      offset += length_action;
      uint32_t length_file_name;
      arrToVar(length_file_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_file_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_file_name-1]=0;
      this->file_name = (char *)(inbuffer + offset-1);
      offset += length_file_name;
      union {
        int32_t real;
        uint32_t base;
      } u_max_time;
      u_max_time.base = 0;
      u_max_time.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_time.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_time.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_time.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->max_time = u_max_time.real;
      offset += sizeof(this->max_time);
     return offset;
    }

    virtual const char * getType() override { return RECORD; };
    virtual const char * getMD5() override { return "dbfe3d7a69656c931298b4f3f1693d9b"; };

  };

  class RecordResponse : public ros::Msg
  {
    public:
      typedef const char* _message_type;
      _message_type message;
      typedef bool _success_type;
      _success_type success;

    RecordResponse():
      message(""),
      success(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_message = strlen(this->message);
      varToArr(outbuffer + offset, length_message);
      offset += 4;
      memcpy(outbuffer + offset, this->message, length_message);
      offset += length_message;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_message;
      arrToVar(length_message, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_message; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_message-1]=0;
      this->message = (char *)(inbuffer + offset-1);
      offset += length_message;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
     return offset;
    }

    virtual const char * getType() override { return RECORD; };
    virtual const char * getMD5() override { return "9bf829f07d795d3f9e541a07897da2c4"; };

  };

  class Record {
    public:
    typedef RecordRequest Request;
    typedef RecordResponse Response;
  };

}
#endif
