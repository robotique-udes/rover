#ifndef _ROS_SERVICE_InsertTask_h
#define _ROS_SERVICE_InsertTask_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robotnik_msgs
{

static const char INSERTTASK[] = "robotnik_msgs/InsertTask";

  class InsertTaskRequest : public ros::Msg
  {
    public:
      typedef int32_t _id_submission_type;
      _id_submission_type id_submission;
      typedef const char* _description_task_type;
      _description_task_type description_task;
      typedef const char* _datatime_start_type;
      _datatime_start_type datatime_start;

    InsertTaskRequest():
      id_submission(0),
      description_task(""),
      datatime_start("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_id_submission;
      u_id_submission.real = this->id_submission;
      *(outbuffer + offset + 0) = (u_id_submission.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_id_submission.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_id_submission.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_id_submission.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->id_submission);
      uint32_t length_description_task = strlen(this->description_task);
      varToArr(outbuffer + offset, length_description_task);
      offset += 4;
      memcpy(outbuffer + offset, this->description_task, length_description_task);
      offset += length_description_task;
      uint32_t length_datatime_start = strlen(this->datatime_start);
      varToArr(outbuffer + offset, length_datatime_start);
      offset += 4;
      memcpy(outbuffer + offset, this->datatime_start, length_datatime_start);
      offset += length_datatime_start;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_id_submission;
      u_id_submission.base = 0;
      u_id_submission.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_id_submission.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_id_submission.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_id_submission.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->id_submission = u_id_submission.real;
      offset += sizeof(this->id_submission);
      uint32_t length_description_task;
      arrToVar(length_description_task, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_description_task; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_description_task-1]=0;
      this->description_task = (char *)(inbuffer + offset-1);
      offset += length_description_task;
      uint32_t length_datatime_start;
      arrToVar(length_datatime_start, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_datatime_start; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_datatime_start-1]=0;
      this->datatime_start = (char *)(inbuffer + offset-1);
      offset += length_datatime_start;
     return offset;
    }

    virtual const char * getType() override { return INSERTTASK; };
    virtual const char * getMD5() override { return "415fc1cb1de92194825450f4e7e89346"; };

  };

  class InsertTaskResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;
      typedef const char* _msg_type;
      _msg_type msg;

    InsertTaskResponse():
      success(0),
      msg("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      uint32_t length_msg = strlen(this->msg);
      varToArr(outbuffer + offset, length_msg);
      offset += 4;
      memcpy(outbuffer + offset, this->msg, length_msg);
      offset += length_msg;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
      uint32_t length_msg;
      arrToVar(length_msg, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_msg; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_msg-1]=0;
      this->msg = (char *)(inbuffer + offset-1);
      offset += length_msg;
     return offset;
    }

    virtual const char * getType() override { return INSERTTASK; };
    virtual const char * getMD5() override { return "e835b04f93d0b30fd8cb71e0967a16db"; };

  };

  class InsertTask {
    public:
    typedef InsertTaskRequest Request;
    typedef InsertTaskResponse Response;
  };

}
#endif
