#ifndef _ROS_SERVICE_LoggerQuery_h
#define _ROS_SERVICE_LoggerQuery_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "robotnik_msgs/Logger.h"

namespace robotnik_msgs
{

static const char LOGGERQUERY[] = "robotnik_msgs/LoggerQuery";

  class LoggerQueryRequest : public ros::Msg
  {
    public:
      typedef robotnik_msgs::Logger _query_type;
      _query_type query;
      typedef int32_t _max_records_type;
      _max_records_type max_records;

    LoggerQueryRequest():
      query(),
      max_records(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->query.serialize(outbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_max_records;
      u_max_records.real = this->max_records;
      *(outbuffer + offset + 0) = (u_max_records.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_records.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_records.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_records.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max_records);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->query.deserialize(inbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_max_records;
      u_max_records.base = 0;
      u_max_records.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_records.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_records.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_records.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->max_records = u_max_records.real;
      offset += sizeof(this->max_records);
     return offset;
    }

    virtual const char * getType() override { return LOGGERQUERY; };
    virtual const char * getMD5() override { return "7c8458e07e92fe2e02cc6baec5d16e1c"; };

  };

  class LoggerQueryResponse : public ros::Msg
  {
    public:
      uint32_t result_length;
      typedef robotnik_msgs::Logger _result_type;
      _result_type st_result;
      _result_type * result;
      typedef bool _success_type;
      _success_type success;
      typedef const char* _message_type;
      _message_type message;

    LoggerQueryResponse():
      result_length(0), st_result(), result(nullptr),
      success(0),
      message("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->result_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->result_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->result_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->result_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->result_length);
      for( uint32_t i = 0; i < result_length; i++){
      offset += this->result[i].serialize(outbuffer + offset);
      }
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      uint32_t length_message = strlen(this->message);
      varToArr(outbuffer + offset, length_message);
      offset += 4;
      memcpy(outbuffer + offset, this->message, length_message);
      offset += length_message;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t result_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      result_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      result_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      result_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->result_length);
      if(result_lengthT > result_length)
        this->result = (robotnik_msgs::Logger*)realloc(this->result, result_lengthT * sizeof(robotnik_msgs::Logger));
      result_length = result_lengthT;
      for( uint32_t i = 0; i < result_length; i++){
      offset += this->st_result.deserialize(inbuffer + offset);
        memcpy( &(this->result[i]), &(this->st_result), sizeof(robotnik_msgs::Logger));
      }
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
      uint32_t length_message;
      arrToVar(length_message, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_message; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_message-1]=0;
      this->message = (char *)(inbuffer + offset-1);
      offset += length_message;
     return offset;
    }

    virtual const char * getType() override { return LOGGERQUERY; };
    virtual const char * getMD5() override { return "b3c85f9c5c8561a9d4106c28be4c697d"; };

  };

  class LoggerQuery {
    public:
    typedef LoggerQueryRequest Request;
    typedef LoggerQueryResponse Response;
  };

}
#endif
