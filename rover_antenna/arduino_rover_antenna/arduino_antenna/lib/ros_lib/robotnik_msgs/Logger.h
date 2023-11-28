#ifndef _ROS_robotnik_msgs_Logger_h
#define _ROS_robotnik_msgs_Logger_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robotnik_msgs
{

  class Logger : public ros::Msg
  {
    public:
      typedef const char* _date_type;
      _date_type date;
      typedef const char* _component_type;
      _component_type component;
      typedef const char* _tag_type;
      _tag_type tag;
      typedef const char* _log_level_type;
      _log_level_type log_level;
      typedef const char* _description_type;
      _description_type description;
      enum { LOG_LEVEL_INFO = Info };
      enum { LOG_LEVEL_WARNING = Warning };
      enum { LOG_LEVEL_ERROR = Error };
      enum { LOG_LEVEL_DEBUG = Debug };
      enum { LOG_LEVEL_USER = User };

    Logger():
      date(""),
      component(""),
      tag(""),
      log_level(""),
      description("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_date = strlen(this->date);
      varToArr(outbuffer + offset, length_date);
      offset += 4;
      memcpy(outbuffer + offset, this->date, length_date);
      offset += length_date;
      uint32_t length_component = strlen(this->component);
      varToArr(outbuffer + offset, length_component);
      offset += 4;
      memcpy(outbuffer + offset, this->component, length_component);
      offset += length_component;
      uint32_t length_tag = strlen(this->tag);
      varToArr(outbuffer + offset, length_tag);
      offset += 4;
      memcpy(outbuffer + offset, this->tag, length_tag);
      offset += length_tag;
      uint32_t length_log_level = strlen(this->log_level);
      varToArr(outbuffer + offset, length_log_level);
      offset += 4;
      memcpy(outbuffer + offset, this->log_level, length_log_level);
      offset += length_log_level;
      uint32_t length_description = strlen(this->description);
      varToArr(outbuffer + offset, length_description);
      offset += 4;
      memcpy(outbuffer + offset, this->description, length_description);
      offset += length_description;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_date;
      arrToVar(length_date, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_date; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_date-1]=0;
      this->date = (char *)(inbuffer + offset-1);
      offset += length_date;
      uint32_t length_component;
      arrToVar(length_component, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_component; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_component-1]=0;
      this->component = (char *)(inbuffer + offset-1);
      offset += length_component;
      uint32_t length_tag;
      arrToVar(length_tag, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_tag; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_tag-1]=0;
      this->tag = (char *)(inbuffer + offset-1);
      offset += length_tag;
      uint32_t length_log_level;
      arrToVar(length_log_level, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_log_level; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_log_level-1]=0;
      this->log_level = (char *)(inbuffer + offset-1);
      offset += length_log_level;
      uint32_t length_description;
      arrToVar(length_description, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_description; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_description-1]=0;
      this->description = (char *)(inbuffer + offset-1);
      offset += length_description;
     return offset;
    }

    virtual const char * getType() override { return "robotnik_msgs/Logger"; };
    virtual const char * getMD5() override { return "897db59ca03596ba34a75f6e11ca536d"; };

  };

}
#endif
