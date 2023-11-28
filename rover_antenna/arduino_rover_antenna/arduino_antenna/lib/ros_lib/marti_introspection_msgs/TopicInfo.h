#ifndef _ROS_marti_introspection_msgs_TopicInfo_h
#define _ROS_marti_introspection_msgs_TopicInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace marti_introspection_msgs
{

  class TopicInfo : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      typedef const char* _resolved_name_type;
      _resolved_name_type resolved_name;
      typedef const char* _description_type;
      _description_type description;
      typedef const char* _group_type;
      _group_type group;
      typedef const char* _message_type_type;
      _message_type_type message_type;
      typedef bool _advertised_type;
      _advertised_type advertised;

    TopicInfo():
      name(""),
      resolved_name(""),
      description(""),
      group(""),
      message_type(""),
      advertised(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      uint32_t length_resolved_name = strlen(this->resolved_name);
      varToArr(outbuffer + offset, length_resolved_name);
      offset += 4;
      memcpy(outbuffer + offset, this->resolved_name, length_resolved_name);
      offset += length_resolved_name;
      uint32_t length_description = strlen(this->description);
      varToArr(outbuffer + offset, length_description);
      offset += 4;
      memcpy(outbuffer + offset, this->description, length_description);
      offset += length_description;
      uint32_t length_group = strlen(this->group);
      varToArr(outbuffer + offset, length_group);
      offset += 4;
      memcpy(outbuffer + offset, this->group, length_group);
      offset += length_group;
      uint32_t length_message_type = strlen(this->message_type);
      varToArr(outbuffer + offset, length_message_type);
      offset += 4;
      memcpy(outbuffer + offset, this->message_type, length_message_type);
      offset += length_message_type;
      union {
        bool real;
        uint8_t base;
      } u_advertised;
      u_advertised.real = this->advertised;
      *(outbuffer + offset + 0) = (u_advertised.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->advertised);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      uint32_t length_resolved_name;
      arrToVar(length_resolved_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_resolved_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_resolved_name-1]=0;
      this->resolved_name = (char *)(inbuffer + offset-1);
      offset += length_resolved_name;
      uint32_t length_description;
      arrToVar(length_description, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_description; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_description-1]=0;
      this->description = (char *)(inbuffer + offset-1);
      offset += length_description;
      uint32_t length_group;
      arrToVar(length_group, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_group; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_group-1]=0;
      this->group = (char *)(inbuffer + offset-1);
      offset += length_group;
      uint32_t length_message_type;
      arrToVar(length_message_type, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_message_type; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_message_type-1]=0;
      this->message_type = (char *)(inbuffer + offset-1);
      offset += length_message_type;
      union {
        bool real;
        uint8_t base;
      } u_advertised;
      u_advertised.base = 0;
      u_advertised.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->advertised = u_advertised.real;
      offset += sizeof(this->advertised);
     return offset;
    }

    virtual const char * getType() override { return "marti_introspection_msgs/TopicInfo"; };
    virtual const char * getMD5() override { return "d5e63e499f2b773abff6025e21d0eb12"; };

  };

}
#endif
