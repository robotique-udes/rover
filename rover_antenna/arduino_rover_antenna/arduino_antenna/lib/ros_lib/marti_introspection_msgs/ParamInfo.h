#ifndef _ROS_marti_introspection_msgs_ParamInfo_h
#define _ROS_marti_introspection_msgs_ParamInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace marti_introspection_msgs
{

  class ParamInfo : public ros::Msg
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
      typedef uint8_t _type_type;
      _type_type type;
      typedef bool _dynamic_type;
      _dynamic_type dynamic;
      typedef int32_t _default_int_type;
      _default_int_type default_int;
      typedef float _default_float_type;
      _default_float_type default_float;
      typedef float _default_double_type;
      _default_double_type default_double;
      typedef const char* _default_string_type;
      _default_string_type default_string;
      typedef bool _default_bool_type;
      _default_bool_type default_bool;
      typedef float _max_value_type;
      _max_value_type max_value;
      typedef float _min_value_type;
      _min_value_type min_value;
      enum { TYPE_XMLRPC = 0 };
      enum { TYPE_DOUBLE = 1 };
      enum { TYPE_STRING = 2 };
      enum { TYPE_INT = 3 };
      enum { TYPE_FLOAT = 4 };
      enum { TYPE_BOOL = 5 };

    ParamInfo():
      name(""),
      resolved_name(""),
      description(""),
      group(""),
      type(0),
      dynamic(0),
      default_int(0),
      default_float(0),
      default_double(0),
      default_string(""),
      default_bool(0),
      max_value(0),
      min_value(0)
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
      *(outbuffer + offset + 0) = (this->type >> (8 * 0)) & 0xFF;
      offset += sizeof(this->type);
      union {
        bool real;
        uint8_t base;
      } u_dynamic;
      u_dynamic.real = this->dynamic;
      *(outbuffer + offset + 0) = (u_dynamic.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->dynamic);
      union {
        int32_t real;
        uint32_t base;
      } u_default_int;
      u_default_int.real = this->default_int;
      *(outbuffer + offset + 0) = (u_default_int.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_default_int.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_default_int.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_default_int.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->default_int);
      union {
        float real;
        uint32_t base;
      } u_default_float;
      u_default_float.real = this->default_float;
      *(outbuffer + offset + 0) = (u_default_float.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_default_float.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_default_float.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_default_float.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->default_float);
      offset += serializeAvrFloat64(outbuffer + offset, this->default_double);
      uint32_t length_default_string = strlen(this->default_string);
      varToArr(outbuffer + offset, length_default_string);
      offset += 4;
      memcpy(outbuffer + offset, this->default_string, length_default_string);
      offset += length_default_string;
      union {
        bool real;
        uint8_t base;
      } u_default_bool;
      u_default_bool.real = this->default_bool;
      *(outbuffer + offset + 0) = (u_default_bool.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->default_bool);
      offset += serializeAvrFloat64(outbuffer + offset, this->max_value);
      offset += serializeAvrFloat64(outbuffer + offset, this->min_value);
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
      this->type =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->type);
      union {
        bool real;
        uint8_t base;
      } u_dynamic;
      u_dynamic.base = 0;
      u_dynamic.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->dynamic = u_dynamic.real;
      offset += sizeof(this->dynamic);
      union {
        int32_t real;
        uint32_t base;
      } u_default_int;
      u_default_int.base = 0;
      u_default_int.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_default_int.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_default_int.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_default_int.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->default_int = u_default_int.real;
      offset += sizeof(this->default_int);
      union {
        float real;
        uint32_t base;
      } u_default_float;
      u_default_float.base = 0;
      u_default_float.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_default_float.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_default_float.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_default_float.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->default_float = u_default_float.real;
      offset += sizeof(this->default_float);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->default_double));
      uint32_t length_default_string;
      arrToVar(length_default_string, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_default_string; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_default_string-1]=0;
      this->default_string = (char *)(inbuffer + offset-1);
      offset += length_default_string;
      union {
        bool real;
        uint8_t base;
      } u_default_bool;
      u_default_bool.base = 0;
      u_default_bool.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->default_bool = u_default_bool.real;
      offset += sizeof(this->default_bool);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->max_value));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->min_value));
     return offset;
    }

    virtual const char * getType() override { return "marti_introspection_msgs/ParamInfo"; };
    virtual const char * getMD5() override { return "7fd9515e21aa85a018b2e533f4b39bf7"; };

  };

}
#endif
