#ifndef _ROS_SERVICE_AddMapvizDisplay_h
#define _ROS_SERVICE_AddMapvizDisplay_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "marti_common_msgs/KeyValue.h"

namespace mapviz
{

static const char ADDMAPVIZDISPLAY[] = "mapviz/AddMapvizDisplay";

  class AddMapvizDisplayRequest : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      typedef const char* _type_type;
      _type_type type;
      typedef int32_t _draw_order_type;
      _draw_order_type draw_order;
      typedef bool _visible_type;
      _visible_type visible;
      uint32_t properties_length;
      typedef marti_common_msgs::KeyValue _properties_type;
      _properties_type st_properties;
      _properties_type * properties;

    AddMapvizDisplayRequest():
      name(""),
      type(""),
      draw_order(0),
      visible(0),
      properties_length(0), st_properties(), properties(nullptr)
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
      uint32_t length_type = strlen(this->type);
      varToArr(outbuffer + offset, length_type);
      offset += 4;
      memcpy(outbuffer + offset, this->type, length_type);
      offset += length_type;
      union {
        int32_t real;
        uint32_t base;
      } u_draw_order;
      u_draw_order.real = this->draw_order;
      *(outbuffer + offset + 0) = (u_draw_order.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_draw_order.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_draw_order.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_draw_order.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->draw_order);
      union {
        bool real;
        uint8_t base;
      } u_visible;
      u_visible.real = this->visible;
      *(outbuffer + offset + 0) = (u_visible.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->visible);
      *(outbuffer + offset + 0) = (this->properties_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->properties_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->properties_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->properties_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->properties_length);
      for( uint32_t i = 0; i < properties_length; i++){
      offset += this->properties[i].serialize(outbuffer + offset);
      }
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
      uint32_t length_type;
      arrToVar(length_type, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_type; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_type-1]=0;
      this->type = (char *)(inbuffer + offset-1);
      offset += length_type;
      union {
        int32_t real;
        uint32_t base;
      } u_draw_order;
      u_draw_order.base = 0;
      u_draw_order.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_draw_order.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_draw_order.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_draw_order.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->draw_order = u_draw_order.real;
      offset += sizeof(this->draw_order);
      union {
        bool real;
        uint8_t base;
      } u_visible;
      u_visible.base = 0;
      u_visible.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->visible = u_visible.real;
      offset += sizeof(this->visible);
      uint32_t properties_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      properties_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      properties_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      properties_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->properties_length);
      if(properties_lengthT > properties_length)
        this->properties = (marti_common_msgs::KeyValue*)realloc(this->properties, properties_lengthT * sizeof(marti_common_msgs::KeyValue));
      properties_length = properties_lengthT;
      for( uint32_t i = 0; i < properties_length; i++){
      offset += this->st_properties.deserialize(inbuffer + offset);
        memcpy( &(this->properties[i]), &(this->st_properties), sizeof(marti_common_msgs::KeyValue));
      }
     return offset;
    }

    virtual const char * getType() override { return ADDMAPVIZDISPLAY; };
    virtual const char * getMD5() override { return "d99db34575927545707e7081858716f3"; };

  };

  class AddMapvizDisplayResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;
      typedef const char* _message_type;
      _message_type message;

    AddMapvizDisplayResponse():
      success(0),
      message("")
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

    virtual const char * getType() override { return ADDMAPVIZDISPLAY; };
    virtual const char * getMD5() override { return "937c9679a518e3a18d831e57125ea522"; };

  };

  class AddMapvizDisplay {
    public:
    typedef AddMapvizDisplayRequest Request;
    typedef AddMapvizDisplayResponse Response;
  };

}
#endif
