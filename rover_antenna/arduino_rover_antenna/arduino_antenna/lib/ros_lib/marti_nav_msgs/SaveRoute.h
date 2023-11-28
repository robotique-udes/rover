#ifndef _ROS_SERVICE_SaveRoute_h
#define _ROS_SERVICE_SaveRoute_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "marti_nav_msgs/Route.h"

namespace marti_nav_msgs
{

static const char SAVEROUTE[] = "marti_nav_msgs/SaveRoute";

  class SaveRouteRequest : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      typedef const char* _guid_type;
      _guid_type guid;
      typedef marti_nav_msgs::Route _route_type;
      _route_type route;
      typedef const char* _thumbnail_type;
      _thumbnail_type thumbnail;

    SaveRouteRequest():
      name(""),
      guid(""),
      route(),
      thumbnail("")
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
      uint32_t length_guid = strlen(this->guid);
      varToArr(outbuffer + offset, length_guid);
      offset += 4;
      memcpy(outbuffer + offset, this->guid, length_guid);
      offset += length_guid;
      offset += this->route.serialize(outbuffer + offset);
      uint32_t length_thumbnail = strlen(this->thumbnail);
      varToArr(outbuffer + offset, length_thumbnail);
      offset += 4;
      memcpy(outbuffer + offset, this->thumbnail, length_thumbnail);
      offset += length_thumbnail;
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
      uint32_t length_guid;
      arrToVar(length_guid, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_guid; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_guid-1]=0;
      this->guid = (char *)(inbuffer + offset-1);
      offset += length_guid;
      offset += this->route.deserialize(inbuffer + offset);
      uint32_t length_thumbnail;
      arrToVar(length_thumbnail, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_thumbnail; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_thumbnail-1]=0;
      this->thumbnail = (char *)(inbuffer + offset-1);
      offset += length_thumbnail;
     return offset;
    }

    virtual const char * getType() override { return SAVEROUTE; };
    virtual const char * getMD5() override { return "d96b0f40cb18a4c24fe9c24bf524e777"; };

  };

  class SaveRouteResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;
      typedef const char* _message_type;
      _message_type message;

    SaveRouteResponse():
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

    virtual const char * getType() override { return SAVEROUTE; };
    virtual const char * getMD5() override { return "937c9679a518e3a18d831e57125ea522"; };

  };

  class SaveRoute {
    public:
    typedef SaveRouteRequest Request;
    typedef SaveRouteResponse Response;
  };

}
#endif
