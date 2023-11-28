#ifndef _ROS_SERVICE_GetRouteList_h
#define _ROS_SERVICE_GetRouteList_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "marti_nav_msgs/Route.h"

namespace marti_nav_msgs
{

static const char GETROUTELIST[] = "marti_nav_msgs/GetRouteList";

  class GetRouteListRequest : public ros::Msg
  {
    public:

    GetRouteListRequest()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
     return offset;
    }

    virtual const char * getType() override { return GETROUTELIST; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetRouteListResponse : public ros::Msg
  {
    public:
      uint32_t routes_length;
      typedef marti_nav_msgs::Route _routes_type;
      _routes_type st_routes;
      _routes_type * routes;
      typedef bool _success_type;
      _success_type success;
      typedef const char* _message_type;
      _message_type message;

    GetRouteListResponse():
      routes_length(0), st_routes(), routes(nullptr),
      success(0),
      message("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->routes_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->routes_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->routes_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->routes_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->routes_length);
      for( uint32_t i = 0; i < routes_length; i++){
      offset += this->routes[i].serialize(outbuffer + offset);
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
      uint32_t routes_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      routes_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      routes_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      routes_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->routes_length);
      if(routes_lengthT > routes_length)
        this->routes = (marti_nav_msgs::Route*)realloc(this->routes, routes_lengthT * sizeof(marti_nav_msgs::Route));
      routes_length = routes_lengthT;
      for( uint32_t i = 0; i < routes_length; i++){
      offset += this->st_routes.deserialize(inbuffer + offset);
        memcpy( &(this->routes[i]), &(this->st_routes), sizeof(marti_nav_msgs::Route));
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

    virtual const char * getType() override { return GETROUTELIST; };
    virtual const char * getMD5() override { return "24b443520442ddc540d0fd59f35403a5"; };

  };

  class GetRouteList {
    public:
    typedef GetRouteListRequest Request;
    typedef GetRouteListResponse Response;
  };

}
#endif
