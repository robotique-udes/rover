#ifndef _ROS_marti_nav_msgs_RouteArray_h
#define _ROS_marti_nav_msgs_RouteArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "marti_nav_msgs/Route.h"
#include "marti_common_msgs/KeyValue.h"

namespace marti_nav_msgs
{

  class RouteArray : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t routes_length;
      typedef marti_nav_msgs::Route _routes_type;
      _routes_type st_routes;
      _routes_type * routes;
      uint32_t properties_length;
      typedef marti_common_msgs::KeyValue _properties_type;
      _properties_type st_properties;
      _properties_type * properties;

    RouteArray():
      header(),
      routes_length(0), st_routes(), routes(nullptr),
      properties_length(0), st_properties(), properties(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->routes_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->routes_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->routes_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->routes_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->routes_length);
      for( uint32_t i = 0; i < routes_length; i++){
      offset += this->routes[i].serialize(outbuffer + offset);
      }
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
      offset += this->header.deserialize(inbuffer + offset);
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

    virtual const char * getType() override { return "marti_nav_msgs/RouteArray"; };
    virtual const char * getMD5() override { return "a2f93d3e8f926d3456c8a282d8869e1f"; };

  };

}
#endif
