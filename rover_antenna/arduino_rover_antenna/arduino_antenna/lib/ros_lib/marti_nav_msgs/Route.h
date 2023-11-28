#ifndef _ROS_marti_nav_msgs_Route_h
#define _ROS_marti_nav_msgs_Route_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "marti_nav_msgs/RoutePoint.h"
#include "marti_common_msgs/KeyValue.h"

namespace marti_nav_msgs
{

  class Route : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t route_points_length;
      typedef marti_nav_msgs::RoutePoint _route_points_type;
      _route_points_type st_route_points;
      _route_points_type * route_points;
      uint32_t properties_length;
      typedef marti_common_msgs::KeyValue _properties_type;
      _properties_type st_properties;
      _properties_type * properties;

    Route():
      header(),
      route_points_length(0), st_route_points(), route_points(nullptr),
      properties_length(0), st_properties(), properties(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->route_points_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->route_points_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->route_points_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->route_points_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->route_points_length);
      for( uint32_t i = 0; i < route_points_length; i++){
      offset += this->route_points[i].serialize(outbuffer + offset);
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
      uint32_t route_points_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      route_points_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      route_points_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      route_points_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->route_points_length);
      if(route_points_lengthT > route_points_length)
        this->route_points = (marti_nav_msgs::RoutePoint*)realloc(this->route_points, route_points_lengthT * sizeof(marti_nav_msgs::RoutePoint));
      route_points_length = route_points_lengthT;
      for( uint32_t i = 0; i < route_points_length; i++){
      offset += this->st_route_points.deserialize(inbuffer + offset);
        memcpy( &(this->route_points[i]), &(this->st_route_points), sizeof(marti_nav_msgs::RoutePoint));
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

    virtual const char * getType() override { return "marti_nav_msgs/Route"; };
    virtual const char * getMD5() override { return "626dfe06202116afac99e6de9fa42b3e"; };

  };

}
#endif
