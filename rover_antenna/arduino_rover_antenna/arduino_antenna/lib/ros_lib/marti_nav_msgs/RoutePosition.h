#ifndef _ROS_marti_nav_msgs_RoutePosition_h
#define _ROS_marti_nav_msgs_RoutePosition_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace marti_nav_msgs
{

  class RoutePosition : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _route_id_type;
      _route_id_type route_id;
      typedef const char* _id_type;
      _id_type id;
      typedef float _distance_type;
      _distance_type distance;

    RoutePosition():
      header(),
      route_id(""),
      id(""),
      distance(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_route_id = strlen(this->route_id);
      varToArr(outbuffer + offset, length_route_id);
      offset += 4;
      memcpy(outbuffer + offset, this->route_id, length_route_id);
      offset += length_route_id;
      uint32_t length_id = strlen(this->id);
      varToArr(outbuffer + offset, length_id);
      offset += 4;
      memcpy(outbuffer + offset, this->id, length_id);
      offset += length_id;
      union {
        float real;
        uint32_t base;
      } u_distance;
      u_distance.real = this->distance;
      *(outbuffer + offset + 0) = (u_distance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_distance.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_distance.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_distance.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->distance);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_route_id;
      arrToVar(length_route_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_route_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_route_id-1]=0;
      this->route_id = (char *)(inbuffer + offset-1);
      offset += length_route_id;
      uint32_t length_id;
      arrToVar(length_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_id-1]=0;
      this->id = (char *)(inbuffer + offset-1);
      offset += length_id;
      union {
        float real;
        uint32_t base;
      } u_distance;
      u_distance.base = 0;
      u_distance.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_distance.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_distance.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_distance.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->distance = u_distance.real;
      offset += sizeof(this->distance);
     return offset;
    }

    virtual const char * getType() override { return "marti_nav_msgs/RoutePosition"; };
    virtual const char * getMD5() override { return "7b490dd73143bf1ce8ca12b9478c3d9d"; };

  };

}
#endif
