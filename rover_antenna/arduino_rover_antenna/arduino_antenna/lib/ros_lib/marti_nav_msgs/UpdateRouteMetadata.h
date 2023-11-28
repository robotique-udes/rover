#ifndef _ROS_SERVICE_UpdateRouteMetadata_h
#define _ROS_SERVICE_UpdateRouteMetadata_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "marti_nav_msgs/RoutePoint.h"

namespace marti_nav_msgs
{

static const char UPDATEROUTEMETADATA[] = "marti_nav_msgs/UpdateRouteMetadata";

  class UpdateRouteMetadataRequest : public ros::Msg
  {
    public:
      typedef const char* _route_guid_type;
      _route_guid_type route_guid;
      uint32_t metadata_points_length;
      typedef marti_nav_msgs::RoutePoint _metadata_points_type;
      _metadata_points_type st_metadata_points;
      _metadata_points_type * metadata_points;

    UpdateRouteMetadataRequest():
      route_guid(""),
      metadata_points_length(0), st_metadata_points(), metadata_points(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_route_guid = strlen(this->route_guid);
      varToArr(outbuffer + offset, length_route_guid);
      offset += 4;
      memcpy(outbuffer + offset, this->route_guid, length_route_guid);
      offset += length_route_guid;
      *(outbuffer + offset + 0) = (this->metadata_points_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->metadata_points_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->metadata_points_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->metadata_points_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->metadata_points_length);
      for( uint32_t i = 0; i < metadata_points_length; i++){
      offset += this->metadata_points[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_route_guid;
      arrToVar(length_route_guid, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_route_guid; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_route_guid-1]=0;
      this->route_guid = (char *)(inbuffer + offset-1);
      offset += length_route_guid;
      uint32_t metadata_points_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      metadata_points_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      metadata_points_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      metadata_points_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->metadata_points_length);
      if(metadata_points_lengthT > metadata_points_length)
        this->metadata_points = (marti_nav_msgs::RoutePoint*)realloc(this->metadata_points, metadata_points_lengthT * sizeof(marti_nav_msgs::RoutePoint));
      metadata_points_length = metadata_points_lengthT;
      for( uint32_t i = 0; i < metadata_points_length; i++){
      offset += this->st_metadata_points.deserialize(inbuffer + offset);
        memcpy( &(this->metadata_points[i]), &(this->st_metadata_points), sizeof(marti_nav_msgs::RoutePoint));
      }
     return offset;
    }

    virtual const char * getType() override { return UPDATEROUTEMETADATA; };
    virtual const char * getMD5() override { return "4326dd3985865ba6412643260ac9da6f"; };

  };

  class UpdateRouteMetadataResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;
      typedef const char* _message_type;
      _message_type message;

    UpdateRouteMetadataResponse():
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

    virtual const char * getType() override { return UPDATEROUTEMETADATA; };
    virtual const char * getMD5() override { return "937c9679a518e3a18d831e57125ea522"; };

  };

  class UpdateRouteMetadata {
    public:
    typedef UpdateRouteMetadataRequest Request;
    typedef UpdateRouteMetadataResponse Response;
  };

}
#endif
