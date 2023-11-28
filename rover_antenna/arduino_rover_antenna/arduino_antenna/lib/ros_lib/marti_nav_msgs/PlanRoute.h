#ifndef _ROS_SERVICE_PlanRoute_h
#define _ROS_SERVICE_PlanRoute_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "marti_nav_msgs/Route.h"
#include "geometry_msgs/Pose.h"

namespace marti_nav_msgs
{

static const char PLANROUTE[] = "marti_nav_msgs/PlanRoute";

  class PlanRouteRequest : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t waypoints_length;
      typedef geometry_msgs::Pose _waypoints_type;
      _waypoints_type st_waypoints;
      _waypoints_type * waypoints;
      typedef bool _plan_from_vehicle_type;
      _plan_from_vehicle_type plan_from_vehicle;

    PlanRouteRequest():
      header(),
      waypoints_length(0), st_waypoints(), waypoints(nullptr),
      plan_from_vehicle(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->waypoints_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->waypoints_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->waypoints_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->waypoints_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->waypoints_length);
      for( uint32_t i = 0; i < waypoints_length; i++){
      offset += this->waypoints[i].serialize(outbuffer + offset);
      }
      union {
        bool real;
        uint8_t base;
      } u_plan_from_vehicle;
      u_plan_from_vehicle.real = this->plan_from_vehicle;
      *(outbuffer + offset + 0) = (u_plan_from_vehicle.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->plan_from_vehicle);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t waypoints_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      waypoints_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      waypoints_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      waypoints_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->waypoints_length);
      if(waypoints_lengthT > waypoints_length)
        this->waypoints = (geometry_msgs::Pose*)realloc(this->waypoints, waypoints_lengthT * sizeof(geometry_msgs::Pose));
      waypoints_length = waypoints_lengthT;
      for( uint32_t i = 0; i < waypoints_length; i++){
      offset += this->st_waypoints.deserialize(inbuffer + offset);
        memcpy( &(this->waypoints[i]), &(this->st_waypoints), sizeof(geometry_msgs::Pose));
      }
      union {
        bool real;
        uint8_t base;
      } u_plan_from_vehicle;
      u_plan_from_vehicle.base = 0;
      u_plan_from_vehicle.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->plan_from_vehicle = u_plan_from_vehicle.real;
      offset += sizeof(this->plan_from_vehicle);
     return offset;
    }

    virtual const char * getType() override { return PLANROUTE; };
    virtual const char * getMD5() override { return "9f8dfd7f9c9e104ba05adfcc37f17c42"; };

  };

  class PlanRouteResponse : public ros::Msg
  {
    public:
      typedef marti_nav_msgs::Route _route_type;
      _route_type route;
      typedef bool _success_type;
      _success_type success;
      typedef const char* _message_type;
      _message_type message;
      typedef float _cost_type;
      _cost_type cost;

    PlanRouteResponse():
      route(),
      success(0),
      message(""),
      cost(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->route.serialize(outbuffer + offset);
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
      offset += serializeAvrFloat64(outbuffer + offset, this->cost);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->route.deserialize(inbuffer + offset);
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
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->cost));
     return offset;
    }

    virtual const char * getType() override { return PLANROUTE; };
    virtual const char * getMD5() override { return "e219318be7814c7e021480543e1b3b56"; };

  };

  class PlanRoute {
    public:
    typedef PlanRouteRequest Request;
    typedef PlanRouteResponse Response;
  };

}
#endif
