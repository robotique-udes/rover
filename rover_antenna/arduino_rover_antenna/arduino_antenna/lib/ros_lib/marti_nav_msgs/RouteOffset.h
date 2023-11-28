#ifndef _ROS_marti_nav_msgs_RouteOffset_h
#define _ROS_marti_nav_msgs_RouteOffset_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"
#include "marti_nav_msgs/RoutePosition.h"

namespace marti_nav_msgs
{

  class RouteOffset : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef geometry_msgs::Pose _relative_pose_type;
      _relative_pose_type relative_pose;
      typedef marti_nav_msgs::RoutePosition _route_position_type;
      _route_position_type route_position;

    RouteOffset():
      header(),
      relative_pose(),
      route_position()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->relative_pose.serialize(outbuffer + offset);
      offset += this->route_position.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->relative_pose.deserialize(inbuffer + offset);
      offset += this->route_position.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "marti_nav_msgs/RouteOffset"; };
    virtual const char * getMD5() override { return "69208bbf68ef432cb1b3530d6fdda6cf"; };

  };

}
#endif
