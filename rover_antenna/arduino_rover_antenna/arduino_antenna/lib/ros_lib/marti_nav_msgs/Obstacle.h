#ifndef _ROS_marti_nav_msgs_Obstacle_h
#define _ROS_marti_nav_msgs_Obstacle_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"

namespace marti_nav_msgs
{

  class Obstacle : public ros::Msg
  {
    public:
      typedef const char* _id_type;
      _id_type id;
      typedef geometry_msgs::Pose _pose_type;
      _pose_type pose;
      uint32_t polygon_length;
      typedef geometry_msgs::Point _polygon_type;
      _polygon_type st_polygon;
      _polygon_type * polygon;

    Obstacle():
      id(""),
      pose(),
      polygon_length(0), st_polygon(), polygon(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_id = strlen(this->id);
      varToArr(outbuffer + offset, length_id);
      offset += 4;
      memcpy(outbuffer + offset, this->id, length_id);
      offset += length_id;
      offset += this->pose.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->polygon_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->polygon_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->polygon_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->polygon_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->polygon_length);
      for( uint32_t i = 0; i < polygon_length; i++){
      offset += this->polygon[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_id;
      arrToVar(length_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_id-1]=0;
      this->id = (char *)(inbuffer + offset-1);
      offset += length_id;
      offset += this->pose.deserialize(inbuffer + offset);
      uint32_t polygon_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      polygon_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      polygon_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      polygon_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->polygon_length);
      if(polygon_lengthT > polygon_length)
        this->polygon = (geometry_msgs::Point*)realloc(this->polygon, polygon_lengthT * sizeof(geometry_msgs::Point));
      polygon_length = polygon_lengthT;
      for( uint32_t i = 0; i < polygon_length; i++){
      offset += this->st_polygon.deserialize(inbuffer + offset);
        memcpy( &(this->polygon[i]), &(this->st_polygon), sizeof(geometry_msgs::Point));
      }
     return offset;
    }

    virtual const char * getType() override { return "marti_nav_msgs/Obstacle"; };
    virtual const char * getMD5() override { return "6379634b2f186de37a480e1f3f9b2e7f"; };

  };

}
#endif
