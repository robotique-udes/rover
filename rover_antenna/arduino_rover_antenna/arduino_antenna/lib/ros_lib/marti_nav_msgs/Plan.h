#ifndef _ROS_marti_nav_msgs_Plan_h
#define _ROS_marti_nav_msgs_Plan_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "marti_nav_msgs/PlanPoint.h"
#include "marti_nav_msgs/PlanPointProperty.h"
#include "marti_common_msgs/KeyValue.h"

namespace marti_nav_msgs
{

  class Plan : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t points_length;
      typedef marti_nav_msgs::PlanPoint _points_type;
      _points_type st_points;
      _points_type * points;
      uint32_t point_properties_length;
      typedef marti_nav_msgs::PlanPointProperty _point_properties_type;
      _point_properties_type st_point_properties;
      _point_properties_type * point_properties;
      typedef const char* _id_type;
      _id_type id;
      uint32_t properties_length;
      typedef marti_common_msgs::KeyValue _properties_type;
      _properties_type st_properties;
      _properties_type * properties;

    Plan():
      header(),
      points_length(0), st_points(), points(nullptr),
      point_properties_length(0), st_point_properties(), point_properties(nullptr),
      id(""),
      properties_length(0), st_properties(), properties(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->points_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->points_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->points_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->points_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->points_length);
      for( uint32_t i = 0; i < points_length; i++){
      offset += this->points[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->point_properties_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->point_properties_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->point_properties_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->point_properties_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->point_properties_length);
      for( uint32_t i = 0; i < point_properties_length; i++){
      offset += this->point_properties[i].serialize(outbuffer + offset);
      }
      uint32_t length_id = strlen(this->id);
      varToArr(outbuffer + offset, length_id);
      offset += 4;
      memcpy(outbuffer + offset, this->id, length_id);
      offset += length_id;
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
      uint32_t points_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      points_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      points_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      points_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->points_length);
      if(points_lengthT > points_length)
        this->points = (marti_nav_msgs::PlanPoint*)realloc(this->points, points_lengthT * sizeof(marti_nav_msgs::PlanPoint));
      points_length = points_lengthT;
      for( uint32_t i = 0; i < points_length; i++){
      offset += this->st_points.deserialize(inbuffer + offset);
        memcpy( &(this->points[i]), &(this->st_points), sizeof(marti_nav_msgs::PlanPoint));
      }
      uint32_t point_properties_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      point_properties_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      point_properties_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      point_properties_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->point_properties_length);
      if(point_properties_lengthT > point_properties_length)
        this->point_properties = (marti_nav_msgs::PlanPointProperty*)realloc(this->point_properties, point_properties_lengthT * sizeof(marti_nav_msgs::PlanPointProperty));
      point_properties_length = point_properties_lengthT;
      for( uint32_t i = 0; i < point_properties_length; i++){
      offset += this->st_point_properties.deserialize(inbuffer + offset);
        memcpy( &(this->point_properties[i]), &(this->st_point_properties), sizeof(marti_nav_msgs::PlanPointProperty));
      }
      uint32_t length_id;
      arrToVar(length_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_id-1]=0;
      this->id = (char *)(inbuffer + offset-1);
      offset += length_id;
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

    virtual const char * getType() override { return "marti_nav_msgs/Plan"; };
    virtual const char * getMD5() override { return "f87359e0c38bc2de4c9250c56b8b6d3f"; };

  };

}
#endif
