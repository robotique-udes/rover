#ifndef _ROS_marti_nav_msgs_GridMap_h
#define _ROS_marti_nav_msgs_GridMap_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/Image.h"

namespace marti_nav_msgs
{

  class GridMap : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef geometry_msgs::Point _top_left_type;
      _top_left_type top_left;
      typedef geometry_msgs::Point _top_right_type;
      _top_right_type top_right;
      typedef geometry_msgs::Point _bottom_right_type;
      _bottom_right_type bottom_right;
      typedef geometry_msgs::Point _bottom_left_type;
      _bottom_left_type bottom_left;
      uint32_t map_names_length;
      typedef char* _map_names_type;
      _map_names_type st_map_names;
      _map_names_type * map_names;
      uint32_t map_data_length;
      typedef sensor_msgs::Image _map_data_type;
      _map_data_type st_map_data;
      _map_data_type * map_data;

    GridMap():
      header(),
      top_left(),
      top_right(),
      bottom_right(),
      bottom_left(),
      map_names_length(0), st_map_names(), map_names(nullptr),
      map_data_length(0), st_map_data(), map_data(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->top_left.serialize(outbuffer + offset);
      offset += this->top_right.serialize(outbuffer + offset);
      offset += this->bottom_right.serialize(outbuffer + offset);
      offset += this->bottom_left.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->map_names_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->map_names_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->map_names_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->map_names_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->map_names_length);
      for( uint32_t i = 0; i < map_names_length; i++){
      uint32_t length_map_namesi = strlen(this->map_names[i]);
      varToArr(outbuffer + offset, length_map_namesi);
      offset += 4;
      memcpy(outbuffer + offset, this->map_names[i], length_map_namesi);
      offset += length_map_namesi;
      }
      *(outbuffer + offset + 0) = (this->map_data_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->map_data_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->map_data_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->map_data_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->map_data_length);
      for( uint32_t i = 0; i < map_data_length; i++){
      offset += this->map_data[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->top_left.deserialize(inbuffer + offset);
      offset += this->top_right.deserialize(inbuffer + offset);
      offset += this->bottom_right.deserialize(inbuffer + offset);
      offset += this->bottom_left.deserialize(inbuffer + offset);
      uint32_t map_names_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      map_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      map_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      map_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->map_names_length);
      if(map_names_lengthT > map_names_length)
        this->map_names = (char**)realloc(this->map_names, map_names_lengthT * sizeof(char*));
      map_names_length = map_names_lengthT;
      for( uint32_t i = 0; i < map_names_length; i++){
      uint32_t length_st_map_names;
      arrToVar(length_st_map_names, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_map_names; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_map_names-1]=0;
      this->st_map_names = (char *)(inbuffer + offset-1);
      offset += length_st_map_names;
        memcpy( &(this->map_names[i]), &(this->st_map_names), sizeof(char*));
      }
      uint32_t map_data_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      map_data_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      map_data_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      map_data_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->map_data_length);
      if(map_data_lengthT > map_data_length)
        this->map_data = (sensor_msgs::Image*)realloc(this->map_data, map_data_lengthT * sizeof(sensor_msgs::Image));
      map_data_length = map_data_lengthT;
      for( uint32_t i = 0; i < map_data_length; i++){
      offset += this->st_map_data.deserialize(inbuffer + offset);
        memcpy( &(this->map_data[i]), &(this->st_map_data), sizeof(sensor_msgs::Image));
      }
     return offset;
    }

    virtual const char * getType() override { return "marti_nav_msgs/GridMap"; };
    virtual const char * getMD5() override { return "3b88254125f8a40bfc5628e3d7439242"; };

  };

}
#endif
