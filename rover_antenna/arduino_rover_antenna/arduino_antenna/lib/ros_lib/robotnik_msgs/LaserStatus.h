#ifndef _ROS_robotnik_msgs_LaserStatus_h
#define _ROS_robotnik_msgs_LaserStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robotnik_msgs
{

  class LaserStatus : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      typedef bool _detecting_obstacles_type;
      _detecting_obstacles_type detecting_obstacles;
      typedef bool _contaminated_type;
      _contaminated_type contaminated;
      typedef bool _free_warning_type;
      _free_warning_type free_warning;
      uint32_t warning_zones_length;
      typedef bool _warning_zones_type;
      _warning_zones_type st_warning_zones;
      _warning_zones_type * warning_zones;

    LaserStatus():
      name(""),
      detecting_obstacles(0),
      contaminated(0),
      free_warning(0),
      warning_zones_length(0), st_warning_zones(), warning_zones(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      union {
        bool real;
        uint8_t base;
      } u_detecting_obstacles;
      u_detecting_obstacles.real = this->detecting_obstacles;
      *(outbuffer + offset + 0) = (u_detecting_obstacles.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->detecting_obstacles);
      union {
        bool real;
        uint8_t base;
      } u_contaminated;
      u_contaminated.real = this->contaminated;
      *(outbuffer + offset + 0) = (u_contaminated.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->contaminated);
      union {
        bool real;
        uint8_t base;
      } u_free_warning;
      u_free_warning.real = this->free_warning;
      *(outbuffer + offset + 0) = (u_free_warning.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->free_warning);
      *(outbuffer + offset + 0) = (this->warning_zones_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->warning_zones_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->warning_zones_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->warning_zones_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->warning_zones_length);
      for( uint32_t i = 0; i < warning_zones_length; i++){
      union {
        bool real;
        uint8_t base;
      } u_warning_zonesi;
      u_warning_zonesi.real = this->warning_zones[i];
      *(outbuffer + offset + 0) = (u_warning_zonesi.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->warning_zones[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      union {
        bool real;
        uint8_t base;
      } u_detecting_obstacles;
      u_detecting_obstacles.base = 0;
      u_detecting_obstacles.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->detecting_obstacles = u_detecting_obstacles.real;
      offset += sizeof(this->detecting_obstacles);
      union {
        bool real;
        uint8_t base;
      } u_contaminated;
      u_contaminated.base = 0;
      u_contaminated.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->contaminated = u_contaminated.real;
      offset += sizeof(this->contaminated);
      union {
        bool real;
        uint8_t base;
      } u_free_warning;
      u_free_warning.base = 0;
      u_free_warning.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->free_warning = u_free_warning.real;
      offset += sizeof(this->free_warning);
      uint32_t warning_zones_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      warning_zones_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      warning_zones_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      warning_zones_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->warning_zones_length);
      if(warning_zones_lengthT > warning_zones_length)
        this->warning_zones = (bool*)realloc(this->warning_zones, warning_zones_lengthT * sizeof(bool));
      warning_zones_length = warning_zones_lengthT;
      for( uint32_t i = 0; i < warning_zones_length; i++){
      union {
        bool real;
        uint8_t base;
      } u_st_warning_zones;
      u_st_warning_zones.base = 0;
      u_st_warning_zones.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->st_warning_zones = u_st_warning_zones.real;
      offset += sizeof(this->st_warning_zones);
        memcpy( &(this->warning_zones[i]), &(this->st_warning_zones), sizeof(bool));
      }
     return offset;
    }

    virtual const char * getType() override { return "robotnik_msgs/LaserStatus"; };
    virtual const char * getMD5() override { return "59f57d3a0c4aa9b97dcd8bd40152ebb4"; };

  };

}
#endif
