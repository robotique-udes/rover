#ifndef _ROS_marti_nav_msgs_PlanPointProperty_h
#define _ROS_marti_nav_msgs_PlanPointProperty_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "marti_common_msgs/KeyValue.h"

namespace marti_nav_msgs
{

  class PlanPointProperty : public ros::Msg
  {
    public:
      typedef uint64_t _index_type;
      _index_type index;
      uint32_t properties_length;
      typedef marti_common_msgs::KeyValue _properties_type;
      _properties_type st_properties;
      _properties_type * properties;

    PlanPointProperty():
      index(0),
      properties_length(0), st_properties(), properties(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->index >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->index >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->index >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->index >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (this->index >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (this->index >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (this->index >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (this->index >> (8 * 7)) & 0xFF;
      offset += sizeof(this->index);
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
      this->index =  ((uint64_t) (*(inbuffer + offset)));
      this->index |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->index |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->index |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->index |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      this->index |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      this->index |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      this->index |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      offset += sizeof(this->index);
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

    virtual const char * getType() override { return "marti_nav_msgs/PlanPointProperty"; };
    virtual const char * getMD5() override { return "d0f376742393237e9e7dab66205838a9"; };

  };

}
#endif
