#ifndef _ROS_marti_nav_msgs_PlanPosition_h
#define _ROS_marti_nav_msgs_PlanPosition_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace marti_nav_msgs
{

  class PlanPosition : public ros::Msg
  {
    public:
      typedef uint64_t _index_type;
      _index_type index;
      typedef float _distance_type;
      _distance_type distance;

    PlanPosition():
      index(0),
      distance(0)
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
      offset += serializeAvrFloat64(outbuffer + offset, this->distance);
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
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->distance));
     return offset;
    }

    virtual const char * getType() override { return "marti_nav_msgs/PlanPosition"; };
    virtual const char * getMD5() override { return "74ccd18bd21b404ec69bcdd443292f74"; };

  };

}
#endif
