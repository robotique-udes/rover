#ifndef _ROS_marti_sensor_msgs_WheelEncoder_h
#define _ROS_marti_sensor_msgs_WheelEncoder_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace marti_sensor_msgs
{

  class WheelEncoder : public ros::Msg
  {
    public:
      typedef float _frequency_type;
      _frequency_type frequency;
      typedef bool _directional_type;
      _directional_type directional;
      typedef uint8_t _id_type;
      _id_type id;

    WheelEncoder():
      frequency(0),
      directional(0),
      id(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->frequency);
      union {
        bool real;
        uint8_t base;
      } u_directional;
      u_directional.real = this->directional;
      *(outbuffer + offset + 0) = (u_directional.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->directional);
      *(outbuffer + offset + 0) = (this->id >> (8 * 0)) & 0xFF;
      offset += sizeof(this->id);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->frequency));
      union {
        bool real;
        uint8_t base;
      } u_directional;
      u_directional.base = 0;
      u_directional.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->directional = u_directional.real;
      offset += sizeof(this->directional);
      this->id =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->id);
     return offset;
    }

    virtual const char * getType() override { return "marti_sensor_msgs/WheelEncoder"; };
    virtual const char * getMD5() override { return "18725be6b3a6d8aef45fc5b0d63b9a58"; };

  };

}
#endif
