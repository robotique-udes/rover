#ifndef _ROS_robotnik_msgs_Cartesian_Euler_pose_h
#define _ROS_robotnik_msgs_Cartesian_Euler_pose_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robotnik_msgs
{

  class Cartesian_Euler_pose : public ros::Msg
  {
    public:
      typedef float _x_type;
      _x_type x;
      typedef float _y_type;
      _y_type y;
      typedef float _z_type;
      _z_type z;
      typedef float _A_type;
      _A_type A;
      typedef float _B_type;
      _B_type B;
      typedef float _C_type;
      _C_type C;

    Cartesian_Euler_pose():
      x(0),
      y(0),
      z(0),
      A(0),
      B(0),
      C(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->x);
      offset += serializeAvrFloat64(outbuffer + offset, this->y);
      offset += serializeAvrFloat64(outbuffer + offset, this->z);
      offset += serializeAvrFloat64(outbuffer + offset, this->A);
      offset += serializeAvrFloat64(outbuffer + offset, this->B);
      offset += serializeAvrFloat64(outbuffer + offset, this->C);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->x));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->y));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->z));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->A));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->B));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->C));
     return offset;
    }

    virtual const char * getType() override { return "robotnik_msgs/Cartesian_Euler_pose"; };
    virtual const char * getMD5() override { return "1b7d5ac5679ead09b31ec87e784aa10e"; };

  };

}
#endif
