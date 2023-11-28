#ifndef _ROS_marti_common_msgs_Matrix3x3Stamped_h
#define _ROS_marti_common_msgs_Matrix3x3Stamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace marti_common_msgs
{

  class Matrix3x3Stamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      float matrix[9];

    Matrix3x3Stamped():
      header(),
      matrix()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      for( uint32_t i = 0; i < 9; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->matrix[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      for( uint32_t i = 0; i < 9; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->matrix[i]));
      }
     return offset;
    }

    virtual const char * getType() override { return "marti_common_msgs/Matrix3x3Stamped"; };
    virtual const char * getMD5() override { return "6f6aca2b78f71bd7b958ad349352f091"; };

  };

}
#endif
