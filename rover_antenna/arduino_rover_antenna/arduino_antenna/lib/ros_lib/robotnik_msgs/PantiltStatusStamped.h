#ifndef _ROS_robotnik_msgs_PantiltStatusStamped_h
#define _ROS_robotnik_msgs_PantiltStatusStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "robotnik_msgs/PantiltStatus.h"

namespace robotnik_msgs
{

  class PantiltStatusStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef robotnik_msgs::PantiltStatus _pantilt_type;
      _pantilt_type pantilt;

    PantiltStatusStamped():
      header(),
      pantilt()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->pantilt.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->pantilt.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "robotnik_msgs/PantiltStatusStamped"; };
    virtual const char * getMD5() override { return "be63351895b74b23132ea9e341c93843"; };

  };

}
#endif
