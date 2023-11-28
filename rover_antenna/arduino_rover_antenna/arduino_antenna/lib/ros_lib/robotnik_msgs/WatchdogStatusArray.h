#ifndef _ROS_robotnik_msgs_WatchdogStatusArray_h
#define _ROS_robotnik_msgs_WatchdogStatusArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "robotnik_msgs/WatchdogStatus.h"

namespace robotnik_msgs
{

  class WatchdogStatusArray : public ros::Msg
  {
    public:
      uint32_t status_length;
      typedef robotnik_msgs::WatchdogStatus _status_type;
      _status_type st_status;
      _status_type * status;

    WatchdogStatusArray():
      status_length(0), st_status(), status(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->status_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->status_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->status_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->status_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->status_length);
      for( uint32_t i = 0; i < status_length; i++){
      offset += this->status[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t status_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      status_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      status_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      status_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->status_length);
      if(status_lengthT > status_length)
        this->status = (robotnik_msgs::WatchdogStatus*)realloc(this->status, status_lengthT * sizeof(robotnik_msgs::WatchdogStatus));
      status_length = status_lengthT;
      for( uint32_t i = 0; i < status_length; i++){
      offset += this->st_status.deserialize(inbuffer + offset);
        memcpy( &(this->status[i]), &(this->st_status), sizeof(robotnik_msgs::WatchdogStatus));
      }
     return offset;
    }

    virtual const char * getType() override { return "robotnik_msgs/WatchdogStatusArray"; };
    virtual const char * getMD5() override { return "1df2dbf9ebcb5eb8dc5708907e8cc240"; };

  };

}
#endif
