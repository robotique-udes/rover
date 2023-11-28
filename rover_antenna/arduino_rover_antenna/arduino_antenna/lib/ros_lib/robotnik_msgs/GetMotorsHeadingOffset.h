#ifndef _ROS_SERVICE_GetMotorsHeadingOffset_h
#define _ROS_SERVICE_GetMotorsHeadingOffset_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "robotnik_msgs/MotorHeadingOffset.h"
#include "std_msgs/Empty.h"

namespace robotnik_msgs
{

static const char GETMOTORSHEADINGOFFSET[] = "robotnik_msgs/GetMotorsHeadingOffset";

  class GetMotorsHeadingOffsetRequest : public ros::Msg
  {
    public:
      typedef std_msgs::Empty _request_type;
      _request_type request;

    GetMotorsHeadingOffsetRequest():
      request()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->request.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->request.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return GETMOTORSHEADINGOFFSET; };
    virtual const char * getMD5() override { return "57c42d82c9bab8a1ec9de111b7540471"; };

  };

  class GetMotorsHeadingOffsetResponse : public ros::Msg
  {
    public:
      uint32_t offsets_length;
      typedef robotnik_msgs::MotorHeadingOffset _offsets_type;
      _offsets_type st_offsets;
      _offsets_type * offsets;

    GetMotorsHeadingOffsetResponse():
      offsets_length(0), st_offsets(), offsets(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->offsets_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->offsets_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->offsets_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->offsets_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->offsets_length);
      for( uint32_t i = 0; i < offsets_length; i++){
      offset += this->offsets[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t offsets_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      offsets_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      offsets_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      offsets_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->offsets_length);
      if(offsets_lengthT > offsets_length)
        this->offsets = (robotnik_msgs::MotorHeadingOffset*)realloc(this->offsets, offsets_lengthT * sizeof(robotnik_msgs::MotorHeadingOffset));
      offsets_length = offsets_lengthT;
      for( uint32_t i = 0; i < offsets_length; i++){
      offset += this->st_offsets.deserialize(inbuffer + offset);
        memcpy( &(this->offsets[i]), &(this->st_offsets), sizeof(robotnik_msgs::MotorHeadingOffset));
      }
     return offset;
    }

    virtual const char * getType() override { return GETMOTORSHEADINGOFFSET; };
    virtual const char * getMD5() override { return "5893bfa41a37a6679598a2513e065848"; };

  };

  class GetMotorsHeadingOffset {
    public:
    typedef GetMotorsHeadingOffsetRequest Request;
    typedef GetMotorsHeadingOffsetResponse Response;
  };

}
#endif
