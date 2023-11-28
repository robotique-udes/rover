#ifndef _ROS_marti_nav_msgs_PlanTrack_h
#define _ROS_marti_nav_msgs_PlanTrack_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"
#include "marti_nav_msgs/PlanPosition.h"

namespace marti_nav_msgs
{

  class PlanTrack : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _plan_id_type;
      _plan_id_type plan_id;
      typedef geometry_msgs::Pose _relative_pose_type;
      _relative_pose_type relative_pose;
      typedef marti_nav_msgs::PlanPosition _plan_position_type;
      _plan_position_type plan_position;

    PlanTrack():
      header(),
      plan_id(""),
      relative_pose(),
      plan_position()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_plan_id = strlen(this->plan_id);
      varToArr(outbuffer + offset, length_plan_id);
      offset += 4;
      memcpy(outbuffer + offset, this->plan_id, length_plan_id);
      offset += length_plan_id;
      offset += this->relative_pose.serialize(outbuffer + offset);
      offset += this->plan_position.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_plan_id;
      arrToVar(length_plan_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_plan_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_plan_id-1]=0;
      this->plan_id = (char *)(inbuffer + offset-1);
      offset += length_plan_id;
      offset += this->relative_pose.deserialize(inbuffer + offset);
      offset += this->plan_position.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "marti_nav_msgs/PlanTrack"; };
    virtual const char * getMD5() override { return "272b3a115ddb82707402b64c92ff6fa0"; };

  };

}
#endif
