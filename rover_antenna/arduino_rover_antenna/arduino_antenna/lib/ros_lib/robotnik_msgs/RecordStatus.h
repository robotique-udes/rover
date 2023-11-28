#ifndef _ROS_robotnik_msgs_RecordStatus_h
#define _ROS_robotnik_msgs_RecordStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robotnik_msgs
{

  class RecordStatus : public ros::Msg
  {
    public:
      typedef bool _recording_type;
      _recording_type recording;
      typedef int32_t _recording_time_type;
      _recording_time_type recording_time;
      typedef const char* _state_description_type;
      _state_description_type state_description;

    RecordStatus():
      recording(0),
      recording_time(0),
      state_description("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_recording;
      u_recording.real = this->recording;
      *(outbuffer + offset + 0) = (u_recording.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->recording);
      union {
        int32_t real;
        uint32_t base;
      } u_recording_time;
      u_recording_time.real = this->recording_time;
      *(outbuffer + offset + 0) = (u_recording_time.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_recording_time.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_recording_time.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_recording_time.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->recording_time);
      uint32_t length_state_description = strlen(this->state_description);
      varToArr(outbuffer + offset, length_state_description);
      offset += 4;
      memcpy(outbuffer + offset, this->state_description, length_state_description);
      offset += length_state_description;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_recording;
      u_recording.base = 0;
      u_recording.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->recording = u_recording.real;
      offset += sizeof(this->recording);
      union {
        int32_t real;
        uint32_t base;
      } u_recording_time;
      u_recording_time.base = 0;
      u_recording_time.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_recording_time.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_recording_time.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_recording_time.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->recording_time = u_recording_time.real;
      offset += sizeof(this->recording_time);
      uint32_t length_state_description;
      arrToVar(length_state_description, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_state_description; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_state_description-1]=0;
      this->state_description = (char *)(inbuffer + offset-1);
      offset += length_state_description;
     return offset;
    }

    virtual const char * getType() override { return "robotnik_msgs/RecordStatus"; };
    virtual const char * getMD5() override { return "536d9648c6be2c09d55b8554e09a4f2c"; };

  };

}
#endif
