#ifndef _ROS_marti_sensor_msgs_DioRealTimeData_h
#define _ROS_marti_sensor_msgs_DioRealTimeData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace marti_sensor_msgs
{

  class DioRealTimeData : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _sample_frequency_type;
      _sample_frequency_type sample_frequency;
      typedef uint64_t _latest_sample_time_type;
      _latest_sample_time_type latest_sample_time;
      uint32_t sample_states_length;
      typedef uint16_t _sample_states_type;
      _sample_states_type st_sample_states;
      _sample_states_type * sample_states;
      uint32_t sample_times_length;
      typedef uint32_t _sample_times_type;
      _sample_times_type st_sample_times;
      _sample_times_type * sample_times;

    DioRealTimeData():
      header(),
      sample_frequency(0),
      latest_sample_time(0),
      sample_states_length(0), st_sample_states(), sample_states(nullptr),
      sample_times_length(0), st_sample_times(), sample_times(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->sample_frequency);
      *(outbuffer + offset + 0) = (this->latest_sample_time >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->latest_sample_time >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->latest_sample_time >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->latest_sample_time >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (this->latest_sample_time >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (this->latest_sample_time >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (this->latest_sample_time >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (this->latest_sample_time >> (8 * 7)) & 0xFF;
      offset += sizeof(this->latest_sample_time);
      *(outbuffer + offset + 0) = (this->sample_states_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sample_states_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sample_states_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sample_states_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sample_states_length);
      for( uint32_t i = 0; i < sample_states_length; i++){
      *(outbuffer + offset + 0) = (this->sample_states[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sample_states[i] >> (8 * 1)) & 0xFF;
      offset += sizeof(this->sample_states[i]);
      }
      *(outbuffer + offset + 0) = (this->sample_times_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sample_times_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sample_times_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sample_times_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sample_times_length);
      for( uint32_t i = 0; i < sample_times_length; i++){
      *(outbuffer + offset + 0) = (this->sample_times[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sample_times[i] >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sample_times[i] >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sample_times[i] >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sample_times[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->sample_frequency));
      this->latest_sample_time =  ((uint64_t) (*(inbuffer + offset)));
      this->latest_sample_time |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->latest_sample_time |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->latest_sample_time |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->latest_sample_time |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      this->latest_sample_time |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      this->latest_sample_time |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      this->latest_sample_time |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      offset += sizeof(this->latest_sample_time);
      uint32_t sample_states_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      sample_states_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      sample_states_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      sample_states_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->sample_states_length);
      if(sample_states_lengthT > sample_states_length)
        this->sample_states = (uint16_t*)realloc(this->sample_states, sample_states_lengthT * sizeof(uint16_t));
      sample_states_length = sample_states_lengthT;
      for( uint32_t i = 0; i < sample_states_length; i++){
      this->st_sample_states =  ((uint16_t) (*(inbuffer + offset)));
      this->st_sample_states |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->st_sample_states);
        memcpy( &(this->sample_states[i]), &(this->st_sample_states), sizeof(uint16_t));
      }
      uint32_t sample_times_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      sample_times_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      sample_times_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      sample_times_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->sample_times_length);
      if(sample_times_lengthT > sample_times_length)
        this->sample_times = (uint32_t*)realloc(this->sample_times, sample_times_lengthT * sizeof(uint32_t));
      sample_times_length = sample_times_lengthT;
      for( uint32_t i = 0; i < sample_times_length; i++){
      this->st_sample_times =  ((uint32_t) (*(inbuffer + offset)));
      this->st_sample_times |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_sample_times |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->st_sample_times |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->st_sample_times);
        memcpy( &(this->sample_times[i]), &(this->st_sample_times), sizeof(uint32_t));
      }
     return offset;
    }

    virtual const char * getType() override { return "marti_sensor_msgs/DioRealTimeData"; };
    virtual const char * getMD5() override { return "98eef69989f7d8b60307368e8e339ff4"; };

  };

}
#endif
