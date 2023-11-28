#ifndef _ROS_marti_nav_msgs_TeleopState_h
#define _ROS_marti_nav_msgs_TeleopState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace marti_nav_msgs
{

  class TeleopState : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t teleopSignals_length;
      typedef int32_t _teleopSignals_type;
      _teleopSignals_type st_teleopSignals;
      _teleopSignals_type * teleopSignals;

    TeleopState():
      header(),
      teleopSignals_length(0), st_teleopSignals(), teleopSignals(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->teleopSignals_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->teleopSignals_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->teleopSignals_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->teleopSignals_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->teleopSignals_length);
      for( uint32_t i = 0; i < teleopSignals_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_teleopSignalsi;
      u_teleopSignalsi.real = this->teleopSignals[i];
      *(outbuffer + offset + 0) = (u_teleopSignalsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_teleopSignalsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_teleopSignalsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_teleopSignalsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->teleopSignals[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t teleopSignals_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      teleopSignals_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      teleopSignals_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      teleopSignals_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->teleopSignals_length);
      if(teleopSignals_lengthT > teleopSignals_length)
        this->teleopSignals = (int32_t*)realloc(this->teleopSignals, teleopSignals_lengthT * sizeof(int32_t));
      teleopSignals_length = teleopSignals_lengthT;
      for( uint32_t i = 0; i < teleopSignals_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_teleopSignals;
      u_st_teleopSignals.base = 0;
      u_st_teleopSignals.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_teleopSignals.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_teleopSignals.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_teleopSignals.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_teleopSignals = u_st_teleopSignals.real;
      offset += sizeof(this->st_teleopSignals);
        memcpy( &(this->teleopSignals[i]), &(this->st_teleopSignals), sizeof(int32_t));
      }
     return offset;
    }

    virtual const char * getType() override { return "marti_nav_msgs/TeleopState"; };
    virtual const char * getMD5() override { return "7af42bf9109e393cbfb4bd740df95c1e"; };

  };

}
#endif
