#ifndef _ROS_robotnik_msgs_SafetyModuleStatus_h
#define _ROS_robotnik_msgs_SafetyModuleStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "robotnik_msgs/LaserMode.h"
#include "robotnik_msgs/LaserStatus.h"

namespace robotnik_msgs
{

  class SafetyModuleStatus : public ros::Msg
  {
    public:
      typedef const char* _operation_mode_type;
      _operation_mode_type operation_mode;
      typedef const char* _safety_mode_type;
      _safety_mode_type safety_mode;
      typedef bool _emergency_stop_type;
      _emergency_stop_type emergency_stop;
      typedef bool _safety_stop_type;
      _safety_stop_type safety_stop;
      typedef bool _lasers_on_standby_type;
      _lasers_on_standby_type lasers_on_standby;
      typedef float _current_speed_type;
      _current_speed_type current_speed;
      typedef robotnik_msgs::LaserMode _lasers_mode_type;
      _lasers_mode_type lasers_mode;
      uint32_t lasers_status_length;
      typedef robotnik_msgs::LaserStatus _lasers_status_type;
      _lasers_status_type st_lasers_status;
      _lasers_status_type * lasers_status;
      enum { SAFE = safe };
      enum { LASER_MUTE = laser_mute };
      enum { OM_AUTO = auto };
      enum { OM_MANUAL = manual };
      enum { OM_MAINTENANCE = maintenance };

    SafetyModuleStatus():
      operation_mode(""),
      safety_mode(""),
      emergency_stop(0),
      safety_stop(0),
      lasers_on_standby(0),
      current_speed(0),
      lasers_mode(),
      lasers_status_length(0), st_lasers_status(), lasers_status(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_operation_mode = strlen(this->operation_mode);
      varToArr(outbuffer + offset, length_operation_mode);
      offset += 4;
      memcpy(outbuffer + offset, this->operation_mode, length_operation_mode);
      offset += length_operation_mode;
      uint32_t length_safety_mode = strlen(this->safety_mode);
      varToArr(outbuffer + offset, length_safety_mode);
      offset += 4;
      memcpy(outbuffer + offset, this->safety_mode, length_safety_mode);
      offset += length_safety_mode;
      union {
        bool real;
        uint8_t base;
      } u_emergency_stop;
      u_emergency_stop.real = this->emergency_stop;
      *(outbuffer + offset + 0) = (u_emergency_stop.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->emergency_stop);
      union {
        bool real;
        uint8_t base;
      } u_safety_stop;
      u_safety_stop.real = this->safety_stop;
      *(outbuffer + offset + 0) = (u_safety_stop.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->safety_stop);
      union {
        bool real;
        uint8_t base;
      } u_lasers_on_standby;
      u_lasers_on_standby.real = this->lasers_on_standby;
      *(outbuffer + offset + 0) = (u_lasers_on_standby.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->lasers_on_standby);
      offset += serializeAvrFloat64(outbuffer + offset, this->current_speed);
      offset += this->lasers_mode.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->lasers_status_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->lasers_status_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->lasers_status_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->lasers_status_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->lasers_status_length);
      for( uint32_t i = 0; i < lasers_status_length; i++){
      offset += this->lasers_status[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_operation_mode;
      arrToVar(length_operation_mode, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_operation_mode; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_operation_mode-1]=0;
      this->operation_mode = (char *)(inbuffer + offset-1);
      offset += length_operation_mode;
      uint32_t length_safety_mode;
      arrToVar(length_safety_mode, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_safety_mode; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_safety_mode-1]=0;
      this->safety_mode = (char *)(inbuffer + offset-1);
      offset += length_safety_mode;
      union {
        bool real;
        uint8_t base;
      } u_emergency_stop;
      u_emergency_stop.base = 0;
      u_emergency_stop.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->emergency_stop = u_emergency_stop.real;
      offset += sizeof(this->emergency_stop);
      union {
        bool real;
        uint8_t base;
      } u_safety_stop;
      u_safety_stop.base = 0;
      u_safety_stop.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->safety_stop = u_safety_stop.real;
      offset += sizeof(this->safety_stop);
      union {
        bool real;
        uint8_t base;
      } u_lasers_on_standby;
      u_lasers_on_standby.base = 0;
      u_lasers_on_standby.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->lasers_on_standby = u_lasers_on_standby.real;
      offset += sizeof(this->lasers_on_standby);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->current_speed));
      offset += this->lasers_mode.deserialize(inbuffer + offset);
      uint32_t lasers_status_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      lasers_status_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      lasers_status_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      lasers_status_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->lasers_status_length);
      if(lasers_status_lengthT > lasers_status_length)
        this->lasers_status = (robotnik_msgs::LaserStatus*)realloc(this->lasers_status, lasers_status_lengthT * sizeof(robotnik_msgs::LaserStatus));
      lasers_status_length = lasers_status_lengthT;
      for( uint32_t i = 0; i < lasers_status_length; i++){
      offset += this->st_lasers_status.deserialize(inbuffer + offset);
        memcpy( &(this->lasers_status[i]), &(this->st_lasers_status), sizeof(robotnik_msgs::LaserStatus));
      }
     return offset;
    }

    virtual const char * getType() override { return "robotnik_msgs/SafetyModuleStatus"; };
    virtual const char * getMD5() override { return "301f2f2b06f84bb7bbe649c08f21a4ff"; };

  };

}
#endif
