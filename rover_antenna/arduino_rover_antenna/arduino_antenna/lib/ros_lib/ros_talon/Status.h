#ifndef _ROS_ros_talon_Status_h
#define _ROS_ros_talon_Status_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace ros_talon
{

  class Status : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _Temperature_type;
      _Temperature_type Temperature;
      typedef float _OutputCurrent_type;
      _OutputCurrent_type OutputCurrent;
      typedef float _BusVoltage_type;
      _BusVoltage_type BusVoltage;
      typedef float _ClosedLoopError_type;
      _ClosedLoopError_type ClosedLoopError;

    Status():
      header(),
      Temperature(0),
      OutputCurrent(0),
      BusVoltage(0),
      ClosedLoopError(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_Temperature;
      u_Temperature.real = this->Temperature;
      *(outbuffer + offset + 0) = (u_Temperature.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Temperature.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Temperature.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Temperature.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Temperature);
      union {
        float real;
        uint32_t base;
      } u_OutputCurrent;
      u_OutputCurrent.real = this->OutputCurrent;
      *(outbuffer + offset + 0) = (u_OutputCurrent.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_OutputCurrent.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_OutputCurrent.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_OutputCurrent.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->OutputCurrent);
      union {
        float real;
        uint32_t base;
      } u_BusVoltage;
      u_BusVoltage.real = this->BusVoltage;
      *(outbuffer + offset + 0) = (u_BusVoltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_BusVoltage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_BusVoltage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_BusVoltage.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->BusVoltage);
      union {
        float real;
        uint32_t base;
      } u_ClosedLoopError;
      u_ClosedLoopError.real = this->ClosedLoopError;
      *(outbuffer + offset + 0) = (u_ClosedLoopError.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ClosedLoopError.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ClosedLoopError.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ClosedLoopError.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ClosedLoopError);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_Temperature;
      u_Temperature.base = 0;
      u_Temperature.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Temperature.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Temperature.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Temperature.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Temperature = u_Temperature.real;
      offset += sizeof(this->Temperature);
      union {
        float real;
        uint32_t base;
      } u_OutputCurrent;
      u_OutputCurrent.base = 0;
      u_OutputCurrent.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_OutputCurrent.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_OutputCurrent.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_OutputCurrent.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->OutputCurrent = u_OutputCurrent.real;
      offset += sizeof(this->OutputCurrent);
      union {
        float real;
        uint32_t base;
      } u_BusVoltage;
      u_BusVoltage.base = 0;
      u_BusVoltage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_BusVoltage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_BusVoltage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_BusVoltage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->BusVoltage = u_BusVoltage.real;
      offset += sizeof(this->BusVoltage);
      union {
        float real;
        uint32_t base;
      } u_ClosedLoopError;
      u_ClosedLoopError.base = 0;
      u_ClosedLoopError.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ClosedLoopError.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ClosedLoopError.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ClosedLoopError.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ClosedLoopError = u_ClosedLoopError.real;
      offset += sizeof(this->ClosedLoopError);
     return offset;
    }

    virtual const char * getType() override { return "ros_talon/Status"; };
    virtual const char * getMD5() override { return "c9698fbd092b40146e5f4c615f4ec677"; };

  };

}
#endif
