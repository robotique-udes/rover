#ifndef _ROS_SERVICE_set_modbus_register_bit_h
#define _ROS_SERVICE_set_modbus_register_bit_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robotnik_msgs
{

static const char SET_MODBUS_REGISTER_BIT[] = "robotnik_msgs/set_modbus_register_bit";

  class set_modbus_register_bitRequest : public ros::Msg
  {
    public:
      typedef int16_t _address_type;
      _address_type address;
      typedef int16_t _bit_type;
      _bit_type bit;
      typedef bool _value_type;
      _value_type value;

    set_modbus_register_bitRequest():
      address(0),
      bit(0),
      value(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_address;
      u_address.real = this->address;
      *(outbuffer + offset + 0) = (u_address.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_address.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->address);
      union {
        int16_t real;
        uint16_t base;
      } u_bit;
      u_bit.real = this->bit;
      *(outbuffer + offset + 0) = (u_bit.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_bit.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->bit);
      union {
        bool real;
        uint8_t base;
      } u_value;
      u_value.real = this->value;
      *(outbuffer + offset + 0) = (u_value.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->value);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_address;
      u_address.base = 0;
      u_address.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_address.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->address = u_address.real;
      offset += sizeof(this->address);
      union {
        int16_t real;
        uint16_t base;
      } u_bit;
      u_bit.base = 0;
      u_bit.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_bit.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->bit = u_bit.real;
      offset += sizeof(this->bit);
      union {
        bool real;
        uint8_t base;
      } u_value;
      u_value.base = 0;
      u_value.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->value = u_value.real;
      offset += sizeof(this->value);
     return offset;
    }

    virtual const char * getType() override { return SET_MODBUS_REGISTER_BIT; };
    virtual const char * getMD5() override { return "802ee9818bd86ff018a6b6838f90a10a"; };

  };

  class set_modbus_register_bitResponse : public ros::Msg
  {
    public:
      typedef bool _ret_type;
      _ret_type ret;

    set_modbus_register_bitResponse():
      ret(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_ret;
      u_ret.real = this->ret;
      *(outbuffer + offset + 0) = (u_ret.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ret);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_ret;
      u_ret.base = 0;
      u_ret.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ret = u_ret.real;
      offset += sizeof(this->ret);
     return offset;
    }

    virtual const char * getType() override { return SET_MODBUS_REGISTER_BIT; };
    virtual const char * getMD5() override { return "e2cc9e9d8c464550830df49c160979ad"; };

  };

  class set_modbus_register_bit {
    public:
    typedef set_modbus_register_bitRequest Request;
    typedef set_modbus_register_bitResponse Response;
  };

}
#endif
