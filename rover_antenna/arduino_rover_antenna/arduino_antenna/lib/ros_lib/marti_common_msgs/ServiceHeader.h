#ifndef _ROS_marti_common_msgs_ServiceHeader_h
#define _ROS_marti_common_msgs_ServiceHeader_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace marti_common_msgs
{

  class ServiceHeader : public ros::Msg
  {
    public:
      typedef ros::Time _stamp_type;
      _stamp_type stamp;
      typedef uint32_t _sequence_type;
      _sequence_type sequence;
      typedef const char* _sender_type;
      _sender_type sender;
      typedef bool _result_type;
      _result_type result;

    ServiceHeader():
      stamp(),
      sequence(0),
      sender(""),
      result(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->stamp.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.sec);
      *(outbuffer + offset + 0) = (this->stamp.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.nsec);
      *(outbuffer + offset + 0) = (this->sequence >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sequence >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sequence >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sequence >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sequence);
      uint32_t length_sender = strlen(this->sender);
      varToArr(outbuffer + offset, length_sender);
      offset += 4;
      memcpy(outbuffer + offset, this->sender, length_sender);
      offset += length_sender;
      union {
        bool real;
        uint8_t base;
      } u_result;
      u_result.real = this->result;
      *(outbuffer + offset + 0) = (u_result.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->result);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->stamp.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.sec);
      this->stamp.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.nsec);
      this->sequence =  ((uint32_t) (*(inbuffer + offset)));
      this->sequence |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->sequence |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->sequence |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->sequence);
      uint32_t length_sender;
      arrToVar(length_sender, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_sender; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_sender-1]=0;
      this->sender = (char *)(inbuffer + offset-1);
      offset += length_sender;
      union {
        bool real;
        uint8_t base;
      } u_result;
      u_result.base = 0;
      u_result.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->result = u_result.real;
      offset += sizeof(this->result);
     return offset;
    }

    virtual const char * getType() override { return "marti_common_msgs/ServiceHeader"; };
    virtual const char * getMD5() override { return "c9ecea07422007ad3a23c820ab38111a"; };

  };

}
#endif
