#ifndef _ROS_rly_08_rly08_io_msg_h
#define _ROS_rly_08_rly08_io_msg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rly_08
{

  class rly08_io_msg : public ros::Msg
  {
    public:
      uint32_t relay_status_length;
      typedef uint32_t _relay_status_type;
      _relay_status_type st_relay_status;
      _relay_status_type * relay_status;

    rly08_io_msg():
      relay_status_length(0), st_relay_status(), relay_status(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->relay_status_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->relay_status_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->relay_status_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->relay_status_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->relay_status_length);
      for( uint32_t i = 0; i < relay_status_length; i++){
      *(outbuffer + offset + 0) = (this->relay_status[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->relay_status[i] >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->relay_status[i] >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->relay_status[i] >> (8 * 3)) & 0xFF;
      offset += sizeof(this->relay_status[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t relay_status_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      relay_status_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      relay_status_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      relay_status_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->relay_status_length);
      if(relay_status_lengthT > relay_status_length)
        this->relay_status = (uint32_t*)realloc(this->relay_status, relay_status_lengthT * sizeof(uint32_t));
      relay_status_length = relay_status_lengthT;
      for( uint32_t i = 0; i < relay_status_length; i++){
      this->st_relay_status =  ((uint32_t) (*(inbuffer + offset)));
      this->st_relay_status |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_relay_status |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->st_relay_status |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->st_relay_status);
        memcpy( &(this->relay_status[i]), &(this->st_relay_status), sizeof(uint32_t));
      }
     return offset;
    }

    virtual const char * getType() override { return "rly_08/rly08_io_msg"; };
    virtual const char * getMD5() override { return "1ed39fcc8050fe5a3fbbe38b86570362"; };

  };

}
#endif
