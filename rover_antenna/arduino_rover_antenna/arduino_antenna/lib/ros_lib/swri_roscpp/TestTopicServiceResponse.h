#ifndef _ROS_swri_roscpp_TestTopicServiceResponse_h
#define _ROS_swri_roscpp_TestTopicServiceResponse_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "marti_common_msgs/ServiceHeader.h"

namespace swri_roscpp
{

  class TestTopicServiceResponse : public ros::Msg
  {
    public:
      typedef marti_common_msgs::ServiceHeader _srv_header_type;
      _srv_header_type srv_header;
      typedef int32_t _response_value_type;
      _response_value_type response_value;

    TestTopicServiceResponse():
      srv_header(),
      response_value(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->srv_header.serialize(outbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_response_value;
      u_response_value.real = this->response_value;
      *(outbuffer + offset + 0) = (u_response_value.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_response_value.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_response_value.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_response_value.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->response_value);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->srv_header.deserialize(inbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_response_value;
      u_response_value.base = 0;
      u_response_value.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_response_value.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_response_value.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_response_value.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->response_value = u_response_value.real;
      offset += sizeof(this->response_value);
     return offset;
    }

    virtual const char * getType() override { return "swri_roscpp/TestTopicServiceResponse"; };
    virtual const char * getMD5() override { return "ffa572c724c22095570f217a11ef9386"; };

  };

}
#endif
