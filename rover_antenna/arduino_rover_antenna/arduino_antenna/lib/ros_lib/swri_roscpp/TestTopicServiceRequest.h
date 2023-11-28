#ifndef _ROS_swri_roscpp_TestTopicServiceRequest_h
#define _ROS_swri_roscpp_TestTopicServiceRequest_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "marti_common_msgs/ServiceHeader.h"

namespace swri_roscpp
{

  class TestTopicServiceRequest : public ros::Msg
  {
    public:
      typedef marti_common_msgs::ServiceHeader _srv_header_type;
      _srv_header_type srv_header;
      typedef int32_t _request_value_type;
      _request_value_type request_value;

    TestTopicServiceRequest():
      srv_header(),
      request_value(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->srv_header.serialize(outbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_request_value;
      u_request_value.real = this->request_value;
      *(outbuffer + offset + 0) = (u_request_value.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_request_value.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_request_value.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_request_value.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->request_value);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->srv_header.deserialize(inbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_request_value;
      u_request_value.base = 0;
      u_request_value.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_request_value.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_request_value.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_request_value.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->request_value = u_request_value.real;
      offset += sizeof(this->request_value);
     return offset;
    }

    virtual const char * getType() override { return "swri_roscpp/TestTopicServiceRequest"; };
    virtual const char * getMD5() override { return "62086d0669b9627ef32b14cac1955c4c"; };

  };

}
#endif
