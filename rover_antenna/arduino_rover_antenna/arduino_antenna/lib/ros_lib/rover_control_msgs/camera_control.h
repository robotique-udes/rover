#ifndef _ROS_SERVICE_camera_control_h
#define _ROS_SERVICE_camera_control_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rover_control_msgs
{

static const char CAMERA_CONTROL[] = "rover_control_msgs/camera_control";

  class camera_controlRequest : public ros::Msg
  {
    public:
      typedef uint8_t _cmd_type;
      _cmd_type cmd;
      typedef const char* _cam_topic_type;
      _cam_topic_type cam_topic;
      enum { CMD_TAKE_PICTURE =  1 };
      enum { CMD_SAVE_PANO =  2 };
      enum { CMD_RESET_PICTURES =  3 };
      enum { CMD_DETECT_ARUCO =  4 };
      enum { CMD_TAKE_AND_SAVE_PANO =  5 };

    camera_controlRequest():
      cmd(0),
      cam_topic("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->cmd >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cmd);
      uint32_t length_cam_topic = strlen(this->cam_topic);
      varToArr(outbuffer + offset, length_cam_topic);
      offset += 4;
      memcpy(outbuffer + offset, this->cam_topic, length_cam_topic);
      offset += length_cam_topic;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->cmd =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->cmd);
      uint32_t length_cam_topic;
      arrToVar(length_cam_topic, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_cam_topic; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_cam_topic-1]=0;
      this->cam_topic = (char *)(inbuffer + offset-1);
      offset += length_cam_topic;
     return offset;
    }

    virtual const char * getType() override { return CAMERA_CONTROL; };
    virtual const char * getMD5() override { return "9e7a6405b8c5949352af17307c74188a"; };

  };

  class camera_controlResponse : public ros::Msg
  {
    public:
      typedef bool _result_type;
      _result_type result;
      uint32_t detected_aruco_marker_length;
      typedef uint8_t _detected_aruco_marker_type;
      _detected_aruco_marker_type st_detected_aruco_marker;
      _detected_aruco_marker_type * detected_aruco_marker;

    camera_controlResponse():
      result(0),
      detected_aruco_marker_length(0), st_detected_aruco_marker(), detected_aruco_marker(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_result;
      u_result.real = this->result;
      *(outbuffer + offset + 0) = (u_result.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->result);
      *(outbuffer + offset + 0) = (this->detected_aruco_marker_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->detected_aruco_marker_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->detected_aruco_marker_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->detected_aruco_marker_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->detected_aruco_marker_length);
      for( uint32_t i = 0; i < detected_aruco_marker_length; i++){
      *(outbuffer + offset + 0) = (this->detected_aruco_marker[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->detected_aruco_marker[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_result;
      u_result.base = 0;
      u_result.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->result = u_result.real;
      offset += sizeof(this->result);
      uint32_t detected_aruco_marker_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      detected_aruco_marker_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      detected_aruco_marker_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      detected_aruco_marker_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->detected_aruco_marker_length);
      if(detected_aruco_marker_lengthT > detected_aruco_marker_length)
        this->detected_aruco_marker = (uint8_t*)realloc(this->detected_aruco_marker, detected_aruco_marker_lengthT * sizeof(uint8_t));
      detected_aruco_marker_length = detected_aruco_marker_lengthT;
      for( uint32_t i = 0; i < detected_aruco_marker_length; i++){
      this->st_detected_aruco_marker =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_detected_aruco_marker);
        memcpy( &(this->detected_aruco_marker[i]), &(this->st_detected_aruco_marker), sizeof(uint8_t));
      }
     return offset;
    }

    virtual const char * getType() override { return CAMERA_CONTROL; };
    virtual const char * getMD5() override { return "6046235aa9debf19a352ec5dcf2097a5"; };

  };

  class camera_control {
    public:
    typedef camera_controlRequest Request;
    typedef camera_controlResponse Response;
  };

}
#endif
