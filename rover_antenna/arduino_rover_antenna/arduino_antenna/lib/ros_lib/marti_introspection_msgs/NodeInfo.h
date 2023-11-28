#ifndef _ROS_marti_introspection_msgs_NodeInfo_h
#define _ROS_marti_introspection_msgs_NodeInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "marti_introspection_msgs/TopicInfo.h"
#include "marti_introspection_msgs/ParamInfo.h"
#include "marti_introspection_msgs/ServiceInfo.h"

namespace marti_introspection_msgs
{

  class NodeInfo : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      typedef const char* _location_type;
      _location_type location;
      typedef const char* _nodelet_manager_type;
      _nodelet_manager_type nodelet_manager;
      typedef const char* _description_type;
      _description_type description;
      uint32_t topics_length;
      typedef marti_introspection_msgs::TopicInfo _topics_type;
      _topics_type st_topics;
      _topics_type * topics;
      uint32_t parameters_length;
      typedef marti_introspection_msgs::ParamInfo _parameters_type;
      _parameters_type st_parameters;
      _parameters_type * parameters;
      uint32_t services_length;
      typedef marti_introspection_msgs::ServiceInfo _services_type;
      _services_type st_services;
      _services_type * services;

    NodeInfo():
      name(""),
      location(""),
      nodelet_manager(""),
      description(""),
      topics_length(0), st_topics(), topics(nullptr),
      parameters_length(0), st_parameters(), parameters(nullptr),
      services_length(0), st_services(), services(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      uint32_t length_location = strlen(this->location);
      varToArr(outbuffer + offset, length_location);
      offset += 4;
      memcpy(outbuffer + offset, this->location, length_location);
      offset += length_location;
      uint32_t length_nodelet_manager = strlen(this->nodelet_manager);
      varToArr(outbuffer + offset, length_nodelet_manager);
      offset += 4;
      memcpy(outbuffer + offset, this->nodelet_manager, length_nodelet_manager);
      offset += length_nodelet_manager;
      uint32_t length_description = strlen(this->description);
      varToArr(outbuffer + offset, length_description);
      offset += 4;
      memcpy(outbuffer + offset, this->description, length_description);
      offset += length_description;
      *(outbuffer + offset + 0) = (this->topics_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->topics_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->topics_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->topics_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->topics_length);
      for( uint32_t i = 0; i < topics_length; i++){
      offset += this->topics[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->parameters_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->parameters_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->parameters_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->parameters_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->parameters_length);
      for( uint32_t i = 0; i < parameters_length; i++){
      offset += this->parameters[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->services_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->services_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->services_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->services_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->services_length);
      for( uint32_t i = 0; i < services_length; i++){
      offset += this->services[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      uint32_t length_location;
      arrToVar(length_location, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_location; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_location-1]=0;
      this->location = (char *)(inbuffer + offset-1);
      offset += length_location;
      uint32_t length_nodelet_manager;
      arrToVar(length_nodelet_manager, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_nodelet_manager; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_nodelet_manager-1]=0;
      this->nodelet_manager = (char *)(inbuffer + offset-1);
      offset += length_nodelet_manager;
      uint32_t length_description;
      arrToVar(length_description, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_description; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_description-1]=0;
      this->description = (char *)(inbuffer + offset-1);
      offset += length_description;
      uint32_t topics_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      topics_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      topics_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      topics_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->topics_length);
      if(topics_lengthT > topics_length)
        this->topics = (marti_introspection_msgs::TopicInfo*)realloc(this->topics, topics_lengthT * sizeof(marti_introspection_msgs::TopicInfo));
      topics_length = topics_lengthT;
      for( uint32_t i = 0; i < topics_length; i++){
      offset += this->st_topics.deserialize(inbuffer + offset);
        memcpy( &(this->topics[i]), &(this->st_topics), sizeof(marti_introspection_msgs::TopicInfo));
      }
      uint32_t parameters_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      parameters_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      parameters_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      parameters_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->parameters_length);
      if(parameters_lengthT > parameters_length)
        this->parameters = (marti_introspection_msgs::ParamInfo*)realloc(this->parameters, parameters_lengthT * sizeof(marti_introspection_msgs::ParamInfo));
      parameters_length = parameters_lengthT;
      for( uint32_t i = 0; i < parameters_length; i++){
      offset += this->st_parameters.deserialize(inbuffer + offset);
        memcpy( &(this->parameters[i]), &(this->st_parameters), sizeof(marti_introspection_msgs::ParamInfo));
      }
      uint32_t services_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      services_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      services_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      services_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->services_length);
      if(services_lengthT > services_length)
        this->services = (marti_introspection_msgs::ServiceInfo*)realloc(this->services, services_lengthT * sizeof(marti_introspection_msgs::ServiceInfo));
      services_length = services_lengthT;
      for( uint32_t i = 0; i < services_length; i++){
      offset += this->st_services.deserialize(inbuffer + offset);
        memcpy( &(this->services[i]), &(this->st_services), sizeof(marti_introspection_msgs::ServiceInfo));
      }
     return offset;
    }

    virtual const char * getType() override { return "marti_introspection_msgs/NodeInfo"; };
    virtual const char * getMD5() override { return "800cb712abf3e0f84bf282d84b0cf195"; };

  };

}
#endif
