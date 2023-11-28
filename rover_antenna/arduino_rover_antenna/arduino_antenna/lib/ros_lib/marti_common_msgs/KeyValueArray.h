#ifndef _ROS_marti_common_msgs_KeyValueArray_h
#define _ROS_marti_common_msgs_KeyValueArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "marti_common_msgs/KeyValue.h"

namespace marti_common_msgs
{

  class KeyValueArray : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t items_length;
      typedef marti_common_msgs::KeyValue _items_type;
      _items_type st_items;
      _items_type * items;

    KeyValueArray():
      header(),
      items_length(0), st_items(), items(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->items_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->items_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->items_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->items_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->items_length);
      for( uint32_t i = 0; i < items_length; i++){
      offset += this->items[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t items_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      items_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      items_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      items_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->items_length);
      if(items_lengthT > items_length)
        this->items = (marti_common_msgs::KeyValue*)realloc(this->items, items_lengthT * sizeof(marti_common_msgs::KeyValue));
      items_length = items_lengthT;
      for( uint32_t i = 0; i < items_length; i++){
      offset += this->st_items.deserialize(inbuffer + offset);
        memcpy( &(this->items[i]), &(this->st_items), sizeof(marti_common_msgs::KeyValue));
      }
     return offset;
    }

    virtual const char * getType() override { return "marti_common_msgs/KeyValueArray"; };
    virtual const char * getMD5() override { return "3b303032d5c2c08f75f9a40a839cb16c"; };

  };

}
#endif
