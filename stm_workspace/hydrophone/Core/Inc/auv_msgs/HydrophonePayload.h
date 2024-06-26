#ifndef _ROS_auv_msgs_HydrophonePayload_h
#define _ROS_auv_msgs_HydrophonePayload_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace auv_msgs
{

  class HydrophonePayload : public ros::Msg
  {
    public:
      typedef uint8_t _hydrophone_type;
      _hydrophone_type hydrophone;
      typedef uint32_t _frequency_type;
      _frequency_type frequency;
      typedef uint32_t _time_type;
      _time_type time;

    HydrophonePayload():
      hydrophone(0),
      frequency(0),
      time(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->hydrophone >> (8 * 0)) & 0xFF;
      offset += sizeof(this->hydrophone);
      *(outbuffer + offset + 0) = (this->frequency >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->frequency >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->frequency >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->frequency >> (8 * 3)) & 0xFF;
      offset += sizeof(this->frequency);
      *(outbuffer + offset + 0) = (this->time >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->time >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->time >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->time >> (8 * 3)) & 0xFF;
      offset += sizeof(this->time);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->hydrophone =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->hydrophone);
      this->frequency =  ((uint32_t) (*(inbuffer + offset)));
      this->frequency |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->frequency |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->frequency |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->frequency);
      this->time =  ((uint32_t) (*(inbuffer + offset)));
      this->time |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->time |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->time |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->time);
     return offset;
    }

    virtual const char * getType() override { return "auv_msgs/HydrophonePayload"; };
    virtual const char * getMD5() override { return "b1876360f6462dc4928877125c2982d9"; };

  };

}
#endif
