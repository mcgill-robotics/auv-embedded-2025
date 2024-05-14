#ifndef _ROS_auv_msgs_ThrusterMicroseconds_h
#define _ROS_auv_msgs_ThrusterMicroseconds_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace auv_msgs
{

  class ThrusterMicroseconds : public ros::Msg
  {
    public:
      uint16_t microseconds[8];
      enum { SURGE_PORT = 0 };
      enum { SURGE_STAR = 1 };
      enum { SWAY_BOW = 2 };
      enum { SWAY_STERN = 3 };
      enum { HEAVE_BOW_PORT = 4 };
      enum { HEAVE_BOW_STAR = 5 };
      enum { HEAVE_STERN_STAR = 6 };
      enum { HEAVE_STERN_PORT = 7 };

    ThrusterMicroseconds():
      microseconds()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 8; i++){
      *(outbuffer + offset + 0) = (this->microseconds[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->microseconds[i] >> (8 * 1)) & 0xFF;
      offset += sizeof(this->microseconds[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 8; i++){
      this->microseconds[i] =  ((uint16_t) (*(inbuffer + offset)));
      this->microseconds[i] |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->microseconds[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return "auv_msgs/ThrusterMicroseconds"; };
    virtual const char * getMD5() override { return "da54d5f5b49d52537ff663ce87808105"; };

  };

}
#endif
