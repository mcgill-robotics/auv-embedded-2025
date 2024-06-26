#ifndef _ROS_auv_msgs_ThrusterForces_h
#define _ROS_auv_msgs_ThrusterForces_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace auv_msgs
{

  class ThrusterForces : public ros::Msg
  {
    public:
      typedef double _FRONT_LEFT_type;
      _FRONT_LEFT_type FRONT_LEFT;
      typedef double _FRONT_RIGHT_type;
      _FRONT_RIGHT_type FRONT_RIGHT;
      typedef double _BACK_LEFT_type;
      _BACK_LEFT_type BACK_LEFT;
      typedef double _BACK_RIGHT_type;
      _BACK_RIGHT_type BACK_RIGHT;
      typedef double _HEAVE_FRONT_LEFT_type;
      _HEAVE_FRONT_LEFT_type HEAVE_FRONT_LEFT;
      typedef double _HEAVE_FRONT_RIGHT_type;
      _HEAVE_FRONT_RIGHT_type HEAVE_FRONT_RIGHT;
      typedef double _HEAVE_BACK_LEFT_type;
      _HEAVE_BACK_LEFT_type HEAVE_BACK_LEFT;
      typedef double _HEAVE_BACK_RIGHT_type;
      _HEAVE_BACK_RIGHT_type HEAVE_BACK_RIGHT;

    ThrusterForces():
      FRONT_LEFT(0),
      FRONT_RIGHT(0),
      BACK_LEFT(0),
      BACK_RIGHT(0),
      HEAVE_FRONT_LEFT(0),
      HEAVE_FRONT_RIGHT(0),
      HEAVE_BACK_LEFT(0),
      HEAVE_BACK_RIGHT(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_FRONT_LEFT;
      u_FRONT_LEFT.real = this->FRONT_LEFT;
      *(outbuffer + offset + 0) = (u_FRONT_LEFT.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_FRONT_LEFT.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_FRONT_LEFT.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_FRONT_LEFT.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_FRONT_LEFT.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_FRONT_LEFT.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_FRONT_LEFT.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_FRONT_LEFT.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->FRONT_LEFT);
      union {
        double real;
        uint64_t base;
      } u_FRONT_RIGHT;
      u_FRONT_RIGHT.real = this->FRONT_RIGHT;
      *(outbuffer + offset + 0) = (u_FRONT_RIGHT.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_FRONT_RIGHT.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_FRONT_RIGHT.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_FRONT_RIGHT.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_FRONT_RIGHT.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_FRONT_RIGHT.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_FRONT_RIGHT.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_FRONT_RIGHT.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->FRONT_RIGHT);
      union {
        double real;
        uint64_t base;
      } u_BACK_LEFT;
      u_BACK_LEFT.real = this->BACK_LEFT;
      *(outbuffer + offset + 0) = (u_BACK_LEFT.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_BACK_LEFT.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_BACK_LEFT.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_BACK_LEFT.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_BACK_LEFT.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_BACK_LEFT.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_BACK_LEFT.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_BACK_LEFT.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->BACK_LEFT);
      union {
        double real;
        uint64_t base;
      } u_BACK_RIGHT;
      u_BACK_RIGHT.real = this->BACK_RIGHT;
      *(outbuffer + offset + 0) = (u_BACK_RIGHT.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_BACK_RIGHT.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_BACK_RIGHT.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_BACK_RIGHT.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_BACK_RIGHT.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_BACK_RIGHT.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_BACK_RIGHT.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_BACK_RIGHT.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->BACK_RIGHT);
      union {
        double real;
        uint64_t base;
      } u_HEAVE_FRONT_LEFT;
      u_HEAVE_FRONT_LEFT.real = this->HEAVE_FRONT_LEFT;
      *(outbuffer + offset + 0) = (u_HEAVE_FRONT_LEFT.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_HEAVE_FRONT_LEFT.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_HEAVE_FRONT_LEFT.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_HEAVE_FRONT_LEFT.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_HEAVE_FRONT_LEFT.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_HEAVE_FRONT_LEFT.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_HEAVE_FRONT_LEFT.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_HEAVE_FRONT_LEFT.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->HEAVE_FRONT_LEFT);
      union {
        double real;
        uint64_t base;
      } u_HEAVE_FRONT_RIGHT;
      u_HEAVE_FRONT_RIGHT.real = this->HEAVE_FRONT_RIGHT;
      *(outbuffer + offset + 0) = (u_HEAVE_FRONT_RIGHT.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_HEAVE_FRONT_RIGHT.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_HEAVE_FRONT_RIGHT.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_HEAVE_FRONT_RIGHT.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_HEAVE_FRONT_RIGHT.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_HEAVE_FRONT_RIGHT.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_HEAVE_FRONT_RIGHT.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_HEAVE_FRONT_RIGHT.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->HEAVE_FRONT_RIGHT);
      union {
        double real;
        uint64_t base;
      } u_HEAVE_BACK_LEFT;
      u_HEAVE_BACK_LEFT.real = this->HEAVE_BACK_LEFT;
      *(outbuffer + offset + 0) = (u_HEAVE_BACK_LEFT.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_HEAVE_BACK_LEFT.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_HEAVE_BACK_LEFT.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_HEAVE_BACK_LEFT.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_HEAVE_BACK_LEFT.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_HEAVE_BACK_LEFT.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_HEAVE_BACK_LEFT.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_HEAVE_BACK_LEFT.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->HEAVE_BACK_LEFT);
      union {
        double real;
        uint64_t base;
      } u_HEAVE_BACK_RIGHT;
      u_HEAVE_BACK_RIGHT.real = this->HEAVE_BACK_RIGHT;
      *(outbuffer + offset + 0) = (u_HEAVE_BACK_RIGHT.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_HEAVE_BACK_RIGHT.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_HEAVE_BACK_RIGHT.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_HEAVE_BACK_RIGHT.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_HEAVE_BACK_RIGHT.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_HEAVE_BACK_RIGHT.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_HEAVE_BACK_RIGHT.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_HEAVE_BACK_RIGHT.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->HEAVE_BACK_RIGHT);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_FRONT_LEFT;
      u_FRONT_LEFT.base = 0;
      u_FRONT_LEFT.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_FRONT_LEFT.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_FRONT_LEFT.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_FRONT_LEFT.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_FRONT_LEFT.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_FRONT_LEFT.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_FRONT_LEFT.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_FRONT_LEFT.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->FRONT_LEFT = u_FRONT_LEFT.real;
      offset += sizeof(this->FRONT_LEFT);
      union {
        double real;
        uint64_t base;
      } u_FRONT_RIGHT;
      u_FRONT_RIGHT.base = 0;
      u_FRONT_RIGHT.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_FRONT_RIGHT.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_FRONT_RIGHT.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_FRONT_RIGHT.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_FRONT_RIGHT.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_FRONT_RIGHT.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_FRONT_RIGHT.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_FRONT_RIGHT.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->FRONT_RIGHT = u_FRONT_RIGHT.real;
      offset += sizeof(this->FRONT_RIGHT);
      union {
        double real;
        uint64_t base;
      } u_BACK_LEFT;
      u_BACK_LEFT.base = 0;
      u_BACK_LEFT.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_BACK_LEFT.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_BACK_LEFT.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_BACK_LEFT.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_BACK_LEFT.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_BACK_LEFT.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_BACK_LEFT.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_BACK_LEFT.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->BACK_LEFT = u_BACK_LEFT.real;
      offset += sizeof(this->BACK_LEFT);
      union {
        double real;
        uint64_t base;
      } u_BACK_RIGHT;
      u_BACK_RIGHT.base = 0;
      u_BACK_RIGHT.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_BACK_RIGHT.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_BACK_RIGHT.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_BACK_RIGHT.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_BACK_RIGHT.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_BACK_RIGHT.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_BACK_RIGHT.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_BACK_RIGHT.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->BACK_RIGHT = u_BACK_RIGHT.real;
      offset += sizeof(this->BACK_RIGHT);
      union {
        double real;
        uint64_t base;
      } u_HEAVE_FRONT_LEFT;
      u_HEAVE_FRONT_LEFT.base = 0;
      u_HEAVE_FRONT_LEFT.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_HEAVE_FRONT_LEFT.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_HEAVE_FRONT_LEFT.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_HEAVE_FRONT_LEFT.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_HEAVE_FRONT_LEFT.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_HEAVE_FRONT_LEFT.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_HEAVE_FRONT_LEFT.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_HEAVE_FRONT_LEFT.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->HEAVE_FRONT_LEFT = u_HEAVE_FRONT_LEFT.real;
      offset += sizeof(this->HEAVE_FRONT_LEFT);
      union {
        double real;
        uint64_t base;
      } u_HEAVE_FRONT_RIGHT;
      u_HEAVE_FRONT_RIGHT.base = 0;
      u_HEAVE_FRONT_RIGHT.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_HEAVE_FRONT_RIGHT.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_HEAVE_FRONT_RIGHT.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_HEAVE_FRONT_RIGHT.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_HEAVE_FRONT_RIGHT.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_HEAVE_FRONT_RIGHT.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_HEAVE_FRONT_RIGHT.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_HEAVE_FRONT_RIGHT.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->HEAVE_FRONT_RIGHT = u_HEAVE_FRONT_RIGHT.real;
      offset += sizeof(this->HEAVE_FRONT_RIGHT);
      union {
        double real;
        uint64_t base;
      } u_HEAVE_BACK_LEFT;
      u_HEAVE_BACK_LEFT.base = 0;
      u_HEAVE_BACK_LEFT.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_HEAVE_BACK_LEFT.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_HEAVE_BACK_LEFT.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_HEAVE_BACK_LEFT.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_HEAVE_BACK_LEFT.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_HEAVE_BACK_LEFT.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_HEAVE_BACK_LEFT.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_HEAVE_BACK_LEFT.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->HEAVE_BACK_LEFT = u_HEAVE_BACK_LEFT.real;
      offset += sizeof(this->HEAVE_BACK_LEFT);
      union {
        double real;
        uint64_t base;
      } u_HEAVE_BACK_RIGHT;
      u_HEAVE_BACK_RIGHT.base = 0;
      u_HEAVE_BACK_RIGHT.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_HEAVE_BACK_RIGHT.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_HEAVE_BACK_RIGHT.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_HEAVE_BACK_RIGHT.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_HEAVE_BACK_RIGHT.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_HEAVE_BACK_RIGHT.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_HEAVE_BACK_RIGHT.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_HEAVE_BACK_RIGHT.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->HEAVE_BACK_RIGHT = u_HEAVE_BACK_RIGHT.real;
      offset += sizeof(this->HEAVE_BACK_RIGHT);
     return offset;
    }

    virtual const char * getType() override { return "auv_msgs/ThrusterForces"; };
    virtual const char * getMD5() override { return "cc67a5cdc84e81413d3e922ea55ecbed"; };

  };

}
#endif
