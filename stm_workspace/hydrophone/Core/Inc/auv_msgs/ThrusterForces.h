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
      typedef double _SURGE_PORT_type;
      _SURGE_PORT_type SURGE_PORT;
      typedef double _SURGE_STAR_type;
      _SURGE_STAR_type SURGE_STAR;
      typedef double _SWAY_BOW_type;
      _SWAY_BOW_type SWAY_BOW;
      typedef double _SWAY_STERN_type;
      _SWAY_STERN_type SWAY_STERN;
      typedef double _HEAVE_BOW_PORT_type;
      _HEAVE_BOW_PORT_type HEAVE_BOW_PORT;
      typedef double _HEAVE_BOW_STAR_type;
      _HEAVE_BOW_STAR_type HEAVE_BOW_STAR;
      typedef double _HEAVE_STERN_STAR_type;
      _HEAVE_STERN_STAR_type HEAVE_STERN_STAR;
      typedef double _HEAVE_STERN_PORT_type;
      _HEAVE_STERN_PORT_type HEAVE_STERN_PORT;

    ThrusterForces():
      SURGE_PORT(0),
      SURGE_STAR(0),
      SWAY_BOW(0),
      SWAY_STERN(0),
      HEAVE_BOW_PORT(0),
      HEAVE_BOW_STAR(0),
      HEAVE_STERN_STAR(0),
      HEAVE_STERN_PORT(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_SURGE_PORT;
      u_SURGE_PORT.real = this->SURGE_PORT;
      *(outbuffer + offset + 0) = (u_SURGE_PORT.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_SURGE_PORT.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_SURGE_PORT.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_SURGE_PORT.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_SURGE_PORT.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_SURGE_PORT.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_SURGE_PORT.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_SURGE_PORT.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->SURGE_PORT);
      union {
        double real;
        uint64_t base;
      } u_SURGE_STAR;
      u_SURGE_STAR.real = this->SURGE_STAR;
      *(outbuffer + offset + 0) = (u_SURGE_STAR.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_SURGE_STAR.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_SURGE_STAR.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_SURGE_STAR.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_SURGE_STAR.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_SURGE_STAR.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_SURGE_STAR.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_SURGE_STAR.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->SURGE_STAR);
      union {
        double real;
        uint64_t base;
      } u_SWAY_BOW;
      u_SWAY_BOW.real = this->SWAY_BOW;
      *(outbuffer + offset + 0) = (u_SWAY_BOW.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_SWAY_BOW.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_SWAY_BOW.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_SWAY_BOW.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_SWAY_BOW.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_SWAY_BOW.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_SWAY_BOW.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_SWAY_BOW.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->SWAY_BOW);
      union {
        double real;
        uint64_t base;
      } u_SWAY_STERN;
      u_SWAY_STERN.real = this->SWAY_STERN;
      *(outbuffer + offset + 0) = (u_SWAY_STERN.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_SWAY_STERN.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_SWAY_STERN.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_SWAY_STERN.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_SWAY_STERN.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_SWAY_STERN.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_SWAY_STERN.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_SWAY_STERN.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->SWAY_STERN);
      union {
        double real;
        uint64_t base;
      } u_HEAVE_BOW_PORT;
      u_HEAVE_BOW_PORT.real = this->HEAVE_BOW_PORT;
      *(outbuffer + offset + 0) = (u_HEAVE_BOW_PORT.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_HEAVE_BOW_PORT.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_HEAVE_BOW_PORT.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_HEAVE_BOW_PORT.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_HEAVE_BOW_PORT.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_HEAVE_BOW_PORT.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_HEAVE_BOW_PORT.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_HEAVE_BOW_PORT.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->HEAVE_BOW_PORT);
      union {
        double real;
        uint64_t base;
      } u_HEAVE_BOW_STAR;
      u_HEAVE_BOW_STAR.real = this->HEAVE_BOW_STAR;
      *(outbuffer + offset + 0) = (u_HEAVE_BOW_STAR.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_HEAVE_BOW_STAR.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_HEAVE_BOW_STAR.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_HEAVE_BOW_STAR.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_HEAVE_BOW_STAR.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_HEAVE_BOW_STAR.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_HEAVE_BOW_STAR.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_HEAVE_BOW_STAR.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->HEAVE_BOW_STAR);
      union {
        double real;
        uint64_t base;
      } u_HEAVE_STERN_STAR;
      u_HEAVE_STERN_STAR.real = this->HEAVE_STERN_STAR;
      *(outbuffer + offset + 0) = (u_HEAVE_STERN_STAR.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_HEAVE_STERN_STAR.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_HEAVE_STERN_STAR.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_HEAVE_STERN_STAR.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_HEAVE_STERN_STAR.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_HEAVE_STERN_STAR.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_HEAVE_STERN_STAR.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_HEAVE_STERN_STAR.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->HEAVE_STERN_STAR);
      union {
        double real;
        uint64_t base;
      } u_HEAVE_STERN_PORT;
      u_HEAVE_STERN_PORT.real = this->HEAVE_STERN_PORT;
      *(outbuffer + offset + 0) = (u_HEAVE_STERN_PORT.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_HEAVE_STERN_PORT.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_HEAVE_STERN_PORT.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_HEAVE_STERN_PORT.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_HEAVE_STERN_PORT.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_HEAVE_STERN_PORT.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_HEAVE_STERN_PORT.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_HEAVE_STERN_PORT.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->HEAVE_STERN_PORT);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_SURGE_PORT;
      u_SURGE_PORT.base = 0;
      u_SURGE_PORT.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_SURGE_PORT.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_SURGE_PORT.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_SURGE_PORT.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_SURGE_PORT.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_SURGE_PORT.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_SURGE_PORT.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_SURGE_PORT.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->SURGE_PORT = u_SURGE_PORT.real;
      offset += sizeof(this->SURGE_PORT);
      union {
        double real;
        uint64_t base;
      } u_SURGE_STAR;
      u_SURGE_STAR.base = 0;
      u_SURGE_STAR.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_SURGE_STAR.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_SURGE_STAR.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_SURGE_STAR.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_SURGE_STAR.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_SURGE_STAR.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_SURGE_STAR.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_SURGE_STAR.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->SURGE_STAR = u_SURGE_STAR.real;
      offset += sizeof(this->SURGE_STAR);
      union {
        double real;
        uint64_t base;
      } u_SWAY_BOW;
      u_SWAY_BOW.base = 0;
      u_SWAY_BOW.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_SWAY_BOW.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_SWAY_BOW.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_SWAY_BOW.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_SWAY_BOW.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_SWAY_BOW.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_SWAY_BOW.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_SWAY_BOW.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->SWAY_BOW = u_SWAY_BOW.real;
      offset += sizeof(this->SWAY_BOW);
      union {
        double real;
        uint64_t base;
      } u_SWAY_STERN;
      u_SWAY_STERN.base = 0;
      u_SWAY_STERN.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_SWAY_STERN.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_SWAY_STERN.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_SWAY_STERN.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_SWAY_STERN.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_SWAY_STERN.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_SWAY_STERN.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_SWAY_STERN.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->SWAY_STERN = u_SWAY_STERN.real;
      offset += sizeof(this->SWAY_STERN);
      union {
        double real;
        uint64_t base;
      } u_HEAVE_BOW_PORT;
      u_HEAVE_BOW_PORT.base = 0;
      u_HEAVE_BOW_PORT.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_HEAVE_BOW_PORT.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_HEAVE_BOW_PORT.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_HEAVE_BOW_PORT.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_HEAVE_BOW_PORT.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_HEAVE_BOW_PORT.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_HEAVE_BOW_PORT.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_HEAVE_BOW_PORT.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->HEAVE_BOW_PORT = u_HEAVE_BOW_PORT.real;
      offset += sizeof(this->HEAVE_BOW_PORT);
      union {
        double real;
        uint64_t base;
      } u_HEAVE_BOW_STAR;
      u_HEAVE_BOW_STAR.base = 0;
      u_HEAVE_BOW_STAR.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_HEAVE_BOW_STAR.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_HEAVE_BOW_STAR.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_HEAVE_BOW_STAR.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_HEAVE_BOW_STAR.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_HEAVE_BOW_STAR.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_HEAVE_BOW_STAR.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_HEAVE_BOW_STAR.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->HEAVE_BOW_STAR = u_HEAVE_BOW_STAR.real;
      offset += sizeof(this->HEAVE_BOW_STAR);
      union {
        double real;
        uint64_t base;
      } u_HEAVE_STERN_STAR;
      u_HEAVE_STERN_STAR.base = 0;
      u_HEAVE_STERN_STAR.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_HEAVE_STERN_STAR.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_HEAVE_STERN_STAR.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_HEAVE_STERN_STAR.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_HEAVE_STERN_STAR.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_HEAVE_STERN_STAR.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_HEAVE_STERN_STAR.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_HEAVE_STERN_STAR.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->HEAVE_STERN_STAR = u_HEAVE_STERN_STAR.real;
      offset += sizeof(this->HEAVE_STERN_STAR);
      union {
        double real;
        uint64_t base;
      } u_HEAVE_STERN_PORT;
      u_HEAVE_STERN_PORT.base = 0;
      u_HEAVE_STERN_PORT.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_HEAVE_STERN_PORT.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_HEAVE_STERN_PORT.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_HEAVE_STERN_PORT.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_HEAVE_STERN_PORT.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_HEAVE_STERN_PORT.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_HEAVE_STERN_PORT.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_HEAVE_STERN_PORT.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->HEAVE_STERN_PORT = u_HEAVE_STERN_PORT.real;
      offset += sizeof(this->HEAVE_STERN_PORT);
     return offset;
    }

    virtual const char * getType() override { return "auv_msgs/ThrusterForces"; };
    virtual const char * getMD5() override { return "56105fa5690ee1db02111af806c88d1a"; };

  };

}
#endif
