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
      typedef float _SURGE_PORT_type;
      _SURGE_PORT_type SURGE_PORT;
      typedef float _SURGE_STAR_type;
      _SURGE_STAR_type SURGE_STAR;
      typedef float _SWAY_BOW_type;
      _SWAY_BOW_type SWAY_BOW;
      typedef float _SWAY_STERN_type;
      _SWAY_STERN_type SWAY_STERN;
      typedef float _HEAVE_BOW_PORT_type;
      _HEAVE_BOW_PORT_type HEAVE_BOW_PORT;
      typedef float _HEAVE_BOW_STAR_type;
      _HEAVE_BOW_STAR_type HEAVE_BOW_STAR;
      typedef float _HEAVE_STERN_STAR_type;
      _HEAVE_STERN_STAR_type HEAVE_STERN_STAR;
      typedef float _HEAVE_STERN_PORT_type;
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
      offset += serializeAvrFloat64(outbuffer + offset, this->SURGE_PORT);
      offset += serializeAvrFloat64(outbuffer + offset, this->SURGE_STAR);
      offset += serializeAvrFloat64(outbuffer + offset, this->SWAY_BOW);
      offset += serializeAvrFloat64(outbuffer + offset, this->SWAY_STERN);
      offset += serializeAvrFloat64(outbuffer + offset, this->HEAVE_BOW_PORT);
      offset += serializeAvrFloat64(outbuffer + offset, this->HEAVE_BOW_STAR);
      offset += serializeAvrFloat64(outbuffer + offset, this->HEAVE_STERN_STAR);
      offset += serializeAvrFloat64(outbuffer + offset, this->HEAVE_STERN_PORT);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->SURGE_PORT));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->SURGE_STAR));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->SWAY_BOW));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->SWAY_STERN));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->HEAVE_BOW_PORT));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->HEAVE_BOW_STAR));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->HEAVE_STERN_STAR));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->HEAVE_STERN_PORT));
     return offset;
    }

    virtual const char * getType() override { return "auv_msgs/ThrusterForces"; };
    virtual const char * getMD5() override { return "56105fa5690ee1db02111af806c88d1a"; };

  };

}
#endif
