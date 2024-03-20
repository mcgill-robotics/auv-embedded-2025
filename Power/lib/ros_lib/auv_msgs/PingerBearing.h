#ifndef _ROS_auv_msgs_PingerBearing_h
#define _ROS_auv_msgs_PingerBearing_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Vector3.h"

namespace auv_msgs
{

  class PingerBearing : public ros::Msg
  {
    public:
      typedef geometry_msgs::Vector3 _pinger1_bearing_type;
      _pinger1_bearing_type pinger1_bearing;
      typedef geometry_msgs::Vector3 _pinger2_bearing_type;
      _pinger2_bearing_type pinger2_bearing;
      typedef geometry_msgs::Vector3 _pinger3_bearing_type;
      _pinger3_bearing_type pinger3_bearing;
      typedef geometry_msgs::Vector3 _pinger4_bearing_type;
      _pinger4_bearing_type pinger4_bearing;
      typedef float _state_x_type;
      _state_x_type state_x;
      typedef float _state_y_type;
      _state_y_type state_y;

    PingerBearing():
      pinger1_bearing(),
      pinger2_bearing(),
      pinger3_bearing(),
      pinger4_bearing(),
      state_x(0),
      state_y(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->pinger1_bearing.serialize(outbuffer + offset);
      offset += this->pinger2_bearing.serialize(outbuffer + offset);
      offset += this->pinger3_bearing.serialize(outbuffer + offset);
      offset += this->pinger4_bearing.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->state_x);
      offset += serializeAvrFloat64(outbuffer + offset, this->state_y);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->pinger1_bearing.deserialize(inbuffer + offset);
      offset += this->pinger2_bearing.deserialize(inbuffer + offset);
      offset += this->pinger3_bearing.deserialize(inbuffer + offset);
      offset += this->pinger4_bearing.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->state_x));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->state_y));
     return offset;
    }

    virtual const char * getType() override { return "auv_msgs/PingerBearing"; };
    virtual const char * getMD5() override { return "7acbc802f052c457652cbb76f41d48ee"; };

  };

}
#endif
