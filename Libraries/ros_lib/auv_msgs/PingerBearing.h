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
      typedef double _state_x_type;
      _state_x_type state_x;
      typedef double _state_y_type;
      _state_y_type state_y;
      typedef double _state_z_type;
      _state_z_type state_z;

    PingerBearing():
      pinger1_bearing(),
      pinger2_bearing(),
      pinger3_bearing(),
      pinger4_bearing(),
      state_x(0),
      state_y(0),
      state_z(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->pinger1_bearing.serialize(outbuffer + offset);
      offset += this->pinger2_bearing.serialize(outbuffer + offset);
      offset += this->pinger3_bearing.serialize(outbuffer + offset);
      offset += this->pinger4_bearing.serialize(outbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_state_x;
      u_state_x.real = this->state_x;
      *(outbuffer + offset + 0) = (u_state_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_state_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_state_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_state_x.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_state_x.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_state_x.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_state_x.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_state_x.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->state_x);
      union {
        double real;
        uint64_t base;
      } u_state_y;
      u_state_y.real = this->state_y;
      *(outbuffer + offset + 0) = (u_state_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_state_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_state_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_state_y.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_state_y.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_state_y.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_state_y.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_state_y.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->state_y);
      union {
        double real;
        uint64_t base;
      } u_state_z;
      u_state_z.real = this->state_z;
      *(outbuffer + offset + 0) = (u_state_z.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_state_z.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_state_z.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_state_z.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_state_z.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_state_z.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_state_z.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_state_z.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->state_z);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->pinger1_bearing.deserialize(inbuffer + offset);
      offset += this->pinger2_bearing.deserialize(inbuffer + offset);
      offset += this->pinger3_bearing.deserialize(inbuffer + offset);
      offset += this->pinger4_bearing.deserialize(inbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_state_x;
      u_state_x.base = 0;
      u_state_x.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_state_x.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_state_x.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_state_x.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_state_x.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_state_x.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_state_x.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_state_x.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->state_x = u_state_x.real;
      offset += sizeof(this->state_x);
      union {
        double real;
        uint64_t base;
      } u_state_y;
      u_state_y.base = 0;
      u_state_y.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_state_y.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_state_y.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_state_y.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_state_y.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_state_y.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_state_y.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_state_y.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->state_y = u_state_y.real;
      offset += sizeof(this->state_y);
      union {
        double real;
        uint64_t base;
      } u_state_z;
      u_state_z.base = 0;
      u_state_z.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_state_z.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_state_z.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_state_z.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_state_z.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_state_z.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_state_z.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_state_z.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->state_z = u_state_z.real;
      offset += sizeof(this->state_z);
     return offset;
    }

    virtual const char * getType() override { return "auv_msgs/PingerBearing"; };
    virtual const char * getMD5() override { return "cc7163b61d111fc3682d27863d3016a7"; };

  };

}
#endif
