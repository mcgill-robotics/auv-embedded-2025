#ifndef _ROS_auv_msgs_UnityState_h
#define _ROS_auv_msgs_UnityState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"

namespace auv_msgs
{

  class UnityState : public ros::Msg
  {
    public:
      typedef geometry_msgs::Vector3 _position_type;
      _position_type position;
      typedef geometry_msgs::Quaternion _orientation_type;
      _orientation_type orientation;
      typedef geometry_msgs::Vector3 _velocity_type;
      _velocity_type velocity;
      typedef geometry_msgs::Vector3 _angular_velocity_type;
      _angular_velocity_type angular_velocity;
      typedef int32_t _isDVLActive_type;
      _isDVLActive_type isDVLActive;
      typedef int32_t _isDepthSensorActive_type;
      _isDepthSensorActive_type isDepthSensorActive;
      typedef int32_t _isIMUActive_type;
      _isIMUActive_type isIMUActive;
      typedef int32_t _isHydrophonesActive_type;
      _isHydrophonesActive_type isHydrophonesActive;

    UnityState():
      position(),
      orientation(),
      velocity(),
      angular_velocity(),
      isDVLActive(0),
      isDepthSensorActive(0),
      isIMUActive(0),
      isHydrophonesActive(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->position.serialize(outbuffer + offset);
      offset += this->orientation.serialize(outbuffer + offset);
      offset += this->velocity.serialize(outbuffer + offset);
      offset += this->angular_velocity.serialize(outbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_isDVLActive;
      u_isDVLActive.real = this->isDVLActive;
      *(outbuffer + offset + 0) = (u_isDVLActive.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_isDVLActive.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_isDVLActive.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_isDVLActive.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->isDVLActive);
      union {
        int32_t real;
        uint32_t base;
      } u_isDepthSensorActive;
      u_isDepthSensorActive.real = this->isDepthSensorActive;
      *(outbuffer + offset + 0) = (u_isDepthSensorActive.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_isDepthSensorActive.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_isDepthSensorActive.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_isDepthSensorActive.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->isDepthSensorActive);
      union {
        int32_t real;
        uint32_t base;
      } u_isIMUActive;
      u_isIMUActive.real = this->isIMUActive;
      *(outbuffer + offset + 0) = (u_isIMUActive.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_isIMUActive.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_isIMUActive.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_isIMUActive.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->isIMUActive);
      union {
        int32_t real;
        uint32_t base;
      } u_isHydrophonesActive;
      u_isHydrophonesActive.real = this->isHydrophonesActive;
      *(outbuffer + offset + 0) = (u_isHydrophonesActive.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_isHydrophonesActive.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_isHydrophonesActive.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_isHydrophonesActive.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->isHydrophonesActive);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->position.deserialize(inbuffer + offset);
      offset += this->orientation.deserialize(inbuffer + offset);
      offset += this->velocity.deserialize(inbuffer + offset);
      offset += this->angular_velocity.deserialize(inbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_isDVLActive;
      u_isDVLActive.base = 0;
      u_isDVLActive.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_isDVLActive.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_isDVLActive.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_isDVLActive.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->isDVLActive = u_isDVLActive.real;
      offset += sizeof(this->isDVLActive);
      union {
        int32_t real;
        uint32_t base;
      } u_isDepthSensorActive;
      u_isDepthSensorActive.base = 0;
      u_isDepthSensorActive.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_isDepthSensorActive.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_isDepthSensorActive.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_isDepthSensorActive.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->isDepthSensorActive = u_isDepthSensorActive.real;
      offset += sizeof(this->isDepthSensorActive);
      union {
        int32_t real;
        uint32_t base;
      } u_isIMUActive;
      u_isIMUActive.base = 0;
      u_isIMUActive.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_isIMUActive.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_isIMUActive.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_isIMUActive.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->isIMUActive = u_isIMUActive.real;
      offset += sizeof(this->isIMUActive);
      union {
        int32_t real;
        uint32_t base;
      } u_isHydrophonesActive;
      u_isHydrophonesActive.base = 0;
      u_isHydrophonesActive.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_isHydrophonesActive.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_isHydrophonesActive.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_isHydrophonesActive.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->isHydrophonesActive = u_isHydrophonesActive.real;
      offset += sizeof(this->isHydrophonesActive);
     return offset;
    }

    virtual const char * getType() override { return "auv_msgs/UnityState"; };
    virtual const char * getMD5() override { return "50dceff8b2762d7705a4a76f09fc6225"; };

  };

}
#endif
