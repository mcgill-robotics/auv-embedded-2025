#ifndef _ROS_gazebo_msgs_PerformanceMetrics_h
#define _ROS_gazebo_msgs_PerformanceMetrics_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "gazebo_msgs/SensorPerformanceMetric.h"

namespace gazebo_msgs
{

  class PerformanceMetrics : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef double _real_time_factor_type;
      _real_time_factor_type real_time_factor;
      uint32_t sensors_length;
      typedef gazebo_msgs::SensorPerformanceMetric _sensors_type;
      _sensors_type st_sensors;
      _sensors_type * sensors;

    PerformanceMetrics():
      header(),
      real_time_factor(0),
      sensors_length(0), st_sensors(), sensors(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_real_time_factor;
      u_real_time_factor.real = this->real_time_factor;
      *(outbuffer + offset + 0) = (u_real_time_factor.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_real_time_factor.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_real_time_factor.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_real_time_factor.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_real_time_factor.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_real_time_factor.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_real_time_factor.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_real_time_factor.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->real_time_factor);
      *(outbuffer + offset + 0) = (this->sensors_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sensors_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sensors_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sensors_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sensors_length);
      for( uint32_t i = 0; i < sensors_length; i++){
      offset += this->sensors[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_real_time_factor;
      u_real_time_factor.base = 0;
      u_real_time_factor.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_real_time_factor.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_real_time_factor.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_real_time_factor.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_real_time_factor.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_real_time_factor.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_real_time_factor.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_real_time_factor.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->real_time_factor = u_real_time_factor.real;
      offset += sizeof(this->real_time_factor);
      uint32_t sensors_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      sensors_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      sensors_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      sensors_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->sensors_length);
      if(sensors_lengthT > sensors_length)
        this->sensors = (gazebo_msgs::SensorPerformanceMetric*)realloc(this->sensors, sensors_lengthT * sizeof(gazebo_msgs::SensorPerformanceMetric));
      sensors_length = sensors_lengthT;
      for( uint32_t i = 0; i < sensors_length; i++){
      offset += this->st_sensors.deserialize(inbuffer + offset);
        memcpy( &(this->sensors[i]), &(this->st_sensors), sizeof(gazebo_msgs::SensorPerformanceMetric));
      }
     return offset;
    }

    virtual const char * getType() override { return "gazebo_msgs/PerformanceMetrics"; };
    virtual const char * getMD5() override { return "884f71fd5037b886ec5e126b83c4425a"; };

  };

}
#endif
