#ifndef _ROS_gazebo_msgs_SensorPerformanceMetric_h
#define _ROS_gazebo_msgs_SensorPerformanceMetric_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace gazebo_msgs
{

  class SensorPerformanceMetric : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      typedef double _sim_update_rate_type;
      _sim_update_rate_type sim_update_rate;
      typedef double _real_update_rate_type;
      _real_update_rate_type real_update_rate;
      typedef double _fps_type;
      _fps_type fps;

    SensorPerformanceMetric():
      name(""),
      sim_update_rate(0),
      real_update_rate(0),
      fps(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      union {
        double real;
        uint64_t base;
      } u_sim_update_rate;
      u_sim_update_rate.real = this->sim_update_rate;
      *(outbuffer + offset + 0) = (u_sim_update_rate.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sim_update_rate.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sim_update_rate.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sim_update_rate.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_sim_update_rate.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_sim_update_rate.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_sim_update_rate.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_sim_update_rate.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->sim_update_rate);
      union {
        double real;
        uint64_t base;
      } u_real_update_rate;
      u_real_update_rate.real = this->real_update_rate;
      *(outbuffer + offset + 0) = (u_real_update_rate.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_real_update_rate.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_real_update_rate.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_real_update_rate.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_real_update_rate.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_real_update_rate.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_real_update_rate.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_real_update_rate.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->real_update_rate);
      union {
        double real;
        uint64_t base;
      } u_fps;
      u_fps.real = this->fps;
      *(outbuffer + offset + 0) = (u_fps.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_fps.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_fps.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_fps.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_fps.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_fps.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_fps.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_fps.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->fps);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      union {
        double real;
        uint64_t base;
      } u_sim_update_rate;
      u_sim_update_rate.base = 0;
      u_sim_update_rate.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_sim_update_rate.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_sim_update_rate.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_sim_update_rate.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_sim_update_rate.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_sim_update_rate.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_sim_update_rate.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_sim_update_rate.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->sim_update_rate = u_sim_update_rate.real;
      offset += sizeof(this->sim_update_rate);
      union {
        double real;
        uint64_t base;
      } u_real_update_rate;
      u_real_update_rate.base = 0;
      u_real_update_rate.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_real_update_rate.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_real_update_rate.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_real_update_rate.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_real_update_rate.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_real_update_rate.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_real_update_rate.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_real_update_rate.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->real_update_rate = u_real_update_rate.real;
      offset += sizeof(this->real_update_rate);
      union {
        double real;
        uint64_t base;
      } u_fps;
      u_fps.base = 0;
      u_fps.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_fps.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_fps.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_fps.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_fps.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_fps.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_fps.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_fps.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->fps = u_fps.real;
      offset += sizeof(this->fps);
     return offset;
    }

    virtual const char * getType() override { return "gazebo_msgs/SensorPerformanceMetric"; };
    virtual const char * getMD5() override { return "01762ded18cfe9ebc7c8222667c99547"; };

  };

}
#endif
