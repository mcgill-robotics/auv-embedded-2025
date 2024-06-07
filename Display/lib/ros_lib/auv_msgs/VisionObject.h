#ifndef _ROS_auv_msgs_VisionObject_h
#define _ROS_auv_msgs_VisionObject_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace auv_msgs
{

  class VisionObject : public ros::Msg
  {
    public:
      typedef const char* _label_type;
      _label_type label;
      typedef double _x_type;
      _x_type x;
      typedef double _y_type;
      _y_type y;
      typedef double _z_type;
      _z_type z;
      typedef double _theta_z_type;
      _theta_z_type theta_z;
      typedef double _extra_field_type;
      _extra_field_type extra_field;
      typedef double _confidence_type;
      _confidence_type confidence;

    VisionObject():
      label(""),
      x(0),
      y(0),
      z(0),
      theta_z(0),
      extra_field(0),
      confidence(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_label = strlen(this->label);
      varToArr(outbuffer + offset, length_label);
      offset += 4;
      memcpy(outbuffer + offset, this->label, length_label);
      offset += length_label;
      union {
        double real;
        uint64_t base;
      } u_x;
      u_x.real = this->x;
      *(outbuffer + offset + 0) = (u_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_x.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_x.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_x.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_x.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_x.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->x);
      union {
        double real;
        uint64_t base;
      } u_y;
      u_y.real = this->y;
      *(outbuffer + offset + 0) = (u_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_y.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_y.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_y.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_y.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_y.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->y);
      union {
        double real;
        uint64_t base;
      } u_z;
      u_z.real = this->z;
      *(outbuffer + offset + 0) = (u_z.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_z.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_z.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_z.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_z.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_z.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_z.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_z.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->z);
      union {
        double real;
        uint64_t base;
      } u_theta_z;
      u_theta_z.real = this->theta_z;
      *(outbuffer + offset + 0) = (u_theta_z.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_theta_z.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_theta_z.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_theta_z.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_theta_z.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_theta_z.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_theta_z.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_theta_z.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->theta_z);
      union {
        double real;
        uint64_t base;
      } u_extra_field;
      u_extra_field.real = this->extra_field;
      *(outbuffer + offset + 0) = (u_extra_field.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_extra_field.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_extra_field.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_extra_field.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_extra_field.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_extra_field.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_extra_field.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_extra_field.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->extra_field);
      union {
        double real;
        uint64_t base;
      } u_confidence;
      u_confidence.real = this->confidence;
      *(outbuffer + offset + 0) = (u_confidence.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_confidence.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_confidence.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_confidence.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_confidence.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_confidence.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_confidence.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_confidence.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->confidence);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_label;
      arrToVar(length_label, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_label; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_label-1]=0;
      this->label = (char *)(inbuffer + offset-1);
      offset += length_label;
      union {
        double real;
        uint64_t base;
      } u_x;
      u_x.base = 0;
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->x = u_x.real;
      offset += sizeof(this->x);
      union {
        double real;
        uint64_t base;
      } u_y;
      u_y.base = 0;
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->y = u_y.real;
      offset += sizeof(this->y);
      union {
        double real;
        uint64_t base;
      } u_z;
      u_z.base = 0;
      u_z.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_z.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_z.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_z.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_z.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_z.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_z.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_z.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->z = u_z.real;
      offset += sizeof(this->z);
      union {
        double real;
        uint64_t base;
      } u_theta_z;
      u_theta_z.base = 0;
      u_theta_z.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_theta_z.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_theta_z.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_theta_z.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_theta_z.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_theta_z.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_theta_z.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_theta_z.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->theta_z = u_theta_z.real;
      offset += sizeof(this->theta_z);
      union {
        double real;
        uint64_t base;
      } u_extra_field;
      u_extra_field.base = 0;
      u_extra_field.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_extra_field.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_extra_field.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_extra_field.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_extra_field.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_extra_field.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_extra_field.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_extra_field.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->extra_field = u_extra_field.real;
      offset += sizeof(this->extra_field);
      union {
        double real;
        uint64_t base;
      } u_confidence;
      u_confidence.base = 0;
      u_confidence.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_confidence.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_confidence.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_confidence.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_confidence.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_confidence.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_confidence.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_confidence.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->confidence = u_confidence.real;
      offset += sizeof(this->confidence);
     return offset;
    }

    virtual const char * getType() override { return "auv_msgs/VisionObject"; };
    virtual const char * getMD5() override { return "0f89e22270b02eb8422664d888a2cde3"; };

  };

}
#endif
