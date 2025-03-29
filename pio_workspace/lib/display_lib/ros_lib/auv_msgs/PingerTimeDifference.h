#ifndef _ROS_auv_msgs_PingerTimeDifference_h
#define _ROS_auv_msgs_PingerTimeDifference_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace auv_msgs
{

  class PingerTimeDifference : public ros::Msg
  {
    public:
      typedef bool _is_pinger1_active_type;
      _is_pinger1_active_type is_pinger1_active;
      typedef bool _is_pinger2_active_type;
      _is_pinger2_active_type is_pinger2_active;
      typedef bool _is_pinger3_active_type;
      _is_pinger3_active_type is_pinger3_active;
      typedef bool _is_pinger4_active_type;
      _is_pinger4_active_type is_pinger4_active;
      uint32_t dt_pinger1_length;
      typedef double _dt_pinger1_type;
      _dt_pinger1_type st_dt_pinger1;
      _dt_pinger1_type * dt_pinger1;
      uint32_t dt_pinger2_length;
      typedef double _dt_pinger2_type;
      _dt_pinger2_type st_dt_pinger2;
      _dt_pinger2_type * dt_pinger2;
      uint32_t dt_pinger3_length;
      typedef double _dt_pinger3_type;
      _dt_pinger3_type st_dt_pinger3;
      _dt_pinger3_type * dt_pinger3;
      uint32_t dt_pinger4_length;
      typedef double _dt_pinger4_type;
      _dt_pinger4_type st_dt_pinger4;
      _dt_pinger4_type * dt_pinger4;

    PingerTimeDifference():
      is_pinger1_active(0),
      is_pinger2_active(0),
      is_pinger3_active(0),
      is_pinger4_active(0),
      dt_pinger1_length(0), st_dt_pinger1(), dt_pinger1(nullptr),
      dt_pinger2_length(0), st_dt_pinger2(), dt_pinger2(nullptr),
      dt_pinger3_length(0), st_dt_pinger3(), dt_pinger3(nullptr),
      dt_pinger4_length(0), st_dt_pinger4(), dt_pinger4(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_is_pinger1_active;
      u_is_pinger1_active.real = this->is_pinger1_active;
      *(outbuffer + offset + 0) = (u_is_pinger1_active.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_pinger1_active);
      union {
        bool real;
        uint8_t base;
      } u_is_pinger2_active;
      u_is_pinger2_active.real = this->is_pinger2_active;
      *(outbuffer + offset + 0) = (u_is_pinger2_active.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_pinger2_active);
      union {
        bool real;
        uint8_t base;
      } u_is_pinger3_active;
      u_is_pinger3_active.real = this->is_pinger3_active;
      *(outbuffer + offset + 0) = (u_is_pinger3_active.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_pinger3_active);
      union {
        bool real;
        uint8_t base;
      } u_is_pinger4_active;
      u_is_pinger4_active.real = this->is_pinger4_active;
      *(outbuffer + offset + 0) = (u_is_pinger4_active.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_pinger4_active);
      *(outbuffer + offset + 0) = (this->dt_pinger1_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->dt_pinger1_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->dt_pinger1_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->dt_pinger1_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dt_pinger1_length);
      for( uint32_t i = 0; i < dt_pinger1_length; i++){
      union {
        double real;
        uint64_t base;
      } u_dt_pinger1i;
      u_dt_pinger1i.real = this->dt_pinger1[i];
      *(outbuffer + offset + 0) = (u_dt_pinger1i.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dt_pinger1i.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dt_pinger1i.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dt_pinger1i.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_dt_pinger1i.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_dt_pinger1i.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_dt_pinger1i.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_dt_pinger1i.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->dt_pinger1[i]);
      }
      *(outbuffer + offset + 0) = (this->dt_pinger2_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->dt_pinger2_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->dt_pinger2_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->dt_pinger2_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dt_pinger2_length);
      for( uint32_t i = 0; i < dt_pinger2_length; i++){
      union {
        double real;
        uint64_t base;
      } u_dt_pinger2i;
      u_dt_pinger2i.real = this->dt_pinger2[i];
      *(outbuffer + offset + 0) = (u_dt_pinger2i.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dt_pinger2i.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dt_pinger2i.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dt_pinger2i.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_dt_pinger2i.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_dt_pinger2i.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_dt_pinger2i.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_dt_pinger2i.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->dt_pinger2[i]);
      }
      *(outbuffer + offset + 0) = (this->dt_pinger3_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->dt_pinger3_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->dt_pinger3_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->dt_pinger3_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dt_pinger3_length);
      for( uint32_t i = 0; i < dt_pinger3_length; i++){
      union {
        double real;
        uint64_t base;
      } u_dt_pinger3i;
      u_dt_pinger3i.real = this->dt_pinger3[i];
      *(outbuffer + offset + 0) = (u_dt_pinger3i.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dt_pinger3i.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dt_pinger3i.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dt_pinger3i.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_dt_pinger3i.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_dt_pinger3i.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_dt_pinger3i.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_dt_pinger3i.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->dt_pinger3[i]);
      }
      *(outbuffer + offset + 0) = (this->dt_pinger4_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->dt_pinger4_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->dt_pinger4_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->dt_pinger4_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dt_pinger4_length);
      for( uint32_t i = 0; i < dt_pinger4_length; i++){
      union {
        double real;
        uint64_t base;
      } u_dt_pinger4i;
      u_dt_pinger4i.real = this->dt_pinger4[i];
      *(outbuffer + offset + 0) = (u_dt_pinger4i.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dt_pinger4i.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dt_pinger4i.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dt_pinger4i.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_dt_pinger4i.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_dt_pinger4i.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_dt_pinger4i.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_dt_pinger4i.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->dt_pinger4[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_is_pinger1_active;
      u_is_pinger1_active.base = 0;
      u_is_pinger1_active.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_pinger1_active = u_is_pinger1_active.real;
      offset += sizeof(this->is_pinger1_active);
      union {
        bool real;
        uint8_t base;
      } u_is_pinger2_active;
      u_is_pinger2_active.base = 0;
      u_is_pinger2_active.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_pinger2_active = u_is_pinger2_active.real;
      offset += sizeof(this->is_pinger2_active);
      union {
        bool real;
        uint8_t base;
      } u_is_pinger3_active;
      u_is_pinger3_active.base = 0;
      u_is_pinger3_active.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_pinger3_active = u_is_pinger3_active.real;
      offset += sizeof(this->is_pinger3_active);
      union {
        bool real;
        uint8_t base;
      } u_is_pinger4_active;
      u_is_pinger4_active.base = 0;
      u_is_pinger4_active.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_pinger4_active = u_is_pinger4_active.real;
      offset += sizeof(this->is_pinger4_active);
      uint32_t dt_pinger1_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      dt_pinger1_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      dt_pinger1_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      dt_pinger1_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->dt_pinger1_length);
      if(dt_pinger1_lengthT > dt_pinger1_length)
        this->dt_pinger1 = (double*)realloc(this->dt_pinger1, dt_pinger1_lengthT * sizeof(double));
      dt_pinger1_length = dt_pinger1_lengthT;
      for( uint32_t i = 0; i < dt_pinger1_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_dt_pinger1;
      u_st_dt_pinger1.base = 0;
      u_st_dt_pinger1.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_dt_pinger1.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_dt_pinger1.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_dt_pinger1.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_dt_pinger1.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_dt_pinger1.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_dt_pinger1.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_dt_pinger1.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_dt_pinger1 = u_st_dt_pinger1.real;
      offset += sizeof(this->st_dt_pinger1);
        memcpy( &(this->dt_pinger1[i]), &(this->st_dt_pinger1), sizeof(double));
      }
      uint32_t dt_pinger2_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      dt_pinger2_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      dt_pinger2_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      dt_pinger2_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->dt_pinger2_length);
      if(dt_pinger2_lengthT > dt_pinger2_length)
        this->dt_pinger2 = (double*)realloc(this->dt_pinger2, dt_pinger2_lengthT * sizeof(double));
      dt_pinger2_length = dt_pinger2_lengthT;
      for( uint32_t i = 0; i < dt_pinger2_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_dt_pinger2;
      u_st_dt_pinger2.base = 0;
      u_st_dt_pinger2.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_dt_pinger2.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_dt_pinger2.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_dt_pinger2.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_dt_pinger2.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_dt_pinger2.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_dt_pinger2.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_dt_pinger2.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_dt_pinger2 = u_st_dt_pinger2.real;
      offset += sizeof(this->st_dt_pinger2);
        memcpy( &(this->dt_pinger2[i]), &(this->st_dt_pinger2), sizeof(double));
      }
      uint32_t dt_pinger3_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      dt_pinger3_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      dt_pinger3_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      dt_pinger3_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->dt_pinger3_length);
      if(dt_pinger3_lengthT > dt_pinger3_length)
        this->dt_pinger3 = (double*)realloc(this->dt_pinger3, dt_pinger3_lengthT * sizeof(double));
      dt_pinger3_length = dt_pinger3_lengthT;
      for( uint32_t i = 0; i < dt_pinger3_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_dt_pinger3;
      u_st_dt_pinger3.base = 0;
      u_st_dt_pinger3.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_dt_pinger3.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_dt_pinger3.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_dt_pinger3.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_dt_pinger3.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_dt_pinger3.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_dt_pinger3.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_dt_pinger3.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_dt_pinger3 = u_st_dt_pinger3.real;
      offset += sizeof(this->st_dt_pinger3);
        memcpy( &(this->dt_pinger3[i]), &(this->st_dt_pinger3), sizeof(double));
      }
      uint32_t dt_pinger4_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      dt_pinger4_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      dt_pinger4_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      dt_pinger4_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->dt_pinger4_length);
      if(dt_pinger4_lengthT > dt_pinger4_length)
        this->dt_pinger4 = (double*)realloc(this->dt_pinger4, dt_pinger4_lengthT * sizeof(double));
      dt_pinger4_length = dt_pinger4_lengthT;
      for( uint32_t i = 0; i < dt_pinger4_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_dt_pinger4;
      u_st_dt_pinger4.base = 0;
      u_st_dt_pinger4.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_dt_pinger4.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_dt_pinger4.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_dt_pinger4.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_dt_pinger4.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_dt_pinger4.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_dt_pinger4.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_dt_pinger4.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_dt_pinger4 = u_st_dt_pinger4.real;
      offset += sizeof(this->st_dt_pinger4);
        memcpy( &(this->dt_pinger4[i]), &(this->st_dt_pinger4), sizeof(double));
      }
     return offset;
    }

    virtual const char * getType() override { return "auv_msgs/PingerTimeDifference"; };
    virtual const char * getMD5() override { return "ca18252a2ff29218ec8cf0924629fb65"; };

  };

}
#endif
