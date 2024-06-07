#ifndef _ROS_auv_msgs_VelocityReport_h
#define _ROS_auv_msgs_VelocityReport_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace auv_msgs
{

  class VelocityReport : public ros::Msg
  {
    public:
      typedef double _vx_type;
      _vx_type vx;
      typedef double _vy_type;
      _vy_type vy;
      typedef double _vz_type;
      _vz_type vz;
      typedef double _altitude_type;
      _altitude_type altitude;
      typedef bool _valid_type;
      _valid_type valid;
      typedef double _fom_type;
      _fom_type fom;
      uint32_t covariance_length;
      typedef double _covariance_type;
      _covariance_type st_covariance;
      _covariance_type * covariance;
      typedef double _time_of_validity_type;
      _time_of_validity_type time_of_validity;
      typedef double _time_of_transmission_type;
      _time_of_transmission_type time_of_transmission;
      typedef double _time_type;
      _time_type time;
      typedef bool _status_type;
      _status_type status;

    VelocityReport():
      vx(0),
      vy(0),
      vz(0),
      altitude(0),
      valid(0),
      fom(0),
      covariance_length(0), st_covariance(), covariance(nullptr),
      time_of_validity(0),
      time_of_transmission(0),
      time(0),
      status(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_vx;
      u_vx.real = this->vx;
      *(outbuffer + offset + 0) = (u_vx.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_vx.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_vx.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_vx.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_vx.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_vx.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_vx.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_vx.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->vx);
      union {
        double real;
        uint64_t base;
      } u_vy;
      u_vy.real = this->vy;
      *(outbuffer + offset + 0) = (u_vy.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_vy.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_vy.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_vy.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_vy.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_vy.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_vy.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_vy.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->vy);
      union {
        double real;
        uint64_t base;
      } u_vz;
      u_vz.real = this->vz;
      *(outbuffer + offset + 0) = (u_vz.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_vz.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_vz.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_vz.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_vz.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_vz.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_vz.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_vz.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->vz);
      union {
        double real;
        uint64_t base;
      } u_altitude;
      u_altitude.real = this->altitude;
      *(outbuffer + offset + 0) = (u_altitude.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_altitude.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_altitude.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_altitude.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_altitude.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_altitude.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_altitude.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_altitude.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->altitude);
      union {
        bool real;
        uint8_t base;
      } u_valid;
      u_valid.real = this->valid;
      *(outbuffer + offset + 0) = (u_valid.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->valid);
      union {
        double real;
        uint64_t base;
      } u_fom;
      u_fom.real = this->fom;
      *(outbuffer + offset + 0) = (u_fom.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_fom.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_fom.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_fom.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_fom.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_fom.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_fom.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_fom.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->fom);
      *(outbuffer + offset + 0) = (this->covariance_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->covariance_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->covariance_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->covariance_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->covariance_length);
      for( uint32_t i = 0; i < covariance_length; i++){
      union {
        double real;
        uint64_t base;
      } u_covariancei;
      u_covariancei.real = this->covariance[i];
      *(outbuffer + offset + 0) = (u_covariancei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_covariancei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_covariancei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_covariancei.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_covariancei.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_covariancei.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_covariancei.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_covariancei.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->covariance[i]);
      }
      union {
        double real;
        uint64_t base;
      } u_time_of_validity;
      u_time_of_validity.real = this->time_of_validity;
      *(outbuffer + offset + 0) = (u_time_of_validity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_time_of_validity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_time_of_validity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_time_of_validity.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_time_of_validity.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_time_of_validity.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_time_of_validity.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_time_of_validity.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->time_of_validity);
      union {
        double real;
        uint64_t base;
      } u_time_of_transmission;
      u_time_of_transmission.real = this->time_of_transmission;
      *(outbuffer + offset + 0) = (u_time_of_transmission.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_time_of_transmission.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_time_of_transmission.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_time_of_transmission.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_time_of_transmission.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_time_of_transmission.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_time_of_transmission.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_time_of_transmission.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->time_of_transmission);
      union {
        double real;
        uint64_t base;
      } u_time;
      u_time.real = this->time;
      *(outbuffer + offset + 0) = (u_time.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_time.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_time.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_time.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_time.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_time.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_time.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_time.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->time);
      union {
        bool real;
        uint8_t base;
      } u_status;
      u_status.real = this->status;
      *(outbuffer + offset + 0) = (u_status.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->status);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_vx;
      u_vx.base = 0;
      u_vx.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_vx.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_vx.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_vx.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_vx.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_vx.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_vx.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_vx.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->vx = u_vx.real;
      offset += sizeof(this->vx);
      union {
        double real;
        uint64_t base;
      } u_vy;
      u_vy.base = 0;
      u_vy.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_vy.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_vy.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_vy.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_vy.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_vy.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_vy.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_vy.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->vy = u_vy.real;
      offset += sizeof(this->vy);
      union {
        double real;
        uint64_t base;
      } u_vz;
      u_vz.base = 0;
      u_vz.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_vz.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_vz.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_vz.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_vz.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_vz.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_vz.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_vz.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->vz = u_vz.real;
      offset += sizeof(this->vz);
      union {
        double real;
        uint64_t base;
      } u_altitude;
      u_altitude.base = 0;
      u_altitude.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_altitude.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_altitude.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_altitude.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_altitude.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_altitude.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_altitude.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_altitude.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->altitude = u_altitude.real;
      offset += sizeof(this->altitude);
      union {
        bool real;
        uint8_t base;
      } u_valid;
      u_valid.base = 0;
      u_valid.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->valid = u_valid.real;
      offset += sizeof(this->valid);
      union {
        double real;
        uint64_t base;
      } u_fom;
      u_fom.base = 0;
      u_fom.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_fom.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_fom.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_fom.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_fom.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_fom.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_fom.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_fom.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->fom = u_fom.real;
      offset += sizeof(this->fom);
      uint32_t covariance_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      covariance_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      covariance_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      covariance_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->covariance_length);
      if(covariance_lengthT > covariance_length)
        this->covariance = (double*)realloc(this->covariance, covariance_lengthT * sizeof(double));
      covariance_length = covariance_lengthT;
      for( uint32_t i = 0; i < covariance_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_covariance;
      u_st_covariance.base = 0;
      u_st_covariance.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_covariance.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_covariance.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_covariance.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_covariance.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_covariance.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_covariance.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_covariance.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_covariance = u_st_covariance.real;
      offset += sizeof(this->st_covariance);
        memcpy( &(this->covariance[i]), &(this->st_covariance), sizeof(double));
      }
      union {
        double real;
        uint64_t base;
      } u_time_of_validity;
      u_time_of_validity.base = 0;
      u_time_of_validity.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_time_of_validity.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_time_of_validity.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_time_of_validity.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_time_of_validity.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_time_of_validity.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_time_of_validity.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_time_of_validity.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->time_of_validity = u_time_of_validity.real;
      offset += sizeof(this->time_of_validity);
      union {
        double real;
        uint64_t base;
      } u_time_of_transmission;
      u_time_of_transmission.base = 0;
      u_time_of_transmission.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_time_of_transmission.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_time_of_transmission.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_time_of_transmission.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_time_of_transmission.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_time_of_transmission.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_time_of_transmission.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_time_of_transmission.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->time_of_transmission = u_time_of_transmission.real;
      offset += sizeof(this->time_of_transmission);
      union {
        double real;
        uint64_t base;
      } u_time;
      u_time.base = 0;
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->time = u_time.real;
      offset += sizeof(this->time);
      union {
        bool real;
        uint8_t base;
      } u_status;
      u_status.base = 0;
      u_status.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->status = u_status.real;
      offset += sizeof(this->status);
     return offset;
    }

    virtual const char * getType() override { return "auv_msgs/VelocityReport"; };
    virtual const char * getMD5() override { return "673a7d5c3a5d7647c78b173ad54c86b6"; };

  };

}
#endif
