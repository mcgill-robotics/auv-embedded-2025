#ifndef _ROS_sbg_driver_SbgAirData_h
#define _ROS_sbg_driver_SbgAirData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "sbg_driver/SbgAirDataStatus.h"

namespace sbg_driver
{

  class SbgAirData : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint32_t _time_stamp_type;
      _time_stamp_type time_stamp;
      typedef sbg_driver::SbgAirDataStatus _status_type;
      _status_type status;
      typedef double _pressure_abs_type;
      _pressure_abs_type pressure_abs;
      typedef double _altitude_type;
      _altitude_type altitude;
      typedef double _pressure_diff_type;
      _pressure_diff_type pressure_diff;
      typedef double _true_air_speed_type;
      _true_air_speed_type true_air_speed;
      typedef double _air_temperature_type;
      _air_temperature_type air_temperature;

    SbgAirData():
      header(),
      time_stamp(0),
      status(),
      pressure_abs(0),
      altitude(0),
      pressure_diff(0),
      true_air_speed(0),
      air_temperature(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->time_stamp >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->time_stamp >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->time_stamp >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->time_stamp >> (8 * 3)) & 0xFF;
      offset += sizeof(this->time_stamp);
      offset += this->status.serialize(outbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_pressure_abs;
      u_pressure_abs.real = this->pressure_abs;
      *(outbuffer + offset + 0) = (u_pressure_abs.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pressure_abs.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pressure_abs.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pressure_abs.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_pressure_abs.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_pressure_abs.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_pressure_abs.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_pressure_abs.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->pressure_abs);
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
        double real;
        uint64_t base;
      } u_pressure_diff;
      u_pressure_diff.real = this->pressure_diff;
      *(outbuffer + offset + 0) = (u_pressure_diff.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pressure_diff.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pressure_diff.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pressure_diff.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_pressure_diff.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_pressure_diff.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_pressure_diff.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_pressure_diff.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->pressure_diff);
      union {
        double real;
        uint64_t base;
      } u_true_air_speed;
      u_true_air_speed.real = this->true_air_speed;
      *(outbuffer + offset + 0) = (u_true_air_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_true_air_speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_true_air_speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_true_air_speed.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_true_air_speed.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_true_air_speed.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_true_air_speed.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_true_air_speed.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->true_air_speed);
      union {
        double real;
        uint64_t base;
      } u_air_temperature;
      u_air_temperature.real = this->air_temperature;
      *(outbuffer + offset + 0) = (u_air_temperature.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_air_temperature.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_air_temperature.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_air_temperature.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_air_temperature.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_air_temperature.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_air_temperature.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_air_temperature.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->air_temperature);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->time_stamp =  ((uint32_t) (*(inbuffer + offset)));
      this->time_stamp |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->time_stamp |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->time_stamp |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->time_stamp);
      offset += this->status.deserialize(inbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_pressure_abs;
      u_pressure_abs.base = 0;
      u_pressure_abs.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pressure_abs.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pressure_abs.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pressure_abs.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_pressure_abs.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_pressure_abs.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_pressure_abs.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_pressure_abs.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->pressure_abs = u_pressure_abs.real;
      offset += sizeof(this->pressure_abs);
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
        double real;
        uint64_t base;
      } u_pressure_diff;
      u_pressure_diff.base = 0;
      u_pressure_diff.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pressure_diff.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pressure_diff.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pressure_diff.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_pressure_diff.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_pressure_diff.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_pressure_diff.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_pressure_diff.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->pressure_diff = u_pressure_diff.real;
      offset += sizeof(this->pressure_diff);
      union {
        double real;
        uint64_t base;
      } u_true_air_speed;
      u_true_air_speed.base = 0;
      u_true_air_speed.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_true_air_speed.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_true_air_speed.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_true_air_speed.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_true_air_speed.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_true_air_speed.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_true_air_speed.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_true_air_speed.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->true_air_speed = u_true_air_speed.real;
      offset += sizeof(this->true_air_speed);
      union {
        double real;
        uint64_t base;
      } u_air_temperature;
      u_air_temperature.base = 0;
      u_air_temperature.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_air_temperature.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_air_temperature.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_air_temperature.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_air_temperature.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_air_temperature.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_air_temperature.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_air_temperature.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->air_temperature = u_air_temperature.real;
      offset += sizeof(this->air_temperature);
     return offset;
    }

    virtual const char * getType() override { return "sbg_driver/SbgAirData"; };
    virtual const char * getMD5() override { return "f7982abc9b7165b89ea4d8dda93717f9"; };

  };

}
#endif
