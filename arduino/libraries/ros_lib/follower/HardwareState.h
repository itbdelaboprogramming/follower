#ifndef _ROS_follower_HardwareState_h
#define _ROS_follower_HardwareState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace follower
{

  class HardwareState : public ros::Msg
  {
    public:
      typedef float _ultrasonic_target_direction_type;
      _ultrasonic_target_direction_type ultrasonic_target_direction;
      typedef float _ultrasonic_target_distance_type;
      _ultrasonic_target_distance_type ultrasonic_target_distance;

    HardwareState():
      ultrasonic_target_direction(0),
      ultrasonic_target_distance(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_ultrasonic_target_direction;
      u_ultrasonic_target_direction.real = this->ultrasonic_target_direction;
      *(outbuffer + offset + 0) = (u_ultrasonic_target_direction.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ultrasonic_target_direction.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ultrasonic_target_direction.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ultrasonic_target_direction.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ultrasonic_target_direction);
      union {
        float real;
        uint32_t base;
      } u_ultrasonic_target_distance;
      u_ultrasonic_target_distance.real = this->ultrasonic_target_distance;
      *(outbuffer + offset + 0) = (u_ultrasonic_target_distance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ultrasonic_target_distance.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ultrasonic_target_distance.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ultrasonic_target_distance.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ultrasonic_target_distance);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_ultrasonic_target_direction;
      u_ultrasonic_target_direction.base = 0;
      u_ultrasonic_target_direction.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ultrasonic_target_direction.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ultrasonic_target_direction.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ultrasonic_target_direction.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ultrasonic_target_direction = u_ultrasonic_target_direction.real;
      offset += sizeof(this->ultrasonic_target_direction);
      union {
        float real;
        uint32_t base;
      } u_ultrasonic_target_distance;
      u_ultrasonic_target_distance.base = 0;
      u_ultrasonic_target_distance.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ultrasonic_target_distance.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ultrasonic_target_distance.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ultrasonic_target_distance.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ultrasonic_target_distance = u_ultrasonic_target_distance.real;
      offset += sizeof(this->ultrasonic_target_distance);
     return offset;
    }

    virtual const char * getType() override { return "follower/HardwareState"; };
    virtual const char * getMD5() override { return "f4bf79318c4e12fa88888a73af77187d"; };

  };

}
#endif
