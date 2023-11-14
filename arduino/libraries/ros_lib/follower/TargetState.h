#ifndef _ROS_follower_TargetState_h
#define _ROS_follower_TargetState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace follower
{

  class TargetState : public ros::Msg
  {
    public:
      typedef uint8_t _target_position_type;
      _target_position_type target_position;
      typedef float _target_distance_type;
      _target_distance_type target_distance;
      typedef uint8_t _cam_angle_command_type;
      _cam_angle_command_type cam_angle_command;

    TargetState():
      target_position(0),
      target_distance(0),
      cam_angle_command(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->target_position >> (8 * 0)) & 0xFF;
      offset += sizeof(this->target_position);
      union {
        float real;
        uint32_t base;
      } u_target_distance;
      u_target_distance.real = this->target_distance;
      *(outbuffer + offset + 0) = (u_target_distance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_target_distance.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_target_distance.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_target_distance.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->target_distance);
      *(outbuffer + offset + 0) = (this->cam_angle_command >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cam_angle_command);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->target_position =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->target_position);
      union {
        float real;
        uint32_t base;
      } u_target_distance;
      u_target_distance.base = 0;
      u_target_distance.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_target_distance.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_target_distance.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_target_distance.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->target_distance = u_target_distance.real;
      offset += sizeof(this->target_distance);
      this->cam_angle_command =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->cam_angle_command);
     return offset;
    }

    virtual const char * getType() override { return "follower/TargetState"; };
    virtual const char * getMD5() override { return "304430ea875847abe6475ffb37cf0dd8"; };

  };

}
#endif
