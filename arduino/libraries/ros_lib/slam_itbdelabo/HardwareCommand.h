#ifndef _ROS_slam_itbdelabo_HardwareCommand_h
#define _ROS_slam_itbdelabo_HardwareCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace slam_itbdelabo
{

  class HardwareCommand : public ros::Msg
  {
    public:
      typedef uint8_t _movement_command_type;
      _movement_command_type movement_command;
      typedef uint8_t _cam_angle_command_type;
      _cam_angle_command_type cam_angle_command;
      typedef float _right_motor_speed_type;
      _right_motor_speed_type right_motor_speed;
      typedef float _left_motor_speed_type;
      _left_motor_speed_type left_motor_speed;

    HardwareCommand():
      movement_command(0),
      cam_angle_command(0),
      right_motor_speed(0),
      left_motor_speed(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->movement_command >> (8 * 0)) & 0xFF;
      offset += sizeof(this->movement_command);
      *(outbuffer + offset + 0) = (this->cam_angle_command >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cam_angle_command);
      union {
        float real;
        uint32_t base;
      } u_right_motor_speed;
      u_right_motor_speed.real = this->right_motor_speed;
      *(outbuffer + offset + 0) = (u_right_motor_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_motor_speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_motor_speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_motor_speed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_motor_speed);
      union {
        float real;
        uint32_t base;
      } u_left_motor_speed;
      u_left_motor_speed.real = this->left_motor_speed;
      *(outbuffer + offset + 0) = (u_left_motor_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_motor_speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_motor_speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_motor_speed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_motor_speed);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->movement_command =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->movement_command);
      this->cam_angle_command =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->cam_angle_command);
      union {
        float real;
        uint32_t base;
      } u_right_motor_speed;
      u_right_motor_speed.base = 0;
      u_right_motor_speed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_motor_speed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_motor_speed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_motor_speed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_motor_speed = u_right_motor_speed.real;
      offset += sizeof(this->right_motor_speed);
      union {
        float real;
        uint32_t base;
      } u_left_motor_speed;
      u_left_motor_speed.base = 0;
      u_left_motor_speed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_motor_speed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_motor_speed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_motor_speed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_motor_speed = u_left_motor_speed.real;
      offset += sizeof(this->left_motor_speed);
     return offset;
    }

    virtual const char * getType() override { return "slam_itbdelabo/HardwareCommand"; };
    virtual const char * getMD5() override { return "90de4a63cfe651e5dbcdc601f211c50f"; };

  };

}
#endif
