#ifndef _ROS_follower_HardwareCommand_h
#define _ROS_follower_HardwareCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace follower
{

  class HardwareCommand : public ros::Msg
  {
    public:
      typedef uint8_t _movement_command_type;
      _movement_command_type movement_command;
      typedef uint8_t _cam_angle_command_type;
      _cam_angle_command_type cam_angle_command;

    HardwareCommand():
      movement_command(0),
      cam_angle_command(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->movement_command >> (8 * 0)) & 0xFF;
      offset += sizeof(this->movement_command);
      *(outbuffer + offset + 0) = (this->cam_angle_command >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cam_angle_command);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->movement_command =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->movement_command);
      this->cam_angle_command =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->cam_angle_command);
     return offset;
    }

    virtual const char * getType() override { return "follower/HardwareCommand"; };
    virtual const char * getMD5() override { return "269e235cd79b6192023f5ada1ae452c0"; };

  };

}
#endif
