#ifndef _ROS_SERVICE_SetMapping_h
#define _ROS_SERVICE_SetMapping_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace slam_itbdelabo
{

static const char SETMAPPING[] = "slam_itbdelabo/SetMapping";

  class SetMappingRequest : public ros::Msg
  {
    public:
      typedef bool _start_type;
      _start_type start;
      typedef bool _pause_type;
      _pause_type pause;
      typedef bool _stop_type;
      _stop_type stop;

    SetMappingRequest():
      start(0),
      pause(0),
      stop(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_start;
      u_start.real = this->start;
      *(outbuffer + offset + 0) = (u_start.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->start);
      union {
        bool real;
        uint8_t base;
      } u_pause;
      u_pause.real = this->pause;
      *(outbuffer + offset + 0) = (u_pause.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->pause);
      union {
        bool real;
        uint8_t base;
      } u_stop;
      u_stop.real = this->stop;
      *(outbuffer + offset + 0) = (u_stop.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->stop);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_start;
      u_start.base = 0;
      u_start.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->start = u_start.real;
      offset += sizeof(this->start);
      union {
        bool real;
        uint8_t base;
      } u_pause;
      u_pause.base = 0;
      u_pause.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->pause = u_pause.real;
      offset += sizeof(this->pause);
      union {
        bool real;
        uint8_t base;
      } u_stop;
      u_stop.base = 0;
      u_stop.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->stop = u_stop.real;
      offset += sizeof(this->stop);
     return offset;
    }

    virtual const char * getType() override { return SETMAPPING; };
    virtual const char * getMD5() override { return "f95466c1308ea80d53f23435b7ed25f4"; };

  };

  class SetMappingResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;
      typedef uint8_t _code_type;
      _code_type code;

    SetMappingResponse():
      success(0),
      code(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      *(outbuffer + offset + 0) = (this->code >> (8 * 0)) & 0xFF;
      offset += sizeof(this->code);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
      this->code =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->code);
     return offset;
    }

    virtual const char * getType() override { return SETMAPPING; };
    virtual const char * getMD5() override { return "484f4187c464b847adba192380ff095e"; };

  };

  class SetMapping {
    public:
    typedef SetMappingRequest Request;
    typedef SetMappingResponse Response;
  };

}
#endif
