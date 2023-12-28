#ifndef _ROS_SERVICE_SetOwnMap_h
#define _ROS_SERVICE_SetOwnMap_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace slam_itbdelabo
{

static const char SETOWNMAP[] = "slam_itbdelabo/SetOwnMap";

  class SetOwnMapRequest : public ros::Msg
  {
    public:
      typedef bool _enable_type;
      _enable_type enable;
      typedef const char* _map_name_type;
      _map_name_type map_name;

    SetOwnMapRequest():
      enable(0),
      map_name("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_enable;
      u_enable.real = this->enable;
      *(outbuffer + offset + 0) = (u_enable.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->enable);
      uint32_t length_map_name = strlen(this->map_name);
      varToArr(outbuffer + offset, length_map_name);
      offset += 4;
      memcpy(outbuffer + offset, this->map_name, length_map_name);
      offset += length_map_name;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_enable;
      u_enable.base = 0;
      u_enable.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->enable = u_enable.real;
      offset += sizeof(this->enable);
      uint32_t length_map_name;
      arrToVar(length_map_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_map_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_map_name-1]=0;
      this->map_name = (char *)(inbuffer + offset-1);
      offset += length_map_name;
     return offset;
    }

    virtual const char * getType() override { return SETOWNMAP; };
    virtual const char * getMD5() override { return "998008f449da609442fa773b7c7fad66"; };

  };

  class SetOwnMapResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;
      typedef uint8_t _code_type;
      _code_type code;

    SetOwnMapResponse():
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

    virtual const char * getType() override { return SETOWNMAP; };
    virtual const char * getMD5() override { return "484f4187c464b847adba192380ff095e"; };

  };

  class SetOwnMap {
    public:
    typedef SetOwnMapRequest Request;
    typedef SetOwnMapResponse Response;
  };

}
#endif
