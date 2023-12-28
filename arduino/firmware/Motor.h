/** 
  Code to assign motor pin in arduino with BTS7960 motor driver
  Author  : Achmad Syahrul Irwansyah
  Project : GPS Tracking
  For more information contact
  -> email: ach.syahrul99@gmail.com
  Reference :
  -
**/

#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor{
    public:
    Motor(int ren, int len, int pwm);

    void begin();
    void rotate(int val);
    void stop();

    private:
    int right_en_pin;
    int left_en_pin;
    int pwm_pin;
};

#endif
