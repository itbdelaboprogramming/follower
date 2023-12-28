/** 
  Code to assign motor pin in arduino with BTS7960 motor driver
  Author  : - Achmad Syahrul Irwansyah
            - M. Luthfi Hariyadin
  Project : GPS Tracking
  For more information contact
  -> email: ach.syahrul99@gmail.com
  Reference :
  -
**/

#include <Arduino.h>
#include "Motor.h"

// Assigning each pin to the Class
Motor::Motor(int ren, int len, int pwm){
  right_en_pin = ren;
  left_en_pin = len;
  pwm_pin = pwm;
};

// Method to set the motor pin mode
void Motor::begin(){
  pinMode(right_en_pin, OUTPUT);
  pinMode(left_en_pin, OUTPUT);
  pinMode(pwm_pin, OUTPUT);
};

// Method to assign PWM value and rotate the motor
void Motor::rotate(int val){
  if (abs(val) >= 255){
    digitalWrite(right_en_pin, LOW);
    digitalWrite(left_en_pin, LOW);
    analogWrite(pwm_pin, 0);
  } else if (val > 0) {
    digitalWrite(right_en_pin, HIGH);
    digitalWrite(left_en_pin, LOW);
    analogWrite(pwm_pin, val);
  } else if (val < 0) {
    digitalWrite(right_en_pin, LOW);
    digitalWrite(left_en_pin, HIGH);
    analogWrite(pwm_pin, -val);
  } else {
    digitalWrite(right_en_pin, LOW);
    digitalWrite(left_en_pin, LOW);
    analogWrite(pwm_pin, 0);
  }
};

void Motor::stop(){
  digitalWrite(right_en_pin, LOW);
  digitalWrite(left_en_pin, LOW);
  analogWrite(pwm_pin, 0);
};
