/** 
  Code to compute PID control
  Author  : Achmad Syahrul Irwansyah
  Project : GPS Tracking
  For more information contact
  -> email: ach.syahrul99@gmail.com
  Reference :
  - [1]R. C. Dorf and R. H. Bishop, Modern Control Systems. Essex: Pearson, Cop, 2017.
  - [2]https://www.sentekdynamics.com/sentek-dynamics-news/2020/8/24/pid-control-theory
**/

#include "pidIr.h"
#include <Arduino.h>

pidIr::pidIr(float p, float i, float d){
    kP = p;
    kI = i;
    kD = d;
    last_error = 0;
    sum_error = 0;
};

float pidIr::compute(float setpoint, float feedback, float max_response, float Ts){
    error = setpoint - feedback;
    sum_error += error;

    output = kP * error + kI * Ts * sum_error + kD/Ts * (error - last_error);
    last_error = error;

    return constrain(output, -max_response, max_response);
};

void pidIr::reset(){
    output = 0;
    last_error = 0;
    sum_error = 0;
};

void pidIr::printError(){
    Serial.print(error); Serial.print("\t");
};

void pidIr::printSumError(){
    Serial.print(sum_error); Serial.print("\t");
};
