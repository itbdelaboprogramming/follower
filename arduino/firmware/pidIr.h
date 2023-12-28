/** 
    Code to compute PID control - Header File
    Author  : Achmad Syahrul Irwansyah
    Project : GPS Tracking
    For more information contact
    -> email: ach.syahrul99@gmail.com
    Reference :
    - [1]R. C. Dorf and R. H. Bishop, Modern Control Systems. Essex: Pearson, Cop, 2017.
**/

#ifndef PIDIR_H
#define PIDIR_H

#include <Arduino.h>

class pidIr{
    public:
    pidIr(float p, float i, float d);

    float compute(float setpoint, float feedback, float max_response, float Ts);
    void reset();
    void printError();
    void printSumError();
    float getError(){return error;};
    float getSumError(){return sum_error;};

    private:
    float error;
    float last_error;
    float sum_error;
    float output;
    float kP;
    float kI;
    float kD;
};

#endif
