/** 
  Low Pass Filter code - Header File
  Author  : Achmad Syahrul Irwansyah
  Project : GPS Tracking
  For more information contact
  -> email: ach.syahrul99@gmail.com
  Reference :
  - [1]W. J. Palm, System dynamics. New York, Ny: Mcgraw-Hill, 2014.
**/

#ifndef LPF_H
#define LPF_H

#include <Arduino.h>
//#define PI 3.141592653

class LPF{
    public:
    LPF(float _cut_off_frequency);
    
    float filter(float raw, float time_sampling_ms);

    private:
    float cut_off_frequency;
    float filtered;
    float last_filtered;
    float alfa;
};

#endif
