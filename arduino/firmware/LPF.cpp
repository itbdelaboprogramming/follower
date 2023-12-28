/** 
  Code Low Pass Filter
  Author  : Achmad Syahrul Irwansyah
  Project : GPS Tracking
  For more information contact
  -> email: ach.syahrul99@gmail.com
  Reference :
  - [1]W. J. Palm, System dynamics. New York, Ny: Mcgraw-Hill, 2014.
**/

#include "LPF.h"

LPF::LPF(float _cut_off_frequency){
    cut_off_frequency = _cut_off_frequency;
    filtered = 0;
    last_filtered = 0;
};

float LPF::filter(float raw, float time_sampling_ms){
    float dt = time_sampling_ms/1000;

    alfa = (2.0*PI*cut_off_frequency*dt)/(1.0+(2*PI*cut_off_frequency*dt));
    filtered = alfa*raw + (1-alfa)*last_filtered;
    
    last_filtered = filtered;

    return filtered;
};
