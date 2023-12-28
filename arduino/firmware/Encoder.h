/** 
  Header file of encoder code
  Author  : - Achmad Syahrul Irwansyah
            - Mochamad Luthfi Hariyadin
  Project : GPS Tracking
  For more information contact
  -> email: ach.syahrul99@gmail.com
  ->      : luthfihariyadin06@gmail.com
  Reference :
  -
**/

#ifndef ENCODER_H
#define ENCODER_H

#define GEAR_RATIO 1980.0
#define REVOLUTION_DEGREE 360.0
#define REVOLUTION_RADIAN 6.28318531
#define S_TO_MS_CONVERTER 1000.0
#define M_TO_S_CONVERTER 60.0

class Encoder {
    public:
    Encoder(int pin_a, int pin_b);
    void start(void(*userFuncA)(void),void(*userFuncB)(void));
    int getPinA(){return enc_pin_a;};
    int getPinB(){return enc_pin_b;};

    long getPulse(){return enc_pulse;};
    float getAngleDeg(){return (float)enc_pulse/GEAR_RATIO * REVOLUTION_DEGREE;};
    float getAngleRad(){return (float)enc_pulse/GEAR_RATIO * REVOLUTION_RADIAN;};
    float getAngleRev(){return (float)enc_pulse/GEAR_RATIO;};

    long getLastPulse(){return enc_pulse_last;};
    float getLastDeg(){return (float)enc_pulse_last/GEAR_RATIO * REVOLUTION_DEGREE;};
    float getLastRad(){return (float)enc_pulse_last/GEAR_RATIO * REVOLUTION_RADIAN;};
    float getLastRev(){return (float)enc_pulse_last/GEAR_RATIO;};

    long getDeltaPulse(){return delta_enc_pulse;};
    long getDeltaTime(){return delta_time_ms;};
    float getDeltaDeg(){return (float)delta_enc_pulse/GEAR_RATIO * REVOLUTION_DEGREE;};
    float getDeltaRad(){return (float)delta_enc_pulse/GEAR_RATIO * REVOLUTION_RADIAN;};
    float getDeltaRev(){return (float)delta_enc_pulse/GEAR_RATIO;};

    float getOmegaPPS(){return enc_omega_pps;};
    float getOmegaDPS(){return enc_omega_pps * REVOLUTION_DEGREE/GEAR_RATIO;};
    float getOmegaRPS(){return enc_omega_pps * REVOLUTION_RADIAN/GEAR_RATIO;};
    float getOmegaRPM(){return enc_omega_pps * M_TO_S_CONVERTER/GEAR_RATIO;};

    void measureOmega();

    void doEncoderA();
    void doEncoderB();

    private:
    int enc_pin_a;
    int enc_pin_b;
    volatile long enc_pulse;
    long enc_pulse_last;
    long enc_pulse_current;
    long last_ms;
    long current_ms;
    long delta_enc_pulse;
    long delta_time_ms;
    float enc_omega_pps;
};

#endif
