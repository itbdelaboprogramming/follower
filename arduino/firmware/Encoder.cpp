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

#include <PinChangeInterrupt.h>
#include "Encoder.h"

Encoder::Encoder(int pin_a, int pin_b){
    enc_pin_a = pin_a;
    enc_pin_b = pin_b;
    enc_pulse = 0;
    enc_pulse_last = 0;
    last_ms = 0;
    enc_omega_pps = 0;
};

void Encoder::start(void(*userFuncA)(void),void(*userFuncB)(void)){
    // Print information that encoder reading is starting
    //Serial.println("Encoder Begin");

    // Set the pin as input pin Arduino
    pinMode(getPinA(), INPUT_PULLUP);
    pinMode(getPinB(), INPUT_PULLUP);
    
    // Interrupt when the encoder pulse changes, then do "userFunc" functions
    if ((getPinA() == 10 || getPinA() == 11 || getPinA() == 12) && (getPinB() == 10 || getPinB() == 11 || getPinB() == 12)){
      attachPCINT(digitalPinToPCINT(getPinA()), userFuncA, CHANGE);
      attachPCINT(digitalPinToPCINT(getPinB()), userFuncB, CHANGE);
    } else if ((getPinA() == 2 || getPinA() == 3) && (getPinB() == 2 || getPinB() == 3)){
      attachInterrupt(digitalPinToInterrupt(getPinA()), userFuncA, CHANGE);
      attachInterrupt(digitalPinToInterrupt(getPinB()), userFuncB, CHANGE);
    } else {
      //Serial.print("Get the right pin.");
      attachPCINT(digitalPinToPCINT(getPinA()), userFuncA, CHANGE);
      attachPCINT(digitalPinToPCINT(getPinB()), userFuncB, CHANGE);
    }   
};

void Encoder::measureOmega(){
    enc_pulse_current = enc_pulse;
    current_ms = millis();

    delta_enc_pulse = enc_pulse_current - enc_pulse_last;
    delta_time_ms = current_ms -  last_ms;

    enc_omega_pps = (float)delta_enc_pulse/delta_time_ms * S_TO_MS_CONVERTER;

    enc_pulse_last = enc_pulse_current;
    last_ms = current_ms;
};

void Encoder::doEncoderA(){
    if (digitalRead(enc_pin_a) == HIGH){
        if (digitalRead(enc_pin_b) == LOW){
            enc_pulse = enc_pulse + 1;          // CW
        } else {
            enc_pulse = enc_pulse - 1;          // CCW
        }
    } else {
        if (digitalRead(enc_pin_b) == HIGH){
            enc_pulse = enc_pulse + 1;          // CW
        } else {
            enc_pulse = enc_pulse - 1;          // CCW
        }
    }
};

void Encoder::doEncoderB(){
    if (digitalRead(enc_pin_b) == HIGH){
        if (digitalRead(enc_pin_a) == HIGH){
            enc_pulse = enc_pulse + 1;          // CW
        } else {
            enc_pulse = enc_pulse - 1;          // CCW
        }
    } else {
        if (digitalRead(enc_pin_a) == LOW){
            enc_pulse = enc_pulse + 1;          // CW
        } else {
            enc_pulse = enc_pulse - 1;          // CCW
        }
    }
};
