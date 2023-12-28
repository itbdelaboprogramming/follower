/*
The Arduino Program for ITBdeLabo's Prototype

Overview:
This Arduino code can take commands from both Radio Remote Control (RC Taranis X7) or Personal Computer (PC) via ROS communication message.
ROS communication message received by this Arduino code should in the form of "slam_itbdelabo/HardwareCommand" msg.
Please refer to SLAM_ITBdeLabo documentation to read more about SLAM_ITBdeLabo ROS package. (https://github.com/itbdelaboprogramming/SLAM_ITBdeLabo/tree/main)

Libraries (needed):
- <PinChangeInterrupt.h> : to change the encoder pins become interrupt pins
- <Wire.h>               : for I2C communication
- <Servo.h>              : to control servo
- <ros.h> and <slam_itbdelabo/HardwareCommand.h> : generated from "rosserial_arduino make_libraries.py ." and is used for ROS communication.

Debugging section: Contain blocks that are needed to monitor some variables. Uncomment blocks that you need to monitor in Serial Monitor. It can be one or multiple block.

In "setup()":
- setupPinReceiver() : Setup the RC Receiver pins as INPUT
- debugHeader()      : Print headers for debugging that depends on what block is active

In "loop()":
- getReceiverSignal() : Get the value of the RC signals. One channel per loop.
- calculatePose()     : Calculate robot poses based on wheel odometry and measure the wheel speed based on encoder readings.
- update_failsafe()   : Read RC ch.4 signal to determine the failsafe state. The prototype is ARMED only when RC ch.4 signal has value of more than 1400.
                        When the prototype is ARMED, it is ready to receive commands from both RC or PC
- update_cmd()        : Update the commands for the prototype. There are RC mode, hold mode, and PC commands mode depends the value of RC ch.3 signal.
                        Once the commands are received, PID controller will compute PWM value to drive the motors or the servo.
                        Some configurations are implemented to compensate the non-ideal condition. 
- debug()             : Print the value of some variables in Serial Monitor for debugging.
 
*/
#include <Wire.h>
#include <ros.h>
#include <follower/HardwareCommand.h>
#include <follower/HardwareState.h>
#include <Servo.h>

#include "Motor.h"
#include "Encoder.h"
#include "LPF.h"
#include "pidIr.h"

// For Debugging, uncomment one of these
//#define RECEIVER_RAW
//#define MOTOR_ANGLE_PULSE
//#define MOTOR_ANGLE_DEGREE
//#define MOTOR_ANGLE_RADIAN
//#define MOTOR_ANGLE_REVOLUTION
//#define MOTOR_SPEED_PPS
//#define MOTOR_SPEED_DPS
//#define MOTOR_SPEED_RPS
#define MOTOR_SPEED_RPM
#define TARGET_RPM
//#define PWM_RESPONSE
//#define VEHICLE_POSITION
//#define VEHICLE_SPEED
//#define EKF_DATA

// Receiver PIN
#define NUM_CH 5
#define PIN_CH_1 A8
#define PIN_CH_2 A9
#define PIN_CH_3 A10
#define PIN_CH_4 A11
#define PIN_CH_5 A12
#define PIN_CH_6 A13
#define PIN_CH_7 A14
#define PIN_CH_8 A15

// Motor PIN
#define RIGHT_MOTOR_REN_PIN 4
#define RIGHT_MOTOR_LEN_PIN 5
#define RIGHT_MOTOR_PWM_PIN 9

#define LEFT_MOTOR_REN_PIN 6
#define LEFT_MOTOR_LEN_PIN 7
#define LEFT_MOTOR_PWM_PIN 8

// Encoder PIN
#define RIGHT_ENC_PIN_A 50
#define RIGHT_ENC_PIN_B 51

#define LEFT_ENC_PIN_A 11
#define LEFT_ENC_PIN_B 10

// LED PIN
#define RED_LED  30
#define BLUE_LED 31

// SERVO
#define CAM_SERVO 3
#define MAX_SERVO_POS 175
#define MIN_SERVO_POS 125
#define INCREMENT_POS 10

// ULTRASONIC
#define ULT_SERIAL Serial2
#define BUFFER_LEN 11

// Constants
#define LOOP_TIME 25                    // in milliseconds
#define PERIOD_TIME 2*pow(10,6)         // in microseconds
#define RECEIVER_LPF_CUT_OFF_FREQ 0.25  // in Hertz (Hz)
#define ENC_LPF_CUT_OFF_FREQ 3          // in Hertz (Hz)
#define PWM_THRESHOLD 150               // in microseconds of receiver signal
#define MAX_RPM_MOVE 180                // in RPM for longitudinal movement
#define MAX_RPM_TURN 70                 // in RPM for rotational movement
#define WHEEL_RADIUS 2.75               // in cm
#define WHEEL_DISTANCE 23.0             // in cm
#define DISTANCE 200                    // in cm
#define MAX_PWM 250                     // saturation PWM for action control (0-255)
#define ARMED 0x00                      // armed condition
#define DISARMED 0x01                   // disarmed condition
#define HMC5983_ADDRESS 0x1E            // magnetometer I2C address

#define KP_RIGHT_MOTOR 5.0
#define KI_RIGHT_MOTOR 0.03
#define KD_RIGHT_MOTOR 0.0

#define KP_LEFT_MOTOR 5.0
#define KI_LEFT_MOTOR 0.03
#define KD_LEFT_MOTOR 0.0

Motor RightMotor(RIGHT_MOTOR_REN_PIN, RIGHT_MOTOR_LEN_PIN, RIGHT_MOTOR_PWM_PIN);
Motor LeftMotor(LEFT_MOTOR_LEN_PIN, LEFT_MOTOR_REN_PIN, LEFT_MOTOR_PWM_PIN);

Encoder RightEncoder(RIGHT_ENC_PIN_B, RIGHT_ENC_PIN_A); //it is inverted to get a right rotation sign
Encoder LeftEncoder(LEFT_ENC_PIN_A, LEFT_ENC_PIN_B);

LPF Ch_1_lpf(RECEIVER_LPF_CUT_OFF_FREQ);
LPF Ch_2_lpf(RECEIVER_LPF_CUT_OFF_FREQ);

LPF RightRPM_lpf(ENC_LPF_CUT_OFF_FREQ);
LPF LeftRPM_lpf(ENC_LPF_CUT_OFF_FREQ);

LPF dlpf(1);

pidIr RightMotorPID(KP_RIGHT_MOTOR, KI_RIGHT_MOTOR, KD_RIGHT_MOTOR);
pidIr LeftMotorPID(KP_LEFT_MOTOR, KI_LEFT_MOTOR, KD_LEFT_MOTOR);

// Magnetometer variables
struct magnetometer {
  int x_msb;
  int x_lsb;
  int z_msb;
  int z_lsb;
  int y_msb;
  int y_lsb;

  float hx;
  float hz;
  float hy;
}; magnetometer magnetometer;

// Encoder's callback functions
void callbackRA(){RightEncoder.doEncoderA();}
void callbackRB(){RightEncoder.doEncoderB();}
void callbackLA(){LeftEncoder.doEncoderA();}
void callbackLB(){LeftEncoder.doEncoderB();}

// Receiver signal variables
byte current_channel = 1;
uint16_t receiver_ch_value[9]; //PIN_CH_1 --> receiver_ch_value[1], and so on.
uint16_t receiver_ch_filtered[9]; //PIN_CH_1 --> receiver_ch_value[1], and so on.

// RPM variables
float right_rpm_filtered = 0;
float left_rpm_filtered = 0;
float right_rpm_target = 0;
float left_rpm_target = 0;

// Heading from magnetometer
float heading = 0;
float heading_filtered = 0;

// Action control
int move_value;
int turn_value;
int right_pwm = 0;
int left_pwm = 0;

// Failsafe
uint16_t failsafe = DISARMED;

// Vehicle Pose or State
float pose_x = 0;           // in cm
float pose_y = 0;           // in cm
float pose_theta = 0;       // in rad
float velocity_right = 0;   // in cm/s
float velocity_left = 0;    // in cm/s

// Timestamp variables
unsigned long time_now = 0;
unsigned long time_last = 0;
float dt;

// Ultrasonic Variables
char ultrasonic_data[BUFFER_LEN];
float direction  = 0.0;
float distance = 0.0;

// Servo
Servo camServo;
int servo_pos = MAX_SERVO_POS;

//ROS Communication
ros::NodeHandle nh;

// Varibles for hardware command
uint8_t movement_command_ = 0;
uint8_t cam_angle_command_ = 0;
float right_motor_speed_ = 0;
float left_motor_speed_ = 0;

// Create publisher for hardware state info
follower::HardwareState hardware_state_msg;
ros::Publisher hardware_state_pub("hardware_state", &hardware_state_msg);

// Callback function that handles data subscribing
void callback_function( const follower::HardwareCommand& msg){
  movement_command_ = msg.movement_command;
  cam_angle_command_ = msg.cam_angle_command;
}

// Create subscriber for hardware command info
ros::Subscriber<follower::HardwareCommand> sub("hardware_command", callback_function);

void setup(){
    Serial.begin(57600);
    Wire.begin();

    //Initiate ROS node
    nh.initNode();
    nh.advertise(hardware_state_pub);
    nh.subscribe(sub);
    
    RightMotor.begin();
    LeftMotor.begin();
    
    setupPinReceiver();

    RightEncoder.start(callbackRA, callbackRB);
    LeftEncoder.start(callbackLA, callbackLB);

    pinMode(RED_LED, OUTPUT);
    pinMode(BLUE_LED, OUTPUT);

    pinMode(RED_LED, OUTPUT);
    pinMode(BLUE_LED, OUTPUT);

    camServo.write(servo_pos);
    camServo.attach(CAM_SERVO);

    debugHeader();

    ULT_SERIAL.begin(9600); // software UART is slow, max baud rate = 38400

    delay(2000);
}

void loop(){
    time_now = millis();
    if(time_now - time_last >= LOOP_TIME){
        dt = time_now - time_last;
        getReceiverSignal();
        
        receiver_ch_filtered[1] = Ch_1_lpf.filter(receiver_ch_value[1], dt);
        receiver_ch_filtered[2] = Ch_2_lpf.filter(receiver_ch_value[2], dt);

        //Calculate the robot position and velocity
        calculatePose();

        update_failsafe();
        update_cmd();
        update_ultrasonic_data();
        
        time_last = time_now;
        debug();
        nh.spinOnce();
    }
}

void setupPinReceiver(){
    pinMode(PIN_CH_1, INPUT);
    pinMode(PIN_CH_2, INPUT);
    pinMode(PIN_CH_3, INPUT);
    pinMode(PIN_CH_4, INPUT);
    pinMode(PIN_CH_5, INPUT);
    pinMode(PIN_CH_6, INPUT);
    pinMode(PIN_CH_7, INPUT);
    pinMode(PIN_CH_8, INPUT);
}

void getReceiverSignal() {
    receiver_ch_value[current_channel] = pulseIn(getChannelPin(current_channel), HIGH, PERIOD_TIME);

    // Constrain the channel value within a specific range if needed
    receiver_ch_value[current_channel] = constrain(receiver_ch_value[current_channel], 1000, 2000);

    // Move to the next channel for the next loop iteration
    current_channel++;
    if (current_channel > NUM_CH) {
        current_channel = 1; // Reset to the first channel if all channels are processed
    }
}


int getChannelPin(byte channel) {
    switch (channel) {
        case 1:
            return PIN_CH_1;
        case 2:
            return PIN_CH_2;
        case 3:
            return PIN_CH_3;
        case 4:
            return PIN_CH_4;
        case 5:
            return PIN_CH_5;
        case 6:
            return PIN_CH_6;
        case 7:
            return PIN_CH_7;
        case 8:
            return PIN_CH_8;
        default:
            return -1;
    }
}

void update_failsafe(){
  // It is chosen because if RC is disconnected, the value is 0
  if(receiver_ch_value[4] > 1500){
    failsafe = ARMED;
  } else{
    failsafe = DISARMED;
  }
}

void update_cmd(){
  if(failsafe == DISARMED){             //Disarmed condition
    right_pwm = 0;
    left_pwm = 0;
    vehicle_stop();
    digitalWrite(RED_LED, LOW);
    digitalWrite(BLUE_LED, LOW);
  } else{                               //Armed condition
    if(receiver_ch_value[3] < 1400){      //RC operation
             
      //---------------------------Close loop-----------------------------//
      //Determine the move_value based on RC channel 1, added a minimum value threshold of PWM_THRESHOLD value for the prototype to be driven 
      if(receiver_ch_filtered[1] >= 1500 + PWM_THRESHOLD){  //Move forward
          //Re-scale the RC signal value from [1500+PWM_THRESHOLD, 2000] to [0,MAX_RPM_TURN]
          move_value = map(receiver_ch_filtered[1], 1500 + PWM_THRESHOLD, 2000, 0, MAX_RPM_MOVE);
      } else if(receiver_ch_filtered[1] <= 1500 - PWM_THRESHOLD){ //Move backward
          //Re-scale the RC signal value from [1500-PWM_THRESHOLD, 1000] to [0,-MAX_RPM_TURN+90]
          //+90 is added to compensate the unsymetrical value due to the encoder interrupts
          move_value = map(receiver_ch_filtered[1], 1500 - PWM_THRESHOLD, 1000, 0, -MAX_RPM_MOVE+90);
      } else {  //Stop
          move_value = 0;
      }

      //Determine the turn_value based on RC channel 2, added a minimum value threshold of PWM_THRESHOLD value for the prototype to be driven
      if(receiver_ch_filtered[2] >= 1450 + PWM_THRESHOLD){  //Turn right
          //Re-scale the RC signal value from [1450+PWM_THRESHOLD, 1950] to [0,MAX_RPM_TURN+80]
          //+80 is added to compensate the unsymetrical value due to the encoder interrupts
          turn_value = map(receiver_ch_filtered[2], 1450 + PWM_THRESHOLD, 1950, 0, MAX_RPM_TURN+80);
      } else if(receiver_ch_filtered[2] <= 1450 - PWM_THRESHOLD){ //Turn left
          //Re-scale the RC signal value from [1450-PWM_THRESHOLD, 1000] to [0,-MAX_RPM_TURN]
          turn_value = map(receiver_ch_filtered[2], 1450 - PWM_THRESHOLD, 1000, 0, -MAX_RPM_TURN);
      } else {  //Stop
          turn_value = 0;
      }
      
      //Compute the RPM target for each motor
      right_rpm_target = move_value - turn_value;
      left_rpm_target = move_value + turn_value;

      //Compute the action control (PWM) value for the motor based on PID and set it to be zero if the command is truely zero
      if(right_rpm_target == 0 && left_rpm_target == 0){
          right_pwm = 0;
          left_pwm = 0;
          vehicle_stop();
      } else if (right_rpm_target == 0 && left_rpm_target != 0) {
          right_pwm = 0;
          RightMotorPID.reset();
          left_pwm = LeftMotorPID.compute(left_rpm_target, left_rpm_filtered, MAX_PWM, dt);
      } else if (right_rpm_target != 0 && left_rpm_target == 0) {
          left_pwm = 0;
          LeftMotorPID.reset();
          right_pwm = RightMotorPID.compute(right_rpm_target, right_rpm_filtered, MAX_PWM, dt);
      } else {
          right_pwm = RightMotorPID.compute(right_rpm_target, right_rpm_filtered, MAX_PWM, dt);
          left_pwm = LeftMotorPID.compute(left_rpm_target, left_rpm_filtered, MAX_PWM, dt);
      }
      //------------------------------------------------------------------//
      
      digitalWrite(RED_LED, LOW);
      digitalWrite(BLUE_LED, HIGH);
    } else {  //PC commands operation
      //Part to control motor target based on the PC commands
      right_rpm_target = right_motor_speed_;
      left_rpm_target = left_motor_speed_;

      //Compute the action control (PWM) value for the motor based on PID and set it to be zero if the command is truely zero
//      if(right_rpm_target == 0 && left_rpm_target == 0){
//          right_pwm = 0;
//          left_pwm = 0;
//          vehicle_stop();
//      } else if (right_rpm_target == 0 && left_rpm_target != 0) {
//          right_pwm = 0;
//          RightMotorPID.reset();
//          left_pwm = LeftMotorPID.compute(left_rpm_target, left_rpm_filtered, MAX_PWM, dt);
//      } else if (right_rpm_target != 0 && left_rpm_target == 0) {
//          left_pwm = 0;
//          LeftMotorPID.reset();
//          right_pwm = RightMotorPID.compute(right_rpm_target, right_rpm_filtered, MAX_PWM, dt);
//      } else {
//          right_pwm = RightMotorPID.compute(right_rpm_target, right_rpm_filtered, MAX_PWM, dt);
//          left_pwm = LeftMotorPID.compute(left_rpm_target, left_rpm_filtered, MAX_PWM, dt);
//      }
      if (movement_command_ == 1) {
        // Right
        right_pwm = -MAX_PWM;
        left_pwm = MAX_PWM;
      }
      else if (movement_command_ == 2) {
        // Left
        right_pwm = MAX_PWM;
        left_pwm = -MAX_PWM;
      }
      else if (movement_command_ == 3) {
        // Forward
        right_pwm = MAX_PWM;
        left_pwm = MAX_PWM;
      }
      else {
        // Stop
        right_pwm = 0;
        left_pwm = 0;
      }
      digitalWrite(RED_LED, HIGH);
      digitalWrite(BLUE_LED, LOW);
    }

    //Drive the motors based on the PWM values that have been computed
    vehicleGo(right_pwm, left_pwm);
    write_servo();
  }
}


void vehicle_stop(){
    RightMotor.stop();
    LeftMotor.stop();
    resetPID();
}

void vehicleGo(int right_pwm, int left_pwm){
    RightMotor.rotate(right_pwm);
    LeftMotor.rotate(left_pwm);
}

void resetPID(){
    RightMotorPID.reset();
    LeftMotorPID.reset();
}

void calculatePose(){
    float delta_angle_right = RightEncoder.getDeltaRad();
    float delta_angle_left = LeftEncoder.getDeltaRad();

    // Calculate robot poses based on wheel odometry
    pose_x = pose_x + WHEEL_RADIUS/2.0 * (delta_angle_right + delta_angle_left) * sin(pose_theta);
    pose_y = pose_y + WHEEL_RADIUS/2.0 * (delta_angle_right + delta_angle_left) * cos(pose_theta);
    pose_theta = pose_theta + (delta_angle_right - delta_angle_left) * WHEEL_RADIUS/WHEEL_DISTANCE;
    
    // Wrap the orientation angle
    pose_theta = wrapAngleRadian(pose_theta);

    // Measure the wheel speed
    RightEncoder.measureOmega();
    LeftEncoder.measureOmega();

    // Filter the measurement of the wheel speed
    right_rpm_filtered = RightRPM_lpf.filter(RightEncoder.getOmegaRPM(), dt);
    left_rpm_filtered = LeftRPM_lpf.filter(LeftEncoder.getOmegaRPM(), dt);

    velocity_right = right_rpm_filtered * PI/30.0 * WHEEL_RADIUS;
    velocity_left = left_rpm_filtered * PI/30.0 * WHEEL_RADIUS;
}

float wrapAngleDegree(float value){
    if(value >= 2*360){
        return wrapAngleDegree(value - 360);
    } else if(value < 0){
        return wrapAngleDegree(value + 360);
    } else {
        return value;
    }
}

float wrapAngleRadian(float value){
    if(value >= 2*PI){
        return wrapAngleRadian(value - 2 * PI);
    } else if(value < 0){
        return wrapAngleRadian(value + 2 * PI);
    } else {
        return value;
    }
}

void write_hmc5983(int reg, int val){
    Wire.beginTransmission(HMC5983_ADDRESS);
    Wire.write(0x3C);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

void read_hmc5983(int reg){
    Wire.beginTransmission(HMC5983_ADDRESS);
    Wire.write(0x3D);
    Wire.write(reg);
    Wire.endTransmission();
}

void get_heading(){
    write_hmc5983(0x00, 0x10);
    write_hmc5983(0x01, 0x20);
    write_hmc5983(0x02, 0x01);
    read_hmc5983(0x03);
  
    Wire.requestFrom(HMC5983_ADDRESS, 6);
    while(Wire.available()){
        magnetometer.x_msb = Wire.read();
        magnetometer.x_lsb = Wire.read();
        magnetometer.z_msb = Wire.read();
        magnetometer.z_lsb = Wire.read();
        magnetometer.y_msb = Wire.read();
        magnetometer.y_lsb = Wire.read();
    }

    magnetometer.hx = (magnetometer.x_msb << 8) + magnetometer.x_lsb;
    magnetometer.hz = (magnetometer.z_msb << 8) + magnetometer.z_lsb;
    magnetometer.hy = (magnetometer.y_msb << 8) + magnetometer.y_lsb;

    if(magnetometer.hx > 0x07FF) magnetometer.hx = 0xFFFF - magnetometer.hx;
    if(magnetometer.hz > 0x07FF) magnetometer.hz = 0xFFFF - magnetometer.hz;
    if(magnetometer.hy > 0x07FF) magnetometer.hy = 0xFFFF - magnetometer.hy;

    heading = atan2(magnetometer.hy, magnetometer.hx) * 180 / PI; //in deg
    heading_filtered = dlpf.filter(heading, dt);
}

void write_servo(){
  // RC mode
  if (receiver_ch_value[3] < 1600) {
    if (receiver_ch_value[5] >= 980 && receiver_ch_value[5] <= 2020){
      servo_pos = map(receiver_ch_value[5], 980, 2020, MIN_SERVO_POS + 5, MAX_SERVO_POS - 5);
      camServo.write(servo_pos);
    } else if (receiver_ch_value[5] > 2020) {
      // most up cam position
      servo_pos = MAX_SERVO_POS;
      camServo.write(servo_pos);
    } else {
      // most straight cam position
      servo_pos = MIN_SERVO_POS;
      camServo.write(servo_pos);
    }
  }
  // PC Mode (Auto)
  else {
    if (cam_angle_command_ == 1) {
      // Up
      servo_pos = servo_pos + INCREMENT_POS;
      servo_pos = min(servo_pos, MAX_SERVO_POS);
      camServo.write(servo_pos);
    }
    else if (cam_angle_command_ == 2) {
      // Down
      servo_pos = servo_pos - INCREMENT_POS;
      servo_pos = max(servo_pos, MIN_SERVO_POS);
      camServo.write(servo_pos);
    }
  }
}

void update_ultrasonic_data(){
  if(ULT_SERIAL.available()){
    ULT_SERIAL.readBytes(ultrasonic_data, 11);
    direction  = atof(strtok(ultrasonic_data, ","));
    distance = atof(strtok(NULL, ",")) / 32.0;
  }
  hardware_state_msg.ultrasonic_target_direction = direction;
  hardware_state_msg.ultrasonic_target_distance = distance;
  hardware_state_pub.publish(&hardware_state_msg);
}

void debugHeader(){
    #ifdef RECEIVER_RAW
    Serial.print(F("Ch1:")); Serial.print("\t");
    Serial.print(F("Ch2:")); Serial.print("\t");
    Serial.print(F("Ch3:")); Serial.print("\t");
    Serial.print(F("Ch4:")); Serial.print("\t");
    #endif

    #ifdef MOTOR_ANGLE_PULSE
    Serial.print(F("Right_Pulse:")); Serial.print("\t");
    Serial.print(F("Left_Pulse:")); Serial.print("\t");
    #endif

    #ifdef MOTOR_ANGLE_DEGREE
    Serial.print(F("Right_Deg:")); Serial.print("\t");
    Serial.print(F("Left_Deg:")); Serial.print("\t");
    #endif

    #ifdef MOTOR_ANGLE_RADIAN
    Serial.print(F("Right_Rad:")); Serial.print("\t");
    Serial.print(F("Left_Rad:")); Serial.print("\t");
    #endif

    #ifdef MOTOR_ANGLE_REVOLUTION
    Serial.print(F("Right_Rev:")); Serial.print("\t");
    Serial.print(F("Left_Rev:")); Serial.print("\t");
    #endif

    #ifdef MOTOR_SPEED_PPS
    Serial.print(F("Right_PPS:")); Serial.print("\t");
    Serial.print(F("Left_PPS:")); Serial.print("\t");
    #endif

    #ifdef MOTOR_SPEED_DPS
    Serial.print(F("Right_DPS:")); Serial.print("\t");
    Serial.print(F("Left_DPS:")); Serial.print("\t");
    #endif

    #ifdef MOTOR_SPEED_RPS
    Serial.print(F("Right_RPS:")); Serial.print("\t");
    Serial.print(F("Left_RPS:")); Serial.print("\t");
    #endif

    #ifdef MOTOR_SPEED_RPM
    Serial.print(F("Right_RPM:")); Serial.print("\t");
    Serial.print(F("Left_RPM:")); Serial.print("\t");
    #endif

    #ifdef TARGET_RPM
    Serial.print(F("Right_Target")); Serial.print("\t");
    Serial.print(F("Left_Target")); Serial.print("\t");
    #endif

    #ifdef PWM_RESPONSE
    Serial.print(F("Right_PWM")); Serial.print("\t");
    Serial.print(F("Left_PWM")); Serial.print("\t");
    #endif

    #ifdef ERROR_PID
    Serial.print(F("Error_Right")); Serial.print("\t");
    Serial.print(F("Error_Left")); Serial.print("\t");
    #endif

    #ifdef SUM_ERROR_PID
    Serial.print(F("Sum_Error_Right")); Serial.print("\t");
    Serial.print(F("Sum_Error_Left")); Serial.print("\t");
    #endif

    #ifdef VEHICLE_POSITION
    Serial.print(F("Vehicle_X_Pose")); Serial.print("\t");
    Serial.print(F("Vehicle_Y_Pose")); Serial.print("\t");
    Serial.print(F("Vehicle_Theta")); Serial.print("\t");
    #endif

    #ifdef VEHICLE_SPEED
    Serial.print(F("Vehicle_Speed_Right")); Serial.print("\t");
    Serial.print(F("Vehicle_Speed_Left")); Serial.print("\t");
    #endif

    #ifdef MOTOR_PULSE_DIFFERENCE
    Serial.print(F("Right_Pulse_diffference")); Serial.print("\t");
    Serial.print(F("Left_Pulse_diffference")); Serial.print("\t");
    #endif

    #ifdef EKF_DATA
    Serial.print(F("Right_Wheel_Speed")); Serial.print(",");
    Serial.print(F("Left_Wheel_Speed")); Serial.print(",");
    Serial.print(F("EKF_State")); Serial.print(",");
    #endif

    Serial.println();
}

void debug(){
    #ifdef RECEIVER_RAW
    Serial.print(receiver_ch_value[1]); Serial.print("\t");
    Serial.print(receiver_ch_value[2]); Serial.print("\t");
    Serial.print(receiver_ch_value[3]); Serial.print("\t");
    Serial.print(receiver_ch_value[4]); Serial.print("\t");
    #endif

    #ifdef MOTOR_ANGLE_PULSE
    Serial.print(RightEncoder.getPulse()); Serial.print("\t");
    Serial.print(LeftEncoder.getPulse()); Serial.print("\t");
    #endif

    #ifdef MOTOR_ANGLE_DEGREE
    Serial.print(RightEncoder.getAngleDeg()); Serial.print("\t");
    Serial.print(LeftEncoder.getAngleDeg()); Serial.print("\t");
    #endif

    #ifdef MOTOR_ANGLE_RADIAN
    Serial.print(RightEncoder.getAngleRad()); Serial.print("\t");
    Serial.print(LeftEncoder.getAngleRad()); Serial.print("\t");
    #endif

    #ifdef MOTOR_ANGLE_REVOLUTION
    Serial.print(RightEncoder.getAngleRev()); Serial.print("\t");
    Serial.print(LeftEncoder.getAngleRev()); Serial.print("\t");
    #endif

    #ifdef MOTOR_SPEED_PPS
    Serial.print(RightEncoder.getOmegaPPS()); Serial.print("\t");
    Serial.print(LeftEncoder.getOmegaPPS()); Serial.print("\t");
    #endif

    #ifdef MOTOR_SPEED_DPS
    Serial.print(RightEncoder.getOmegaDPS()); Serial.print("\t");
    Serial.print(LeftEncoder.getOmegaDPS()); Serial.print("\t");
    #endif

    #ifdef MOTOR_SPEED_RPS
    Serial.print(RightEncoder.getOmegaRPS()); Serial.print("\t");
    Serial.print(LeftEncoder.getOmegaRPS()); Serial.print("\t");
    #endif

    #ifdef MOTOR_SPEED_RPM
    Serial.print(right_rpm_filtered); Serial.print("\t");
    Serial.print(left_rpm_filtered); Serial.print("\t");
    #endif

    #ifdef TARGET_RPM
    Serial.print(right_rpm_target); Serial.print("\t");
    Serial.print(left_rpm_target); Serial.print("\t");
    #endif

    #ifdef PWM_RESPONSE
    Serial.print(right_pwm/2.0); Serial.print("\t");
    Serial.print(left_pwm/2.0); Serial.print("\t");
    #endif

    #ifdef ERROR_PID
    Serial.print(RightMotorPID.getError()); Serial.print("\t");
    Serial.print(LeftMotorPID.getError()); Serial.print("\t");
    #endif

    #ifdef SUM_ERROR_PID
    Serial.print(RightMotorPID.getSumError()); Serial.print("\t");
    Serial.print(LeftMotorPID.getSumError()); Serial.print("\t");
    #endif

    #ifdef VEHICLE_POSITION
    Serial.print(pose_x); Serial.print("\t");
    Serial.print(pose_y); Serial.print("\t");
    Serial.print(pose_theta/PI*180.0); Serial.print("\t");
    #endif
    
    #ifdef VEHICLE_SPEED
    Serial.print(velocity_right); Serial.print("\t");
    Serial.print(velocity_left); Serial.print("\t");
    #endif

    #ifdef MOTOR_PULSE_DIFFERENCE
    Serial.print(RightEncoder.getDeltaPulse()); Serial.print("\t");
    Serial.print(LeftEncoder.getDeltaPulse()); Serial.print("\t");
    #endif

    #ifdef EKF_DATA
    Serial.print(velocity_right); Serial.print(",");
    Serial.print(velocity_left); Serial.print(",");
    #endif

    Serial.println();
}