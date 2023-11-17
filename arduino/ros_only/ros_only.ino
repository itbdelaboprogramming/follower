#include <ros.h>
#include <follower/TargetState.h>

// MT is motor, used to customize motor parameter
#define MT_MAX_PWM  120

// ML is motor on the left
#define ML_EN   7
#define ML_RPWM 9
#define ML_LPWM 10
#define MTL_ACT 13

// MR is motor on the right
#define MR_EN   8
#define MR_RPWM 6
#define MR_LPWM 5
#define MTR_ACT 4

// LED PIN
#define RED_LED  30
#define BLUE_LED 31

// Define Node Handler
ros::NodeHandle nh;

// Varible for target position and distaance
uint8_t target_position_ = 0;
uint8_t cam_angle_command_ = 0;
float target_distance_ = 0.0;
int16_t  pwm_r = 0, pwm_l = 0;

// Callback function that handles data subscribing
void callback_function(const follower::TargetState& msg){
  target_position_ = msg.target_position;
  target_distance_ = msg.target_distance;
  cam_angle_command_ = msg.cam_angle_command;
}

// Create subscriber for target info
ros::Subscriber<follower::TargetState> sub("rover_command", &callback_function);

void rotate_left(){
  // Write down the algorithm so the vehicle rotate left
  pwm_r = 1.0*MT_MAX_PWM;
  pwm_l = 1.0*MT_MAX_PWM;
}

void rotate_right(){
  // Write down the algorithm so the vehicle rotate right
  pwm_r = -1.0*MT_MAX_PWM;
  pwm_l = -1.0*MT_MAX_PWM;
}

void move_forward(){
  // Write down the algorithm so the vehicle move forward
  pwm_r = -1.0*MT_MAX_PWM;
  pwm_l = 1.0*MT_MAX_PWM;
}

void stop(){
  // Write down the algorithm so the vehicle stop moving
  pwm_r = 0;
  pwm_l = 0;
}

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  
  pinMode(MTL_ACT, OUTPUT);
  pinMode(ML_EN, OUTPUT);
  pinMode(ML_RPWM, OUTPUT);
  pinMode(ML_LPWM, OUTPUT);
  
  pinMode(MTR_ACT, OUTPUT);
  pinMode(MR_EN, OUTPUT);
  pinMode(MR_RPWM, OUTPUT);
  pinMode(MR_LPWM, OUTPUT);

  pinMode(RED_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);

  digitalWrite(MTL_ACT, HIGH);
  digitalWrite(MTR_ACT, HIGH);

}

void loop() {
  update_cmd();

  write_motor();
  
  nh.spinOnce();
  delay(25); //40 Hz
}

void update_cmd(){
    if (target_position_ == 1){
      rotate_left();
    } else if (target_position_ == 2){
      rotate_right();
    } else if (target_position_ == 3){
      move_forward();
    } else {
      stop();
    }
}


void write_motor(){
  // Rotate right motor
  if(pwm_r == 0){
    digitalWrite(MR_EN, LOW);
  }else if(pwm_r > 0){
    digitalWrite(MR_EN, HIGH);
    analogWrite(MR_RPWM, 0);
    analogWrite(MR_LPWM, pwm_r);
  }else{
    digitalWrite(MR_EN, HIGH);
    analogWrite(MR_LPWM, 0);
    analogWrite(MR_RPWM, -pwm_r);
  }
  
  // Rotate left motor
  if(pwm_l == 0){
    digitalWrite(ML_EN, LOW);
  }else if(pwm_l > 0){
    digitalWrite(ML_EN, HIGH);
    analogWrite(ML_RPWM, 0);
    analogWrite(ML_LPWM, pwm_l);
  }else{
    digitalWrite(ML_EN, HIGH);
    analogWrite(ML_LPWM, 0);
    analogWrite(ML_RPWM, -pwm_l);
  }
}
