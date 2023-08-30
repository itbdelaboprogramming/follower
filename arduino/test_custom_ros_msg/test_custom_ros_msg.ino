#include <ros.h>
#include <follower/TargetState.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

#include <RC_Receiver.h>

// RC is receiver with 4 PWM channel
#define RC_CH_COUNT 4
#define RC_CH1      48
#define RC_CH2      49
#define RC_CH3      47
#define RC_CH4      43 // used as failsafe

// MT is motor, used to customize motor parameter
#define MT_MAX_PWM  100

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

// ARMED and DISARMED
#define ARMED    0x00
#define DISARMED 0x01

// Uncomment line below to activate publisher to debugging
// #define DEBUG

// Define Node Handler
ros::NodeHandle nh;

// Varible for target position and distaance
uint16_t target_position_ = 0;
float target_distance_ = -1;

RC_Receiver receiver(RC_CH1, RC_CH2, RC_CH3, RC_CH4);
uint16_t pwm_in[RC_CH_COUNT + 1]; // this array starts from 1
long int pwm_offset[] = {0, 9, 18, 9, 9}; // this array starts from 1
int16_t  pwm_r = 0, pwm_l = 0, failsafe = DISARMED;

// Callback function that handles data subscribing
void callback_function( const follower::TargetState& msg){
  target_position_ = msg.target_position;
  target_distance_ = msg.target_distance;
}

// Create subscriber for target info
ros::Subscriber<follower::TargetState> sub("rover_command", callback_function);

#ifdef DEBUG
// Create publisher because when using rosserial you can't open serial monitor, so this is for debugging and to print out the value of variable that you want to check. The value will be accessible in ros terminal
std_msgs::String str_msg;
std_msgs::Float32 dis_msg;
ros::Publisher logger("from_arduino", &str_msg);
ros::Publisher dis_logger("dis_arduino", &dis_msg);
#endif
// Set the length of msg you want to send + 1
/* Example: 
 * L : Left
 * R : Right
 * C : Center
 * H : Hold
 * S : Stop
 * G : Go
 * 
 * The message will be LS, LG, RS, RG, CS, CG, HH
 * so the length of the message is 2 + 1 as null terminator
 */
unsigned char msg[3] = "HH";
/* Additional Note:
 * Avoid using String type data in Arduino, instead use array of char
 * If you want to use another data type change the data type of the message accordingly
 * Don't forget to import the data type from std_msgs
 */


// For timing purposes
uint32_t tlast = 0;
uint32_t period = 200;

void rotate_left(){
  // Write down the algorithm so the vehicle rotate left
  pwm_r = 0.6*MT_MAX_PWM;
  pwm_l = 0.6*MT_MAX_PWM;
}

void rotate_right(){
  // Write down the algorithm so the vehicle rotate right
  pwm_r = -0.6*MT_MAX_PWM;
  pwm_l = -0.6*MT_MAX_PWM;
}

void move_forward(){
  // Write down the algorithm so the vehicle move forward
  pwm_r = -0.6*MT_MAX_PWM;
  pwm_l = 0.6*MT_MAX_PWM;
}

void stop(){
  // Write down the algorithm so the vehicle stop moving
  pwm_r = 0;
  pwm_l = 0;
}

void setup() {
  // Initializing node
  nh.initNode();
  nh.subscribe(sub);

  #ifdef DEBUG
  nh.advertise(logger);
  nh.advertise(dis_logger);
  #endif

  // Write the rest of setup() code below
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
  // Write the rest of loop() code below
  // Get command from RC
  update_rc();

  // Update failsafe from RC input
  update_failsafe();

  // Update command from RC input
  update_cmd();

  // Write to motor
  write_motor();
  /*=============*/

  // Only for testing
  if (target_distance_ <= 100){
    msg[1] = 'S';
  } else {
    msg[1] = 'G';
  }

  /*==============*/

  // Here is an example of logging: to log data, you can convert the data into a char array (String) and provide the length of the data.
  if (millis() - tlast >= period){
    #ifdef DEBUG
    str_msg.data = msg;
    dis_msg.data = target_distance_;
    logger.publish(&str_msg);
    dis_logger.publish(&dis_msg);
    #endif
    tlast = millis();
  }
  
  
  nh.spinOnce();
  delay(1);
}

void update_rc(){
  for(byte i = 1; i <= 4; i++){
    pwm_in[i] = receiver.getRaw(i) + pwm_offset[i];
  }
}

void update_failsafe(){
  // It is chosen because if RC is disconnected, the value is 0
  if(pwm_in[4] > 1500){
    failsafe = ARMED;
  }else{
    failsafe = DISARMED;
  }
}

void update_cmd(){
  if(failsafe == DISARMED){ // Disarmed condition
    pwm_r = 0;
    pwm_l = 0;
    digitalWrite(RED_LED, LOW);
    digitalWrite(BLUE_LED, LOW);
  }else{ // Armed condition
    if(pwm_in[3] < 1600){
      int16_t cmd_front_back = map(pwm_in[1], 1000, 2000, -MT_MAX_PWM, MT_MAX_PWM);
      int16_t cmd_right_left = map(pwm_in[2], 1000, 2000, -MT_MAX_PWM, MT_MAX_PWM);
      pwm_r = cmd_right_left - cmd_front_back;
      pwm_l = cmd_right_left + cmd_front_back;
      digitalWrite(RED_LED, LOW);
      digitalWrite(BLUE_LED, HIGH);
    }else{
      // Part to control vehicle heading based on the target position
      if (target_position_ == 1 && target_distance_ > 150){
        rotate_left();
        msg[0] = 'L';
      } else if (target_position_ == 2){
        rotate_right();
        msg[0] = 'R';
      } else if (target_position_ == 3){
        move_forward();
        msg[0] = 'C';
      } else {
        stop();
        msg[0] = 'H';
      }
      digitalWrite(RED_LED, HIGH);
      digitalWrite(BLUE_LED, LOW);
    }
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
