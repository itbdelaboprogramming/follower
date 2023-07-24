#include <ros.h>
#include <follower/TargetState.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

// Uncomment line below to activate publisher to debugging
//#define DEBUG

// Define Node Handler
ros::NodeHandle nh;

// Varible for target position and distaance
uint16_t target_position_ = 0;
float target_distance_ = -1;

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

}

void rotate_right(){
  // Write down the algorithm so the vehicle rotate right

}

void move_forward(){
  // Write down the algorithm so the vehicle move forward

}

void stop(){
  // Write down the algorithm so the vehicle stop moving

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

}

void loop() {
  // Write the rest of loop() code below

  
  
  /*=============*/

  // Part to control vehicle heading based on the target position
  if (target_position_ == 1){
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

  // Only for testing
  if (target_distance_ <= 100){
    msg[1] = 'S';
  } else {
    msg[1] = 'G';
  }

  /*==============*/

  // Write the rest of loop() code below




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

