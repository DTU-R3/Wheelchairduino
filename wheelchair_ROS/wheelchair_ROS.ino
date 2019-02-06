#include <ros.h>
#include <math.h> 
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

#define FWD_L_PWM 22
#define BACK_L_PWM 23
#define FWD_R_PWM 20
#define BACK_R_PWM 21
#define FREQ 25000

// Distance between wheels in meters
#define WHEELBASE 0.48

// Variables
ros::NodeHandle  nh;

float leftSpeed = 0;
float rightSpeed = 0;
int leftCmd = 0;
int rightCmd = 0;
int minpower = 40;
float Eb = 1.25;
float Eb2 = 1.34;
float front_range = 255.0;
float left_range = 255.0;
float right_range = 255.0;
// Cofficient to map velocity to motor signal, m/s to PWM
float k = 50.0;
// Valid threshold for laser scan 
float slow_thres = 2.0;
float stop_thres = 0.5;
float turn_thres = 0.7;
float reactive_vel = 0.1;

// 'cmd_vel' callback function
void velCB( const geometry_msgs::Twist& vel) {
  float vel_x = vel.linear.x;
  // Slow down if obstacle in front far away
  if ((front_range < slow_thres)&&(front_range > stop_thres)) {
    float max_linear_x = (front_range-stop_thres)/(slow_thres-stop_thres)*vel_x;
    if (vel_x > max_linear_x) {
      vel_x = max_linear_x;
    }
  }
  
  // Calculate the velocity for each wheel
  leftSpeed = vel_x - vel.angular.z * 0.5;
  rightSpeed = vel_x + vel.angular.z * 0.5;

  // Reactive control
  // Work only when moving forward
  if (vel_x > 0) {
    if (front_range <= stop_thres) {
      leftSpeed = 0;
      rightSpeed = 0;
    }
    else if (left_range < turn_thres) {
      leftSpeed = reactive_vel;
      rightSpeed = -reactive_vel;
    }   
    else if (right_range < turn_thres) {
      leftSpeed = -reactive_vel;
      rightSpeed = reactive_vel;
    }   
  }
  
  
  // Map the velocity to motor signal
  if (leftSpeed > 0) {
    leftCmd = (int) (Eb*((leftSpeed * k) + minpower));
  }
  else if(leftSpeed < 0) {
    leftCmd = (int) (Eb2 * ((leftSpeed * k) - minpower));
  }
  else {
    leftCmd = 0;
  }
  if (rightSpeed != 0) {
    rightCmd = (int) ((rightSpeed * k) + rightSpeed/abs(rightSpeed)*minpower);
  }
  else {
    rightCmd = 0;
  }
}

// 'k_vel' callback function
void kCB( const std_msgs::Float32& f) {
  k = f.data;
}

// 'minpower' callback function
void minCB( const std_msgs::Int32& i) {
  minpower = i.data;
}

// 'wheelchair/front' callback function
void frontCB( const std_msgs::Float32& f) {
  if (isinf(f.data)) {
    front_range = stop_thres;
  }
  else {
    front_range = f.data;
  }
}

// 'wheelchair/left' callback function
void leftCB( const std_msgs::Float32& f) {
  if (isinf(f.data)) {
    left_range = stop_thres;
  }
  else {
    left_range = f.data;
  }
}

// 'wheelchair/right' callback function
void rightCB( const std_msgs::Float32& f) {
  if (isinf(f.data)) {
    right_range = stop_thres;
  }
  else {
    right_range = f.data;
  }
}

ros::Subscriber<geometry_msgs::Twist> velSub("cmd_vel", &velCB );
ros::Subscriber<std_msgs::Float32> kSub("k_vel", &kCB );
ros::Subscriber<std_msgs::Int32> minSub("min_power", &minCB );
ros::Subscriber<std_msgs::Float32> frontSub("wheelchair/front", &frontCB );
ros::Subscriber<std_msgs::Float32> leftSub("wheelchair/left", &leftCB );
ros::Subscriber<std_msgs::Float32> rightSub("wheelchair/right", &rightCB );

void setup() {
  
	// Set pin mode
	pinMode(FWD_L_PWM, OUTPUT);
	pinMode(BACK_L_PWM, OUTPUT);
	pinMode(FWD_R_PWM, OUTPUT);
	pinMode(BACK_R_PWM, OUTPUT);
	analogWriteFrequency(FWD_L_PWM, FREQ);
  analogWriteFrequency(BACK_L_PWM, FREQ);
  analogWriteFrequency(FWD_R_PWM, FREQ);
  analogWriteFrequency(BACK_R_PWM, FREQ);
  
	// Init all pins
	analogWrite(FWD_L_PWM, 0);
	analogWrite(BACK_L_PWM, 0);
	analogWrite(FWD_R_PWM, 0);
	analogWrite(BACK_R_PWM, 0);
	delay(1000);

  nh.initNode();
  nh.subscribe(velSub);
  nh.subscribe(kSub);
  nh.subscribe(minSub);
  nh.subscribe(frontSub);
  nh.subscribe(leftSub);
  nh.subscribe(rightSub);
}

void loop() {
  // Assign cmd to the motor
  if ((leftCmd > 0) && (leftCmd < 255)) {
    analogWrite(FWD_L_PWM, leftCmd);
    analogWrite(BACK_L_PWM, 0);
  }
  else if ((leftCmd < 0) && (leftCmd > -255)) {
    analogWrite(FWD_L_PWM, 0);
    analogWrite(BACK_L_PWM, -leftCmd);
  }
  else {
    analogWrite(FWD_L_PWM, 0);
    analogWrite(BACK_L_PWM, 0);
    analogWrite(FWD_R_PWM, 0);
    analogWrite(BACK_R_PWM, 0);
  }

  if ((rightCmd > 0) && (rightCmd < 255)) {
    analogWrite(FWD_R_PWM, rightCmd);
    analogWrite(BACK_R_PWM, 0);
  }
  else if ((rightCmd < 0) && (rightCmd > -255)) {
    analogWrite(FWD_R_PWM, 0);
    analogWrite(BACK_R_PWM, -rightCmd);
  }
  else {
    analogWrite(FWD_L_PWM, 0);
    analogWrite(BACK_L_PWM, 0);
    analogWrite(FWD_R_PWM, 0);
    analogWrite(BACK_R_PWM, 0);
  } 
  nh.spinOnce();
  delay(1);
}
