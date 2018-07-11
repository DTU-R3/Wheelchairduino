#include <ros.h>
#include <math.h> 
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

// Pins
#define L_FWD 3
#define L_BACK 4
#define R_FWD 6
#define R_BACK 5
#define L_SENSE 7
#define R_SENSE 8

#define FREQ 25000

// Robot parameter
#define WHEELBASE 0.228
#define METERPERCOUNT 0.0000439
#define SPEEDTOCMD 543.48
#define MIN_POWER 11
#define FWD_EB 1.05
#define BACK_EB 1.065
#define MAXSPEED (200.0-MIN_POWER)/SPEEDTOCMD

// Variables
ros::NodeHandle  nh;
float leftSpeed = 0;
float rightSpeed = 0;
int leftCmd = 0;
int rightCmd = 0;
int minpower = 60;
int leftCount = 0;
int rightCount = 0;
int leftDir = 0;
int rightDir = 0;

// 'cmd_vel' callback function
void velCB( const geometry_msgs::Twist& vel) {
  float lin_vel = vel.linear.x;
  float ang_vel = vel.angular.z;

  // Handle speed limit
  if ((lin_vel>0)&&((lin_vel+abs(ang_vel)*WHEELBASE*0.5)>MAXSPEED)) {
    lin_vel = MAXSPEED - abs(ang_vel)*WHEELBASE*0.5;
  }
  else if ((lin_vel<0)&&((-lin_vel+abs(ang_vel)*WHEELBASE*0.5)>MAXSPEED)) {
    lin_vel = -MAXSPEED + abs(ang_vel)*WHEELBASE*0.5;
  }
  
  // Calculate the velocity for each wheel
  leftSpeed = lin_vel - ang_vel * 0.5 * WHEELBASE;
  rightSpeed = lin_vel + ang_vel * 0.5 * WHEELBASE;

    
  // Map the velocity to motor signal
  if (leftSpeed > 0) {
    leftCmd = (int) (FWD_EB * ((leftSpeed * SPEEDTOCMD) + MIN_POWER));
    leftDir = 1;
    analogWrite(L_FWD, abs(leftCmd));
    analogWrite(L_BACK, 0);
  }
  else if(leftSpeed < 0) {
    leftCmd = (int) (BACK_EB * ((leftSpeed * SPEEDTOCMD) - MIN_POWER));
    leftDir = -1;
    analogWrite(L_FWD, 0);
    analogWrite(L_BACK, abs(leftCmd));
  }
  else {
    leftCmd = 0;
    leftDir = 0;
    analogWrite(L_FWD, 0);
    analogWrite(L_BACK, 0);
  }
  
  if (rightSpeed > 0) {
    rightCmd = (int) ((rightSpeed * SPEEDTOCMD) + MIN_POWER);
    rightDir = 1;
    analogWrite(R_FWD, abs(rightCmd));
    analogWrite(R_BACK, 0);
  }
  else if (rightSpeed < 0) {
    rightCmd = (int) ((rightSpeed * SPEEDTOCMD) - MIN_POWER);
    rightDir = -1;
    analogWrite(R_FWD, 0);
    analogWrite(R_BACK, abs(rightCmd));
  }
  else {
    rightCmd = 0;
    rightDir = 0;
    analogWrite(R_FWD, 0);
    analogWrite(R_BACK, 0);
  }
}

void resetCB( const std_msgs::Bool& b) {
  if (b.data) {
    leftCount = 0;
    rightCount = 0;
  }
}

std_msgs::Int32 left_count;
std_msgs::Int32 right_count;
ros::Publisher leftPub("padbot/left_count", &left_count);
ros::Publisher rightPub("padbot/right_count", &right_count);
ros::Subscriber<geometry_msgs::Twist> velSub("cmd_vel", &velCB );
ros::Subscriber<std_msgs::Bool> resetSub("padbot/reset_encoder", &resetCB );

void setup() {
	// Config
  pinMode(L_FWD, OUTPUT);
  pinMode(L_BACK, OUTPUT);
  pinMode(R_FWD, OUTPUT);
  pinMode(R_BACK, OUTPUT);
  pinMode(L_SENSE, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(L_SENSE), L_Encoder, CHANGE); 
  pinMode(R_SENSE, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(R_SENSE), R_Encoder, CHANGE); 
  analogWriteFrequency(L_FWD, FREQ);
  analogWriteFrequency(L_BACK, FREQ);
  analogWriteFrequency(R_FWD, FREQ);
  analogWriteFrequency(R_BACK, FREQ);
  delay(1000);
	
	// Init all pins
	analogWrite(L_FWD, 0);
	analogWrite(L_BACK, 0);
	analogWrite(R_FWD, 0);
	analogWrite(R_BACK, 0);
	delay(1000);
  nh.initNode();
  nh.advertise(leftPub);
  nh.advertise(rightPub);
  nh.subscribe(velSub);
  nh.subscribe(resetSub);
}

void L_Encoder() { 
  leftCount += leftDir; 
} 
 
void R_Encoder() { 
  rightCount += rightDir; 
} 

void loop() {
  left_count.data = leftCount;
  leftPub.publish(&left_count );
  right_count.data = rightCount;
  rightPub.publish(&right_count );
  nh.spinOnce();
  delay(100);
}
