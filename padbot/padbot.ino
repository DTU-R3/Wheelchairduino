#include <ros.h>
#include <math.h> 
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

// Pins
#define L_FWD 20
#define L_BACK 19
#define L_SENSE 18
#define L_EN 23

#define R_FWD 17
#define R_BACK 16
#define R_SENSE 15
#define R_EN 22

#define FREQ 25000

// Robot parameter
#define WHEELBASE 0.48
#define METERPERCOUNT 0.003

// Variables
ros::NodeHandle  nh;
float leftSpeed = 0;
float rightSpeed = 0;
int leftCmd = 0;
int rightCmd = 0;
int minpower = 30;
float Eb = 1.00;
float Eb2 = 1.00;
float k = 50.0;
float max_speed = 1.0;

// 'cmd_vel' callback function
void velCB( const geometry_msgs::Twist& vel) {
  float lin_vel = vel.linear.x;
  float ang_vel = vel.angular.z;

  // Handle speed limit
  if ((lin_vel>0)&&((lin_vel+abs(ang_vel)*WHEELBASE*0.5)>max_speed)) {
    lin_vel = max_speed - abs(ang_vel)*WHEELBASE*0.5;
  }
  else if ((lin_vel<0)&&((-lin_vel+abs(ang_vel)*WHEELBASE*0.5)>max_speed)) {
    lin_vel = -max_speed + abs(ang_vel)*WHEELBASE*0.5;
  }
  
  // Calculate the velocity for each wheel
  leftSpeed = lin_vel - ang_vel * 0.5 * WHEELBASE;
  rightSpeed = lin_vel + ang_vel * 0.5 * WHEELBASE;

    
  // Map the velocity to motor signal
  if (leftSpeed > 0) {
    leftCmd = (int) (Eb * ((leftSpeed * k) + minpower));
    digitalWrite(L_FWD, HIGH);
    digitalWrite(L_BACK, LOW);
  }
  else if(leftSpeed < 0) {
    leftCmd = (int) (Eb2 * ((leftSpeed * k) - minpower));
    digitalWrite(L_FWD, LOW);
    digitalWrite(L_BACK, HIGH);
  }
  else {
    leftCmd = 0;
    digitalWrite(L_FWD, LOW);
    digitalWrite(L_BACK, LOW);
  }
  
  if (rightSpeed > 0) {
    rightCmd = (int) ((rightSpeed * k) + minpower);
    digitalWrite(R_FWD, HIGH);
    digitalWrite(R_BACK, LOW);
  }
  else if (rightSpeed < 0) {
    rightCmd = (int) ((rightSpeed * k) - minpower);
    digitalWrite(R_FWD, LOW);
    digitalWrite(R_BACK, HIGH);
  }
  else {
    rightCmd = 0;
    digitalWrite(R_FWD, LOW);
    digitalWrite(R_BACK, LOW);
  }
  analogWrite(L_EN, leftCmd);
  analogWrite(R_EN, rightCmd);
}

// 'k_vel' callback function
void kCB( const std_msgs::Float32& f) {
  k = f.data;
}

// 'minpower' callback function
void minCB( const std_msgs::Int32& i) {
  minpower = i.data;
}

// 'k_vel' callback function
void speedCB( const std_msgs::Float32& f) {
  max_speed = f.data;
}


ros::Subscriber<geometry_msgs::Twist> velSub("cmd_vel", &velCB );
ros::Subscriber<std_msgs::Float32> kSub("k_vel", &kCB );
ros::Subscriber<std_msgs::Int32> minSub("min_power", &minCB );
ros::Subscriber<std_msgs::Float32> speedSub("max_speed", &speedCB );

void setup() {
	// Config
	pinMode(L_FWD, OUTPUT);
	pinMode(L_BACK, OUTPUT);
	pinMode(L_SENSE, INPUT);
	pinMode(L_EN, OUTPUT);
  pinMode(R_FWD, OUTPUT);
  pinMode(R_BACK, OUTPUT);
  pinMode(R_SENSE, INPUT);
  pinMode(R_EN, OUTPUT);
  analogWriteFrequency(L_EN, FREQ);
  analogWriteFrequency(R_EN, FREQ);
	delay(1000);
	
	// Init all pins
	digitalWrite(L_FWD, LOW);
	digitalWrite(L_BACK, LOW);
	analogWrite(L_EN, 0);
  digitalWrite(R_FWD, LOW);
  digitalWrite(R_BACK, LOW);
  analogWrite(R_EN, 0);
	delay(1000);
  nh.initNode();
  nh.subscribe(velSub);
  nh.subscribe(kSub);
  nh.subscribe(minSub);
  nh.subscribe(speedSub);
}

void loop() {
  nh.spinOnce();
  delay(1);
}
