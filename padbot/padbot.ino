#include <ros.h>
#include <math.h> 
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

// Pins
#define L_SENSE 0
#define R_SENSE 1
#define L_DIR 2
#define L_EN 4
#define R_DIR 3
#define R_EN 5

#define FREQ 25000

// Robot parameter
#define WHEELBASE 0.228
#define METERPERCOUNT 0.0000439
#define SPEEDTOCMD 543.48
#define MIN_POWER 11
#define FWD_EB 1.05
#define BACK_EB 1.065
#define MAXSPEED (200.0-MIN_POWER)/SPEEDTOCMD
#define TIMEOUT 5000  // in ms

// Control parameter
#define KP 0.3

// Variables
ros::NodeHandle  nh;
float leftSpeed = 0;
float rightSpeed = 0;
float currentLeftSpeed = 0;
float currentRightSpeed = 0;
float cmdLeftSpeed = 0;
float cmdRightSpeed = 0;
int minpower = 60;
int leftCount = 0;
int rightCount = 0;
int lastLeftCount = 0;
int lastRightCount = 0;
int leftDir = 0;
int rightDir = 0;
unsigned long vel_received = 0;
unsigned long encoder_published = 0;

// Control Function
int WheelControl(float spd, int dir_pin, int en_pin, float fwd_Eb, float back_Eb) {
  int dir = 0;
  int cmd = 0;
  if (spd > 0.05) { 
    cmd = (int) (fwd_Eb * ((spd * SPEEDTOCMD) + MIN_POWER)); 
    dir = 1; 
    analogWrite(dir_pin, LOW);
    analogWrite(en_pin, min(abs(cmd),200)); 
  } 
  else if(spd < 0.05) { 
    cmd = (int) (back_Eb * ((spd * SPEEDTOCMD) - MIN_POWER)); 
    dir = -1; 
    analogWrite(dir_pin, HIGH);
    analogWrite(en_pin, min(abs(cmd),200)); 
  } 
  else { 
    cmd = 0; 
    dir = 0; 
    analogWrite(dir_pin, LOW);
    analogWrite(en_pin, 0); 
  } 
  return dir;
}

// 'cmd_vel' callback function
void velCB( const geometry_msgs::Twist& vel) {
  vel_received = millis();
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
  cmdLeftSpeed = leftSpeed;
  cmdRightSpeed = rightSpeed;

  leftDir = WheelControl(cmdLeftSpeed, L_DIR, L_EN, FWD_EB, BACK_EB);  
  rightDir = WheelControl(cmdRightSpeed, R_DIR, R_EN, 1.00, 1.00); 
}

void resetCB( const std_msgs::Bool& b) {
  if (b.data) {
    leftCount = 0;
    rightCount = 0;
    lastLeftCount = 0;
    lastRightCount = 0;
  }
}

std_msgs::Int32 left_count;
std_msgs::Int32 right_count;
ros::Publisher leftPub("padbot/left_count", &left_count);
ros::Publisher rightPub("padbot/right_count", &right_count);
std_msgs::Float32 left_speed;
std_msgs::Float32 right_speed;
ros::Publisher leftSpdPub("padbot/left_speed", &left_speed);
ros::Publisher rightSpdPub("padbot/right_speed", &right_speed);
ros::Subscriber<geometry_msgs::Twist> velSub("cmd_vel", &velCB );
ros::Subscriber<std_msgs::Bool> resetSub("padbot/reset_encoder", &resetCB );

void setup() {
	// Config
  pinMode(L_DIR, OUTPUT);
  pinMode(R_DIR, OUTPUT);
  pinMode(L_EN, OUTPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(L_SENSE, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(L_SENSE), L_Encoder, CHANGE); 
  pinMode(R_SENSE, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(R_SENSE), R_Encoder, CHANGE); 
  analogWriteFrequency(L_EN, FREQ);
  analogWriteFrequency(R_EN, FREQ);
  delay(1000);
	
	// Init all pins
	analogWrite(L_DIR, LOW);
  analogWrite(R_DIR, LOW);
  analogWrite(L_EN, 0);
  analogWrite(R_EN, 0);
	delay(1000);
  nh.initNode();
  nh.advertise(leftPub);
  nh.advertise(rightPub);
  nh.advertise(leftSpdPub);
  nh.advertise(rightSpdPub);
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
  int deltaMS = millis() - encoder_published;
  encoder_published = millis();
  int deltaLeftCount = leftCount - lastLeftCount;
  int deltaRightCount = rightCount - lastRightCount;
  lastLeftCount = leftCount;
  lastRightCount = rightCount;

  // Feedback Control
  currentLeftSpeed = deltaLeftCount * METERPERCOUNT * 1000 / deltaMS;
  currentRightSpeed = deltaRightCount * METERPERCOUNT * 1000 / deltaMS;
  left_speed.data = currentLeftSpeed;
  leftSpdPub.publish(&left_speed );
  right_speed.data = currentRightSpeed;
  rightSpdPub.publish(&right_speed );
  cmdLeftSpeed += KP * (leftSpeed - currentLeftSpeed);
  cmdRightSpeed += KP * (rightSpeed - currentRightSpeed);
  leftDir = WheelControl(cmdLeftSpeed, L_DIR, L_EN, FWD_EB, BACK_EB);  
  rightDir = WheelControl(cmdRightSpeed, R_DIR, R_EN, 1.00, 1.00);
  
  
  // If timeout, stop the robot
  if ( millis() - vel_received > TIMEOUT ) {
    analogWrite(L_DIR, LOW);
    analogWrite(R_DIR, LOW);
    analogWrite(L_EN, 0);
    analogWrite(R_EN, 0);
    leftDir = 0;
    rightDir = 0;
  }
  
  delay(100);
  nh.spinOnce();
}
