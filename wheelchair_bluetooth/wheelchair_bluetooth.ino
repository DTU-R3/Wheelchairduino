#include <SoftwareSerial.h>

SoftwareSerial mySerial(9, 10); // RX, TX 

#define FWD_L_PWM 5
#define BACK_L_PWM 6
#define FWD_R_PWM 23
#define BACK_R_PWM 22

void setup() {
	
	// open the serial port
	Serial.begin(9600);
  mySerial.begin(9600);
 
	// Motion control
	pinMode(FWD_L_PWM, OUTPUT);
	pinMode(BACK_L_PWM, OUTPUT);
	pinMode(FWD_R_PWM, OUTPUT);
	pinMode(BACK_R_PWM, OUTPUT);
	delay(1000);
	
	// Init all pins
	analogWrite(FWD_L_PWM, 0);
	analogWrite(BACK_L_PWM, 0);
	analogWrite(FWD_R_PWM, 0);
	analogWrite(BACK_R_PWM, 0);
	delay(1000);
}

// Command init
int len = 12;
char cmd[12] = {'0', '0','0','0' '1', '0', '0', '0', '1', '0', '0', '0'};

// Speed of the wheel
int leftSpeed = 0;
int rightSpeed = 0;

void Park()
{
  analogWrite(FWD_L_PWM, 0);
  analogWrite(BACK_L_PWM, 0);
  analogWrite(FWD_R_PWM, 0);
  analogWrite(BACK_R_PWM, 0);
}

int charToInt(char ch)
{
	int tmp = 0;
	tmp = tmp * 10 + (ch - 48);
	return tmp;
}

void loop() {
	if (Serial.available() > 0) {
		Serial.readBytes(cmd, len);
	}
 
  if (mySerial.available() > 0) {
    mySerial.readBytes(cmd, len);
    Serial.println(cmd);
  }
	
	// Analyse command
	if (cmd[0] == 'D' && cmd[1] == 'K') {

		// calculate wheel speed
		leftSpeed = charToInt(cmd[5]) * 100 + charToInt(cmd[6]) * 10 + charToInt(cmd[7]);
		rightSpeed = charToInt(cmd[9]) * 100 + charToInt(cmd[10]) * 10 + charToInt(cmd[11]);
		
		switch (cmd[4]) {
			// Backward
			case '0':
				analogWrite(FWD_L_PWM, 0);
				analogWrite(BACK_L_PWM, leftSpeed);
				break;
			// Forward
			case '2':			
				analogWrite(FWD_L_PWM, leftSpeed);
				analogWrite(BACK_L_PWM, 0);
				break;   
			// Park
			default:
				analogWrite(FWD_L_PWM, 0);
				analogWrite(BACK_L_PWM, 0);
				break;
		}

		switch (cmd[8]) {
			// Backward
			case '0':
				analogWrite(FWD_R_PWM, 0);
				analogWrite(BACK_R_PWM, rightSpeed);
				break;
			// Forward
			case '2':
				analogWrite(FWD_R_PWM, rightSpeed);
				analogWrite(BACK_R_PWM, 0);
				break;
			// Park
			default:
				analogWrite(FWD_R_PWM, 0);
				analogWrite(BACK_R_PWM, 0);
				break;
		}	
	}
	// Error command
	else {
		Serial.flush();
    mySerial.flush();
		Park();		
	}
}
