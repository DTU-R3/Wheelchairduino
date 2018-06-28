#define L_FWD 0
#define L_BACK 1
#define L_SENSE 2
#define L_EN 3

#define R_FWD 5
#define R_BACK 6
#define R_SENSE 7
#define R_EN 4

#define FREQ 25000

void setup() {
  // Start serial port
  Serial.begin(9600);
  
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
}

// Speed of the wheel
int leftSpeed = 0;
int rightSpeed = 0;
long int leftCount = 0;
long int rightCount = 0;
int lastL = LOW;
int lastR = LOW;


void loop() {
	digitalWrite(L_FWD, HIGH);
  digitalWrite(L_BACK, LOW);
  analogWrite(L_EN, 255);
  digitalWrite(R_FWD, HIGH);
  digitalWrite(R_BACK, LOW);
  analogWrite(R_EN, 255);
  int l_read = digitalRead(L_SENSE);
  int r_read = digitalRead(R_SENSE);
  if (lastL != l_read) {
    leftCount++;
  }
  if (lastR != r_read) {
    rightCount++;
  }
  lastL = l_read;
  lastR = r_read;
  
  Serial.println(String(leftCount) + "," + String(rightCount));
}
