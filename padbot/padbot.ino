#define L_FWD 20
#define L_BACK 19
#define L_SENSE 18
#define L_EN 23

#define R_FWD 17
#define R_BACK 16
#define R_SENSE 15
#define R_EN 22

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

void loop() {
	digitalWrite(L_FWD, HIGH);
  digitalWrite(L_BACK, LOW);
  analogWrite(L_EN, 200);
  digitalWrite(R_FWD, HIGH);
  digitalWrite(R_BACK, LOW);
  analogWrite(R_EN, 200);
}
