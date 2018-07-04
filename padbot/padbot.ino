#define L_FWD 3
#define L_BACK 4
#define R_FWD 6
#define R_BACK 5
#define L_SENSE 7
#define R_SENSE 8

#define FREQ 25000

void setup() {
  
	// Config
	pinMode(L_FWD, OUTPUT);
	pinMode(L_BACK, OUTPUT);
	pinMode(L_SENSE, INPUT);
  pinMode(R_FWD, OUTPUT);
  pinMode(R_BACK, OUTPUT);
  pinMode(R_SENSE, INPUT);
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
}

void loop() {
	analogWrite(L_FWD, 60);
  analogWrite(L_BACK, 0);
  analogWrite(R_FWD, 60);
  analogWrite(R_BACK, 0);
}
