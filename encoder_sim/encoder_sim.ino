#include <Encoder.h>

Encoder knobLeft(3, 7);
Encoder knobRight(19, 22);
long leftTicks = 0;
long rightTicks = 0;

void setup() {
  Serial.begin(9600);
}

char cmd[2] = {'0', '0'};
void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) {
    int incomingByte = Serial.read();
    long newLeft, newRight;
    newLeft = knobLeft.read();
    newRight = knobRight.read();
    if (newLeft != leftTicks || newRight != rightTicks) {
      leftTicks = newLeft;
      rightTicks = newRight;
    }
    Serial.println("wc:"+String(leftTicks)+","+String(rightTicks)+";");
  }
  else {
    Serial.flush();
  }
}
