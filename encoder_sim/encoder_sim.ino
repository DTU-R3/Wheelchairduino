#include <Encoder.h>

Encoder knobLeft(3, 7);
Encoder knobRight(19, 22);
String incomingByte = "";
long int leftcount = 0, rightcount = 0;
char cmd[2] = {'0','0'};
void setup() {
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) {
    Serial.readBytes(cmd,2);
    if ((cmd[0]=='d')&&(cmd[1]==';')) {
      long int newLeft, newRight;
      newLeft = knobLeft.read();
      newRight = knobRight.read();
      newLeft = -newLeft/2;  // Factor to be changed
      newRight = newRight/2;
      // Print only if it does not overflow
      if (((newLeft-leftcount) < 10000000) && ((newRight - rightcount) < 10000000))
      {
        Serial.println("wc:"+String(newLeft - leftcount)+","+String(newRight - rightcount)+";");
      }
      leftcount = newLeft;
      rightcount = newRight;
      Serial.flush();
    }
    else {
      Serial.flush();
    }
  }
  else {
    Serial.flush();
  }
}
