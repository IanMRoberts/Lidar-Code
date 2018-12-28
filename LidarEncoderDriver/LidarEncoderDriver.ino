#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>


Encoder LidarEnc(2,3);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(500000);
}

long EncPos  = -999;

void loop() {
  // put your main code here, to run repeatedly:
  /*long NewEncPos;
  NewEncPos = LidarEnc.read();
  if ( NewEncPos != EncPos ) {
    Serial.println(NewEncPos);
    EncPos = NewEncPos;
  }
  /**/
  if (Serial.available()) {
    char cmd = Serial.read();
  //  Serial.print("I received: ");
  //  Serial.println(cmd);
    if ( cmd == 'w' ){
      //Serial.println("Reset to zero");
      LidarEnc.write(0);
    }
    if (cmd == 'r' ){
      //Serial.print("Position = ");
      EncPos = LidarEnc.read();
      Serial.println(EncPos);
    }
  }
}
