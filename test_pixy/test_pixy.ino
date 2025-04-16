#include "pixy.h"

PixySensor pixy;

void setup() {
  Serial.begin(115200);
  pixy.begin();
  Serial.println("Pixy Test Started...");
}

void loop() {
  int sig = pixy.getSignature();

  if (sig == -1) {
    Serial.println("No block detected.");
  } else {
    int x = pixy.getBlockX();
    String region = pixy.getPositionRegion();
    int offset = pixy.getXOffsetFromCenter();

    Serial.println("-----------");
    Serial.print("Signature: ");
    Serial.println(sig);

    Serial.print("X Pos: ");
    Serial.println(x);

    Serial.print("Region: ");
    Serial.println(region);

    Serial.print("Offset from center: ");
    Serial.println(offset);
  }

  delay(200); 
}
