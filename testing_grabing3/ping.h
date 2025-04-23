#ifndef PIXY_H
#define PIXY_H

#include <Arduino.h>
#include <Pixy2.h>

class PixySensor {
public:
  PixySensor();
  void begin();
  bool seesPuck();
  int getPuckX();
  int getPuckY();
  int getPuckWidth();
  int getPuckHeight();

private:
  Pixy2 pixy;
  const int PUCK_SIGNATURE = 1;  // Orange puck signature
};

#endif