#ifndef PIXY_H
#define PIXY_H

#include <Arduino.h>
#include <Pixy2.h>

class PixySensor {
public:
  // Constructor
  PixySensor();

  // Initializes the Pixy camera.
  void begin();

  /**
   * Gets the signature of the first detected block.
   * Returns the signature as an int, or -1 if no block is detected.
   */ 
  int getSignature();

private:
  Pixy2 pixy;
};

#endif
