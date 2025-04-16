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

  /**
   * Returns the x-position of the first detected block (0–316),
   * or -1 if no block is detected.
   */
  int getBlockX();

  /**
   * Returns a string: "left", "center", "right", or "none"
   * based on where the block is located in the camera frame.
   */
  String getPositionRegion();

  /**
   * Returns how far the block is from center (positive = right, negative = left)
   * or 0 if no block is detected.
   */
  int getXOffsetFromCenter();

private:
  Pixy2 pixy;
};

#endif
