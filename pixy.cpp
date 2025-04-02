#include "pixy.h"

// Constructor: symbolic
PixySensor::PixySensor() {
}

void PixySensor::begin() {
  pixy.init();
}

int PixySensor::getSignature() {
  pixy.ccc.getBlocks(); // Fetch blocks from Pixy

  if (pixy.ccc.numBlocks > 0) {
    return pixy.ccc.blocks[0].m_signature;
  }
  
  // No block detected
  return -1;
}
