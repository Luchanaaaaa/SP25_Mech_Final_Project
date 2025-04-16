#include "pixy.h"

PixySensor::PixySensor() {
}

void PixySensor::begin() {
  pixy.init();
}

int PixySensor::getSignature() {
  pixy.ccc.getBlocks();
  if (pixy.ccc.numBlocks > 0) {
    return pixy.ccc.blocks[0].m_signature;
  }
  // No block detected
  return -1;
}

int PixySensor::getBlockX() {
  pixy.ccc.getBlocks();
  if (pixy.ccc.numBlocks > 0) {
    return pixy.ccc.blocks[0].m_x;
  }
  return -1;
}

String PixySensor::getPositionRegion() {
  int x = getBlockX();
  if (x == -1) return "none";

  const int CENTER = 158;
  const int TOL = 10;

  if (x < CENTER - TOL) return "left";
  else if (x > CENTER + TOL) return "right";
  else return "center";
}

int PixySensor::getXOffsetFromCenter() {
  int x = getBlockX();
  if (x == -1) return 0;
  return x - 158;
}


