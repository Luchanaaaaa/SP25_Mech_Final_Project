#include "pixy.h"

PixySensor::PixySensor() {
  // Constructor
}

void PixySensor::begin() {
  pixy.init();
}

bool PixySensor::seesPuck() {
  pixy.ccc.getBlocks();
  return (pixy.ccc.numBlocks > 0 && pixy.ccc.blocks[0].m_signature == PUCK_SIGNATURE);
}

int PixySensor::getPuckX() {
  if (seesPuck()) {
    return pixy.ccc.blocks[0].m_x;
  }
  return -1;  // Invalid value
}

int PixySensor::getPuckY() {
  if (seesPuck()) {
    return pixy.ccc.blocks[0].m_y;
  }
  return -1;  // Invalid value
}

int PixySensor::getPuckWidth() {
  if (seesPuck()) {
    return pixy.ccc.blocks[0].m_width;
  }
  return -1;  // Invalid value
}

int PixySensor::getPuckHeight() {
  if (seesPuck()) {
    return pixy.ccc.blocks[0].m_height;
  }
  return -1;  // Invalid value
}