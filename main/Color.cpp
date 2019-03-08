#include <math.h>
#include <stdlib.h>

#include "Color.h"

float Color::powf(const float x, const float y) {
  return (float)(pow((double)x, (double)y));
}

void Color::write8(uint8_t reg, uint32_t value) {
  m_wire->beginTransmission(COLOR_ADDRESS);
  m_wire->write(COLOR_COMMAND_BIT | reg);
  m_wire->write(value & 0xFF);
  m_wire->endTransmission();
}

uint8_t Color::read8(uint8_t reg) {
  m_wire->beginTransmission(COLOR_ADDRESS);
  m_wire->write(COLOR_COMMAND_BIT | reg);
  m_wire->endTransmission();
  m_wire->requestFrom(COLOR_ADDRESS, 1);

  return m_wire->read();
}

uint16_t Color::read16(uint8_t reg) {
  uint16_t x; uint16_t t;

  m_wire->beginTransmission(COLOR_ADDRESS);
  m_wire->write(COLOR_COMMAND_BIT | reg);
  m_wire->endTransmission();

  m_wire->requestFrom(COLOR_ADDRESS, 2);

  t = m_wire->read();
  x = m_wire->read();

  x <<= 8;
  x |= t;
  return x;
}

void Color::enable() {
  write8(COLOR_ENABLE, COLOR_ENABLE_PON);
  delay(3);
  write8(COLOR_ENABLE, COLOR_ENABLE_PON | COLOR_ENABLE_AEN);  

  switch (m_color_integration_time) {
  case COLOR_INTEGRATIONTIME_2_4MS:
    delay(3);
    break;
  case COLOR_INTEGRATIONTIME_24MS:
    delay(24);
    break;
  case COLOR_INTEGRATIONTIME_50MS:
    delay(50);
    break;
  case COLOR_INTEGRATIONTIME_101MS:
    delay(101);
    break;
  case COLOR_INTEGRATIONTIME_154MS:
    delay(154);
    break;
  case COLOR_INTEGRATIONTIME_700MS:
    delay(700);
    break;
  }
}

Color::Color(color_integration_time_t it, color_gain_t gain) {
  m_color_initialised = false;
  m_color_integration_time = it;
  m_color_gain = gain;

  m_wire = &Wire;
}

boolean Color::begin() {
  m_wire->begin();

  uint8_t x = read8(COLOR_ID);
  if ((x != 0x44) && (x != 0x10)) {
    return false;
  }
  m_color_initialised = true;

  setIntegrationTime(m_color_integration_time);
  setGain(m_color_gain);

  enable();

  return true;
}

void Color::setIntegrationTime(color_integration_time_t it) {
  if (!m_color_initialised) begin();

  write8(COLOR_ATIME, it);

  m_color_integration_time = it;
}

void Color::setGain(color_gain_t gain) {
  if (!m_color_initialised) begin();

  write8(COLOR_CONTROL, gain);

  m_color_gain = gain;
}

void Color::getRawData(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c) {
  if (!m_color_initialised) begin();

  *c = read16(COLOR_CDATAL);
  *r = read16(COLOR_RDATAL);
  *g = read16(COLOR_GDATAL);
  *b = read16(COLOR_BDATAL);

  switch (m_color_integration_time)
  {
    case COLOR_INTEGRATIONTIME_2_4MS:
      delay(3);
      break;
    case COLOR_INTEGRATIONTIME_24MS:
      delay(24);
      break;
    case COLOR_INTEGRATIONTIME_50MS:
      delay(50);
      break;
    case COLOR_INTEGRATIONTIME_101MS:
      delay(101);
      break;
    case COLOR_INTEGRATIONTIME_154MS:
      delay(154);
      break;
    case COLOR_INTEGRATIONTIME_700MS:
      delay(700);
      break;
  }
}

uint16_t Color::calculateColorTemperature(uint16_t r, uint16_t g, uint16_t b) {
  float X, Y, Z;
  float xc, yc;
  float n;
  float cct;

  X = (-0.14282F * r) + (1.54924F * g) + (-0.95641F * b);
  Y = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);
  Z = (-0.68202F * r) + (0.77073F * g) + (0.56332F * b);

  xc = (X) / (X + Y + Z);
  yc = (Y) / (X + Y + Z);

  n = (xc - 0.3320F) / (0.1858F - yc);

  cct = (449.0F * powf(n, 3)) + (3525.0F * powf(n, 2)) + (6823.3F * n) + 5520.33F;

  return (uint16_t)cct;
}

uint16_t Color::calculateLux(uint16_t r, uint16_t g, uint16_t b) {
  float illuminance;

  illuminance = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);

  return (uint16_t)illuminance;
}
