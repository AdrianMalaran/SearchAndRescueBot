#include <stdlib.h>
#include <math.h>

#include "utilities/SoftwareWire.h"
#include "ColorSoft.h"

float ColorSoft::powf(const float x, const float y) {
  return (float)(pow((double)x, (double)y));
}

void ColorSoft::write8(uint8_t reg, uint32_t value) {
  m_i2c.beginTransmission(COLOR_SOFT_ADDRESS);
  m_i2c.write(COLOR_SOFT_COMMAND_BIT | reg);
  m_i2c.write(value & 0xFF);
  m_i2c.endTransmission();
}

uint8_t ColorSoft::read8(uint8_t reg) {
  m_i2c.beginTransmission(COLOR_SOFT_ADDRESS);
  m_i2c.write(COLOR_SOFT_COMMAND_BIT | reg);
  m_i2c.endTransmission();
  m_i2c.requestFrom(COLOR_SOFT_ADDRESS, 1);

  return m_i2c.read();
}

uint16_t ColorSoft::read16(uint8_t reg) {
  uint16_t x; uint16_t t;

  m_i2c.beginTransmission(COLOR_SOFT_ADDRESS);
  m_i2c.write(COLOR_SOFT_COMMAND_BIT | reg);
  m_i2c.endTransmission();

  m_i2c.requestFrom(COLOR_SOFT_ADDRESS, 2);

  t = m_i2c.read();
  x = m_i2c.read();

  x <<= 8;
  x |= t;
  return x;
}

void ColorSoft::enable() {
  write8(COLOR_SOFT_ENABLE, COLOR_SOFT_ENABLE_PON);
  delay(3);
  write8(COLOR_SOFT_ENABLE, COLOR_SOFT_ENABLE_PON | COLOR_SOFT_ENABLE_AEN);

  switch (m_color_integration_time) {
  case COLOR_SOFT_INTEGRATIONTIME_2_4MS:
    delay(3);
    break;
  case COLOR_SOFT_INTEGRATIONTIME_24MS:
    delay(24);
    break;
  case COLOR_SOFT_INTEGRATIONTIME_50MS:
    delay(50);
    break;
  case COLOR_SOFT_INTEGRATIONTIME_101MS:
    delay(101);
    break;
  case COLOR_SOFT_INTEGRATIONTIME_154MS:
    delay(154);
    break;
  case COLOR_SOFT_INTEGRATIONTIME_700MS:
    delay(700);
    break;
  }
}

ColorSoft::ColorSoft(color_soft_integration_time_t it, color_soft_gain_t gain, uint8_t sda_pin, uint8_t scl_pin) {
  m_color_initialised = false;
  m_color_integration_time = it;
  m_color_gain = gain;

  m_sda_pin = sda_pin;
  m_scl_pin = scl_pin;

  m_i2c = SoftwareWire (sda_pin, scl_pin);
}

boolean ColorSoft::begin() {
  m_i2c.begin();

  uint8_t x = read8(COLOR_SOFT_ID);
  Serial.println(x, HEX);
  if (x != 0x44) {
    return false;
  }
  m_color_initialised = true;

  setIntegrationTime(m_color_integration_time);
  setGain(m_color_gain);

  enable();

  return true;
}

void ColorSoft::setIntegrationTime(color_soft_integration_time_t it) {
  if (!m_color_initialised) begin();

  write8(COLOR_SOFT_ATIME, it);

  m_color_integration_time = it;
}

void ColorSoft::setGain(color_soft_gain_t gain) {
  if (!m_color_initialised) begin();

  write8(COLOR_SOFT_CONTROL, gain);

  m_color_gain = gain;
}

void ColorSoft::getRawData(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c) {
  if (!m_color_initialised) begin();

  *c = read16(COLOR_SOFT_CDATAL);
  *r = read16(COLOR_SOFT_RDATAL);
  *g = read16(COLOR_SOFT_GDATAL);
  *b = read16(COLOR_SOFT_BDATAL);

  switch (m_color_integration_time)
  {
    case COLOR_SOFT_INTEGRATIONTIME_2_4MS:
      delay(3);
      break;
    case COLOR_SOFT_INTEGRATIONTIME_24MS:
      delay(24);
      break;
    case COLOR_SOFT_INTEGRATIONTIME_50MS:
      delay(50);
      break;
    case COLOR_SOFT_INTEGRATIONTIME_101MS:
      delay(101);
      break;
    case COLOR_SOFT_INTEGRATIONTIME_154MS:
      delay(154);
      break;
    case COLOR_SOFT_INTEGRATIONTIME_700MS:
      delay(700);
      break;
  }
}

uint16_t ColorSoft::calculateColorTemperature(uint16_t r, uint16_t g, uint16_t b) {
  float X, Y, Z;
  float xc, yc;
  float n;
  float cct;

  X = (-0.14282F * r) + (1.54924F * g) + (-0.95641F * b);
  Y = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);
  Z = (-0.68202F * r) + (0.77073F * g) + ( 0.56332F * b);

  xc = (X) / (X + Y + Z);
  yc = (Y) / (X + Y + Z);

  n = (xc - 0.3320F) / (0.1858F - yc);

  cct = (449.0F * powf(n, 3)) + (3525.0F * powf(n, 2)) + (6823.3F * n) + 5520.33F;

  return (uint16_t)cct;
}

uint16_t ColorSoft::calculateLux(uint16_t r, uint16_t g, uint16_t b) {
  float illuminance;

  illuminance = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);

  return (uint16_t)illuminance;
}
