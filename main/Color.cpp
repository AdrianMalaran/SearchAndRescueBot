#include <stdlib.h>
#include <math.h>

#include "SoftwareWire.h"
#include "Color.h"

double Color::powf(const double x, const double y) {
  return (double)(pow((double)x, (double)y));
}

void Color::write8(uint8_t reg, uint32_t value) {
  if(m_is_soft) {
    m_i2c.beginTransmission(COLOR_ADDRESS);
    m_i2c.write(COLOR_COMMAND_BIT | reg);
    m_i2c.write(value & 0xFF);
    m_i2c.endTransmission();
  } else {
    m_wire->beginTransmission(COLOR_ADDRESS);
    m_wire->write(COLOR_COMMAND_BIT | reg);
    m_wire->write(value & 0xFF);
    m_wire->endTransmission();
  }
}

uint8_t Color::read8(uint8_t reg) {
  if(m_is_soft) {
    m_i2c.beginTransmission(COLOR_ADDRESS);
    m_i2c.write(COLOR_COMMAND_BIT | reg);
    m_i2c.endTransmission();
    m_i2c.requestFrom(COLOR_ADDRESS, 1);

    return m_i2c.read();
  } else {
    m_wire->beginTransmission(COLOR_ADDRESS);
    m_wire->write(COLOR_COMMAND_BIT | reg);
    m_wire->endTransmission();
    m_wire->requestFrom(COLOR_ADDRESS, 1);

    return m_wire->read();
  }
}

uint16_t Color::read16(uint8_t reg) {
  uint16_t x; uint16_t t;

  if(m_is_soft) {
    m_i2c.beginTransmission(COLOR_ADDRESS);
    m_i2c.write(COLOR_COMMAND_BIT | reg);
    m_i2c.endTransmission();

    m_i2c.requestFrom(COLOR_ADDRESS, 2);

    t = m_i2c.read();
    x = m_i2c.read();
  } else {
    m_wire->beginTransmission(COLOR_ADDRESS);
    m_wire->write(COLOR_COMMAND_BIT | reg);
    m_wire->endTransmission();

    m_wire->requestFrom(COLOR_ADDRESS, 2);

    t = m_wire->read();
    x = m_wire->read();
  }

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

Color::Color() {}

Color::Color(color_integration_time_t it, color_gain_t gain) {
  m_is_soft = false;
  m_color_initialised = false;
  m_color_integration_time = it;
  m_color_gain = gain;

  m_wire = &Wire;
}

Color::Color(uint8_t sda_pin, uint8_t scl_pin, color_integration_time_t it, color_gain_t gain) {
  m_is_soft = true;
  m_color_initialised = false;
  m_color_integration_time = it;
  m_color_gain = gain;

  m_sda_pin = sda_pin;
  m_scl_pin = scl_pin;

  m_i2c = SoftwareWire(sda_pin, scl_pin);
}

boolean Color::begin() {
  m_is_soft ? m_i2c.begin() : m_wire->begin();

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

uint16_t Color::calculateColorTemperature(uint16_t r, uint16_t g, uint16_t b) {
  double X, Y, Z;
  double xc, yc;
  double n;
  double cct;

  X = (-0.14282F * r) + (1.54924F * g) + (-0.95641F * b);
  Y = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);
  Z = (-0.68202F * r) + (0.77073F * g) + (0.56332F * b);

  xc = (X) / (X + Y + Z);
  yc = (Y) / (X + Y + Z);

  n = (xc - 0.3320F) / (0.1858F - yc);

  cct = (449.0F * powf(n, 3)) + (3525.0F * powf(n, 2)) + (6823.3F * n) + 5520.33F;

  return (uint16_t)cct;
}

BlockType Color::getTerrainColor() {
  uint16_t r_tot = 0, g_tot = 0, b_tot = 0;
  uint16_t r, g, b, c;
  for(int i = 0; i < 10; i++) {
      getRawData(&r, &g, &b, &c);
      r_tot+=r;
      g_tot+=g;
      b_tot+=b;
  }
  r_tot/=10;
  g_tot/=10;
  b_tot/=10;

  Serial.print("R: "); Serial.print(r_tot);
  Serial.print(" G: "); Serial.print(g_tot);
  Serial.print(" B: "); Serial.println(b_tot);

/*
PARTICLE Board readings
    R: 180 - 290
    G: 170 - 255
    B: 190 - 230

SAND Board readings
    R: 90 - 140
    G: 95 - 135
    B: 100 - 140

GRAVEL Board readings
    R: 150 - 190
    G: 155 - 195
    B: 145 - 180

WATER Board readings
    R: 185 - 260
    G: 160 - 210
    B: 150 - 190
*/

  // TODO: All of these values need to be calibrated
 if (r_tot > 185 && r_tot < 260 && g_tot > 160 && g_tot < 210 && b_tot > 150 && b_tot < 190) {
    Serial.println("WATER");
    return WATER; // Water
}
  else if (r_tot > 90 && r_tot < 140 && g_tot > 95 && g_tot < 135 && b_tot > 100 && b_tot < 140) {
    Serial.println("SAND");
    return SAND; // Sand
}
  else if (r_tot > 150 && r_tot < 200 && g_tot > 140 && g_tot < 195 && b_tot > 140 && b_tot < 180) {
    Serial.println("GRAVEL");
    return GRAVEL; // Gravel
}
 else if (r_tot > 180 && r_tot < 290 && g_tot > 170 && g_tot < 255 && b_tot > 190 && b_tot < 230) {
  Serial.println("PARTICLE");
  return PARTICLE; // Particle Board
}
  else {
    Serial.println("UNKNOWN");
    return UNKNOWN; // Unknown
    }
}

int Color::getStructureColor() {
  uint16_t r_tot = 0, g_tot = 0, b_tot = 0;
  uint16_t r, g, b, c;
  for(int i = 0; i < 10; i++) {
      getRawData(&r, &g, &b, &c);
      r_tot+=r;
      g_tot+=g;
      b_tot+=b;
  }
  r_tot/=10;
  g_tot/=10;
  b_tot/=10;

  Serial.print("R: "); Serial.print(r_tot);
  Serial.print(" G: "); Serial.print(g_tot);
  Serial.print(" B: "); Serial.println(b_tot);

  // TODO: All of these values need to be calibrated
  if (r_tot > 100 && r_tot < 200 && g_tot > 100 && g_tot < 200 && b_tot > 100 && b_tot < 200)
    return 1; // Red House
  else if (r_tot > 150 && r_tot < 200 && g_tot > 20 && g_tot < 100 && b_tot > 175 && b_tot < 250)
    return 2; // Yellow House
  else
    return 0; // Unknown

}

uint16_t Color::calculateLux(uint16_t r, uint16_t g, uint16_t b) {
  double illuminance;

  illuminance = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);

  return (uint16_t)illuminance;
}
