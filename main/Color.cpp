/*!
 *  @file Adafruit_TCS34725.cpp
 *
 *  @mainpage Driver for the TCS34725 digital color sensors.
 *
 *  @section intro_sec Introduction
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing
 *  products from Adafruit!
 *
 *  @section author Author
 *
 *  KTOWN (Adafruit Industries)
 *
 *  @section license License
 *
 *  BSD (see license.txt)
 *
 *  @section HISTORY
 *
 *  v1.0 - First release
 */

#include <math.h>
#include <stdlib.h>

#include "Color.h"

/*!
 *  @brief  Implements missing powf function
 *  @param  x
 *          Base number
 *  @param  y
 *          Exponent
 *  @return x raised to the power of y
 */
float Color::powf(const float x, const float y) {
  return (float)(pow((double)x, (double)y));
}


/*!
 *  @brief  Writes a register and an 8 bit value over I2C
 *  @param  reg
 *  @param  value
 */
void Color::write8(uint8_t reg, uint32_t value) {
  m_wire->beginTransmission(COLOR_ADDRESS);
  m_wire->write(COLOR_COMMAND_BIT | reg);
  m_wire->write(value & 0xFF);
  m_wire->endTransmission();
}

/*!
 *  @brief  Reads an 8 bit value over I2C
 *  @param  reg
 *  @return value
 */
uint8_t Color::read8(uint8_t reg) {
  m_wire->beginTransmission(COLOR_ADDRESS);
  m_wire->write(COLOR_COMMAND_BIT | reg);
  m_wire->endTransmission();
  m_wire->requestFrom(COLOR_ADDRESS, 1);

  return m_wire->read();
}

/*!
 *  @brief  Reads a 16 bit values over I2C
 *  @param  reg
 *  @return value
 */
uint16_t Color::read16(uint8_t reg) {
  uint16_t x;
  uint16_t t;

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

/*!
 *  @brief  Enables the device
 */
void Color::enable() {
  write8(COLOR_ENABLE, COLOR_ENABLE_PON);
  delay(3);
  write8(COLOR_ENABLE, COLOR_ENABLE_PON | COLOR_ENABLE_AEN);  
  /* Set a delay for the integration time.
    This is only necessary in the case where enabling and then
    immediately trying to read values back. This is because setting
    AEN triggers an automatic integration, so if a read RGBC is
    performed too quickly, the data is not yet valid and all 0's are
    returned */
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

/*!
 *  @brief  Constructor
 *  @param  it
 *          Integration Time
 *  @param  gain
 *          Gain
 */
Color::Color(color_integration_time_t it, color_gain_t gain) {
  m_color_initialised = false;
  m_color_integration_time = it;
  m_color_gain = gain;

  m_wire = &Wire;
}

/*!
 *  @brief  Part of begin
 *  @return True if initialization was successful, otherwise false.
 */
boolean Color::init() {
  m_wire->begin();

  /* Make sure we're actually connected */
  uint8_t x = read8(COLOR_ID);
  if ((x != 0x44) && (x != 0x10)) {
    return false;
  }
  m_color_initialised = true;

  /* Set default integration time and gain */
  setIntegrationTime(m_color_integration_time);
  setGain(m_color_gain);

  /* Note: by default, the device is in power down mode on bootup */
  enable();

  return true;
}

/*!
 *  @brief  Sets the integration time for the TC34725
 *  @param  it
 *          Integration Time
 */
void Color::setIntegrationTime(color_integration_time_t it) {
  if (!m_color_initialised) init();

  /* Update the timing register */
  write8(COLOR_ATIME, it);

  /* Update value placeholders */
  m_color_integration_time = it;
}

/*!
 *  @brief  Adjusts the gain on the TCS34725
 *  @param  gain
 *          Gain (sensitivity to light)
 */
void Color::setGain(color_gain_t gain) {
  if (!m_color_initialised) init();

  /* Update the timing register */
  write8(COLOR_CONTROL, gain);

  /* Update value placeholders */
  m_color_gain = gain;
}

/*!
 *  @brief  Reads the raw red, green, blue and clear channel values
 *  @param  *r
 *          Red value
 *  @param  *g
 *          Green value
 *  @param  *b
 *          Blue value
 *  @param  *c
 *          Clear channel value
 */
void Color::getRawData(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c) {
  if (!m_color_initialised) init();

  *c = read16(COLOR_CDATAL);
  *r = read16(COLOR_RDATAL);
  *g = read16(COLOR_GDATAL);
  *b = read16(COLOR_BDATAL);

  /* Set a delay for the integration time */
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

/*!
 *  @brief  Converts the raw R/G/B values to color temperature in degrees Kelvin
 *  @param  r
 *          Red value
 *  @param  g
 *          Green value
 *  @param  b
 *          Blue value
 *  @return Color temperature in degrees Kelvin
 */
uint16_t Color::calculateColorTemperature(uint16_t r, uint16_t g, uint16_t b) {
  float X, Y, Z; /* RGB to XYZ correlation      */
  float xc, yc;  /* Chromaticity co-ordinates   */
  float n;       /* McCamy's formula            */
  float cct;

  /* 1. Map RGB values to their XYZ counterparts.    */
  /* Based on 6500K fluorescent, 3000K fluorescent   */
  /* and 60W incandescent values for a wide range.   */
  /* Note: Y = Illuminance or lux                    */
  X = (-0.14282F * r) + (1.54924F * g) + (-0.95641F * b);
  Y = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);
  Z = (-0.68202F * r) + (0.77073F * g) + (0.56332F * b);

  /* 2. Calculate the chromaticity co-ordinates      */
  xc = (X) / (X + Y + Z);
  yc = (Y) / (X + Y + Z);

  /* 3. Use McCamy's formula to determine the CCT    */
  n = (xc - 0.3320F) / (0.1858F - yc);

  /* Calculate the final CCT */
  cct = (449.0F * powf(n, 3)) + (3525.0F * powf(n, 2)) + (6823.3F * n) + 5520.33F;

  /* Return the results in degrees Kelvin */
  return (uint16_t)cct;
}

/*!
 *  @brief  Converts the raw R/G/B values to lux
 *  @param  r
 *          Red value
 *  @param  g
 *          Green value
 *  @param  b
 *          Blue value
 *  @return Lux value
 */
uint16_t Color::calculateLux(uint16_t r, uint16_t g, uint16_t b) {
  float illuminance;

  /* This only uses RGB ... how can we integrate clear or calculate lux */
  /* based exclusively on clear since this might be more reliable?      */
  illuminance = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);

  return (uint16_t)illuminance;
}
