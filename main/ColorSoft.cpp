/**************************************************************************/
/*!
    @file     ColorSoft.cpp
    @author   KTOWN (Adafruit Industries)
    @license  BSD (see license.txt)

    Driver for the digital color sensors.

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    @section  HISTORY

    v1.0 - First release
*/
/**************************************************************************/
#include <stdlib.h>
#include <math.h>

#include "SoftwareWire.h"
#include "ColorSoft.h"

/*========================================================================*/
/*                          PRIVATE FUNCTIONS                             */
/*========================================================================*/

/**************************************************************************/
/*!
    @brief  Implements missing powf function
*/
/**************************************************************************/
float ColorSoft::powf(const float x, const float y)
{
  return (float)(pow((double)x, (double)y));
}

/**************************************************************************/
/*!
    @brief  Writes a register and an 8 bit value over I2C
*/
/**************************************************************************/
void ColorSoft::write8(uint8_t reg, uint32_t value)
{
  m_i2c.beginTransmission(COLOR_ADDRESS);
  m_i2c.write(COLOR_COMMAND_BIT | reg);
  m_i2c.write(value & 0xFF);
  m_i2c.endTransmission();
}

/**************************************************************************/
/*!
    @brief  Reads an 8 bit value over I2C
*/
/**************************************************************************/
uint8_t ColorSoft::read8(uint8_t reg)
{
  m_i2c.beginTransmission(COLOR_ADDRESS);
  m_i2c.write(COLOR_COMMAND_BIT | reg);
  m_i2c.endTransmission();
  m_i2c.requestFrom(COLOR_ADDRESS, 1);

  return m_i2c.read();
}

/**************************************************************************/
/*!
    @brief  Reads a 16 bit values over I2C
*/
/**************************************************************************/
uint16_t ColorSoft::read16(uint8_t reg)
{
  uint16_t x; uint16_t t;

  m_i2c.beginTransmission(COLOR_ADDRESS);
  m_i2c.write(COLOR_COMMAND_BIT | reg);
  m_i2c.endTransmission();

  m_i2c.requestFrom(COLOR_ADDRESS, 2);

  t = m_i2c.read();
  x = m_i2c.read();

  x <<= 8;
  x |= t;
  return x;
}

/**************************************************************************/
/*!
    Enables the device
*/
/**************************************************************************/
void ColorSoft::enable(void)
{
  write8(COLOR_ENABLE, COLOR_ENABLE_PON);
  delay(3);
  write8(COLOR_ENABLE, COLOR_ENABLE_PON | COLOR_ENABLE_AEN);
}

/*========================================================================*/
/*                            CONSTRUCTORS                                */
/*========================================================================*/

/**************************************************************************/
/*!
    Constructor
*/
/**************************************************************************/
ColorSoft::ColorSoft(color_integration_time_t it, color_gain_t gain, uint8_t sda_pin, uint8_t scl_pin)
{
  m_color_initialised = false;
  m_color_integration_time = it;
  m_color_gain = gain;

  m_sda_pin = sda_pin;
  m_scl_pin = scl_pin;

  m_i2c = SoftwareWire (sda_pin, scl_pin);
}

/*========================================================================*/
/*                           PUBLIC FUNCTIONS                             */
/*========================================================================*/

/**************************************************************************/
/*!
    Initializes I2C and configures the sensor (call this function before
    doing anything else)
*/
/**************************************************************************/
boolean ColorSoft::begin(void)
{
  m_i2c.begin();

  /* Make sure we're actually connected */
  uint8_t x = read8(COLOR_ID);
  Serial.println(x, HEX);
  if (x != 0x44)
  {
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

/**************************************************************************/
/*!
    Sets the integration time for the TC34725
*/
/**************************************************************************/
void ColorSoft::setIntegrationTime(color_integration_time_t it)
{
  if (!m_color_initialised) begin();

  /* Update the timing register */
  write8(COLOR_ATIME, it);

  /* Update value placeholders */
  m_color_integration_time = it;
}

/**************************************************************************/
/*!
    Adjusts the gain on the Color Sensor (adjusts the sensitivity to light)
*/
/**************************************************************************/
void ColorSoft::setGain(color_gain_t gain)
{
  if (!m_color_initialised) begin();

  /* Update the timing register */
  write8(COLOR_CONTROL, gain);

  /* Update value placeholders */
  m_color_gain = gain;
}

/**************************************************************************/
/*!
    @brief  Reads the raw red, green, blue and clear channel values
*/
/**************************************************************************/
void ColorSoft::getRawData(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
  if (!m_color_initialised) begin();

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

/**************************************************************************/
/*!
    @brief  Converts the raw R/G/B values to color temperature in degrees
            Kelvin
*/
/**************************************************************************/
uint16_t ColorSoft::calculateColorTemperature(uint16_t r, uint16_t g, uint16_t b)
{
  float X, Y, Z;      /* RGB to XYZ correlation      */
  float xc, yc;       /* Chromaticity co-ordinates   */
  float n;            /* McCamy's formula            */
  float cct;

  /* 1. Map RGB values to their XYZ counterparts.    */
  /* Based on 6500K fluorescent, 3000K fluorescent   */
  /* and 60W incandescent values for a wide range.   */
  /* Note: Y = Illuminance or lux                    */
  X = (-0.14282F * r) + (1.54924F * g) + (-0.95641F * b);
  Y = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);
  Z = (-0.68202F * r) + (0.77073F * g) + ( 0.56332F * b);

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

/**************************************************************************/
/*!
    @brief  Converts the raw R/G/B values to color temperature in degrees
            Kelvin
*/
/**************************************************************************/
uint16_t ColorSoft::calculateLux(uint16_t r, uint16_t g, uint16_t b)
{
  float illuminance;

  /* This only uses RGB ... how can we integrate clear or calculate lux */
  /* based exclusively on clear since this might be more reliable?      */
  illuminance = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);

  return (uint16_t)illuminance;
}
