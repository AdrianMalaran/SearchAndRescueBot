/**************************************************************************/
/*!
    @file     Adafruit_COLORsofti2c.h
    @author   KTOWN (Adafruit Industries)

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2013, Adafruit Industries
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/
#ifndef _ColorSoft_H
#define _ColorSoft_H

#include "SoftwareWire.h"

#include <Arduino.h>
#include <Wire.h>

#define COLOR_ADDRESS          (0x29)

#define COLOR_COMMAND_BIT      (0x80)

#define COLOR_ENABLE           (0x00)
#define COLOR_ENABLE_AEN       (0x02)    /* RGBC Enable - Writing 1 actives the ADC, 0 disables it */
#define COLOR_ENABLE_PON       (0x01)    /* Power on - Writing 1 activates the internal oscillator, 0 disables it */
#define COLOR_ATIME            (0x01)    /* Integration time */
#define COLOR_CONTROL          (0x0F)    /* Set the gain level for the sensor */
#define COLOR_ID               (0x12)    /* 0x44 = TCS34721/COLOR, 0x4D = TCS34723/TCS34727 */
#define COLOR_CDATAL           (0x14)    /* Clear channel data */
#define COLOR_RDATAL           (0x16)    /* Red channel data */
#define COLOR_GDATAL           (0x18)    /* Green channel data */
#define COLOR_BDATAL           (0x1A)    /* Blue channel data */

typedef enum
{
  COLOR_INTEGRATIONTIME_2_4MS  = 0xFF,   /**<  2.4ms - 1 cycle    - Max Count: 1024  */
  COLOR_INTEGRATIONTIME_24MS   = 0xF6,   /**<  24ms  - 10 cycles  - Max Count: 10240 */
  COLOR_INTEGRATIONTIME_50MS   = 0xEB,   /**<  50ms  - 20 cycles  - Max Count: 20480 */
  COLOR_INTEGRATIONTIME_101MS  = 0xD5,   /**<  101ms - 42 cycles  - Max Count: 43008 */
  COLOR_INTEGRATIONTIME_154MS  = 0xC0,   /**<  154ms - 64 cycles  - Max Count: 65535 */
  COLOR_INTEGRATIONTIME_700MS  = 0x00    /**<  700ms - 256 cycles - Max Count: 65535 */
}
color_integration_time_t;

typedef enum
{
  COLOR_GAIN_1X                = 0x00,   /**<  No gain  */
  COLOR_GAIN_4X                = 0x01,   /**<  2x gain  */
  COLOR_GAIN_16X               = 0x02,   /**<  16x gain */
  COLOR_GAIN_60X               = 0x03    /**<  60x gain */
}
color_gain_t;

////////   Green is SDA, Blue is SCL   ////////

class ColorSoft {
 public:
  ColorSoft(color_integration_time_t = COLOR_INTEGRATIONTIME_2_4MS, color_gain_t = COLOR_GAIN_1X, uint8_t sda_pin = 30, uint8_t scl_pin = 31);

  boolean begin(void);
  void setIntegrationTime(color_integration_time_t it);
  void setGain(color_gain_t gain);
  void getRawData(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c);
  uint16_t calculateColorTemperature(uint16_t r, uint16_t g, uint16_t b);
  uint16_t calculateLux(uint16_t r, uint16_t g, uint16_t b);
  void write8 (uint8_t reg, uint32_t value);
  uint8_t read8 (uint8_t reg);
  uint16_t read16 (uint8_t reg);
  void enable(void);

 private:
  boolean m_color_initialised;
  color_gain_t m_color_gain;
  color_integration_time_t m_color_integration_time;
  uint8_t m_sda_pin;
  uint8_t m_scl_pin;
  SoftwareWire m_i2c;

  float powf(const float x, const float y);
};

#endif
