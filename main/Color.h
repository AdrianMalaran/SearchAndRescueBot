#ifndef _Color_H_
#define _Color_H_

#include "SoftwareWire.h"

#include <Arduino.h>
#include <Wire.h>

#define COLOR_ADDRESS          (0x29)

#define COLOR_COMMAND_BIT      (0x80)

#define COLOR_ENABLE           (0x00)
#define COLOR_ENABLE_AEN       (0x02)
#define COLOR_ENABLE_PON       (0x01)
#define COLOR_ATIME            (0x01)
#define COLOR_CONTROL          (0x0F)
#define COLOR_ID               (0x12)
#define COLOR_CDATAL           (0x14)
#define COLOR_RDATAL           (0x16)
#define COLOR_GDATAL           (0x18)
#define COLOR_BDATAL           (0x1A)

typedef enum {
  COLOR_INTEGRATIONTIME_2_4MS  = 0xFF,   /**<  2.4ms - 1 cycle    - Max Count: 1024  */
  COLOR_INTEGRATIONTIME_24MS   = 0xF6,   /**<  24ms  - 10 cycles  - Max Count: 10240 */
  COLOR_INTEGRATIONTIME_50MS   = 0xEB,   /**<  50ms  - 20 cycles  - Max Count: 20480 */
  COLOR_INTEGRATIONTIME_101MS  = 0xD5,   /**<  101ms - 42 cycles  - Max Count: 43008 */
  COLOR_INTEGRATIONTIME_154MS  = 0xC0,   /**<  154ms - 64 cycles  - Max Count: 65535 */
  COLOR_INTEGRATIONTIME_700MS  = 0x00    /**<  700ms - 256 cycles - Max Count: 65535 */
} color_integration_time_t;

typedef enum {
  COLOR_GAIN_1X                = 0x00,   /**<  No gain  */
  COLOR_GAIN_4X                = 0x01,   /**<  2x gain  */
  COLOR_GAIN_16X               = 0x02,   /**<  16x gain */
  COLOR_GAIN_60X               = 0x03    /**<  60x gain */
} color_gain_t;

////////   Green is SDA, Blue is SCL   ////////

class Color {
public:
  Color(color_integration_time_t = COLOR_INTEGRATIONTIME_2_4MS, color_gain_t = COLOR_GAIN_1X);
  Color(uint8_t sda_pin, uint8_t scl_pin, color_integration_time_t = COLOR_INTEGRATIONTIME_2_4MS, color_gain_t = COLOR_GAIN_1X);

  boolean begin();
  void setIntegrationTime(color_integration_time_t it);
  void setGain(color_gain_t gain);
  void getRawData(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c);
  uint16_t calculateColorTemperature(uint16_t r, uint16_t g, uint16_t b);
  uint16_t calculateLux(uint16_t r, uint16_t g, uint16_t b);
  void write8(uint8_t reg, uint32_t value);
  uint8_t read8(uint8_t reg);
  uint16_t read16(uint8_t reg);
  void enable();

private:
  boolean m_color_initialised;
  color_gain_t m_color_gain;
  color_integration_time_t m_color_integration_time;

  boolean m_is_soft;
  uint8_t m_sda_pin;
  uint8_t m_scl_pin;
  SoftwareWire m_i2c;
  TwoWire *m_wire;

  float powf(const float x, const float y);
};

#endif
