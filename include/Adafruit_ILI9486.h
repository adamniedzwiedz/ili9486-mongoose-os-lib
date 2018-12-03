/* 
This is port of Adafruit_TFTLCD library containing only code for ILI9481 driver
*/

#ifndef _Adafruit_ILI9486_H_
#define _Adafruit_ILI9486_H_

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Adafruit_GFX.h>

class Adafruit_ILI9486 : public Adafruit_GFX {
  public:
    Adafruit_ILI9486(void);
    void reset(void);
    void begin(void);
    void setRotation(uint8_t x);
    void setAddrWindow(int x1, int y1, int x2, int y2);
    void fillScreen(uint16_t color);

    void drawXBitmap(int16_t x, int16_t y, const uint8_t *bitmap, 
                     int16_t w, int16_t h, uint16_t color);
    void drawPixel(int16_t x, int16_t y, uint16_t color);
    uint16_t color565(uint8_t r, uint8_t g, uint8_t b);

  private:
    void init(void);
    void setWriteDir(void);
    void setReadDir(void);
    void write8(uint8_t value);
    void writeRegister8(uint8_t a, uint8_t d);
    void writeRegister32(uint8_t a, uint32_t d);
    void flood(uint16_t color, uint32_t len);
};

#endif /* _Adafruit_ILI9486_H_ */