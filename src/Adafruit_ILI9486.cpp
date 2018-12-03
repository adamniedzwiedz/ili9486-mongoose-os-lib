#include "Adafruit_GFX.h"
#include "Adafruit_ILI9486.h"
#include "mgos_system.h"

#define LCD_RD 2
#define LCD_WR 4
#define LCD_RS 15  // CD
#define LCD_CS 33
#define LCD_RST 32
#define LCD_CD LCD_RS

static const uint8_t LCD_DATA[8] = { 12, 13, 26, 25, 17, 16, 27, 14};

#define CS_IDLE digitalWrite(LCD_CS, HIGH)
#define CS_ACTIVE digitalWrite(LCD_CS, LOW)
#define WR_IDLE digitalWrite(LCD_WR, HIGH)
#define WR_ACTIVE digitalWrite(LCD_WR, LOW)
#define RD_IDLE digitalWrite(LCD_RD, HIGH)
#define RD_ACTIVE digitalWrite(LCD_RD, LOW)

#define CD_DATA digitalWrite(LCD_CD, HIGH)
#define CD_COMMAND digitalWrite(LCD_CD, LOW)

#define WR_STROBE { WR_ACTIVE; WR_IDLE; }

#define TFTWIDTH   320
#define TFTHEIGHT  480

#define TFTLCD_DELAY 0xFF

#define ILI9486_COLADDRSET         0x2A
#define ILI9486_PAGEADDRSET        0x2B
#define ILI9486_MADCTL             0x36

#define ILI9486_MADCTL_MY  0x80
#define ILI9486_MADCTL_MX  0x40
#define ILI9486_MADCTL_MV  0x20
#define ILI9486_MADCTL_BGR 0x08

#define pgm_read_byte(addr) (*(const unsigned char *)(addr))

static const uint8_t ILI9486_regValues[] = {
  0x11, 0,                   // exit sleep mode
	TFTLCD_DELAY, 50,          // delay 50ms
	0xC0, 2, 0x07, 0x07,       // power control1
	0xC1, 2, 0x42, 0x07,       // power control1
	0xC5, 4, 0x00, 0x24, 0x80, 0x00, // VCOM control
	0xC2, 1, 0x43,             // power setting for normal mode
	0xB6, 3, 0x08, 0x01, 0x3B, // panel driving setting
	0xB1, 2, 0xF0, 0x00,       // frame rate and inversion control
	0x34, 0,                   // set tear off
  0x36, 1, 0x88,             // set address mode 
	0x3A, 1, 0x55,             // set pixel format
	0x2A, 4, 0x00, 0x00,0x01,0x3F, // set column address
	0x2B, 4, 0x00, 0x00,0x01,0xE0, // set page address
	TFTLCD_DELAY, 120,	
	0x29, 0,                   // set display on
	0x2C, 0,                   // write memory start		
};

Adafruit_ILI9486::Adafruit_ILI9486(void) : Adafruit_GFX(TFTWIDTH, TFTHEIGHT) 
{
  init();
}

void Adafruit_ILI9486::init(void) 
{
  CS_IDLE;
  WR_IDLE;
  RD_IDLE;
  CD_DATA;
  digitalWrite(LCD_RST, HIGH);

  pinMode(LCD_CS, OUTPUT);    // Enable outputs
  pinMode(LCD_RS, OUTPUT);
  pinMode(LCD_WR, OUTPUT);
  pinMode(LCD_RD, OUTPUT);
  pinMode(LCD_RST, OUTPUT);

  digitalWrite(LCD_RST, HIGH);

  setWriteDir(); // Set up LCD data port(s) for WRITE operations

  rotation  = 0;
  cursor_y  = cursor_x = 0;
  textsize  = 1;
  textcolor = 0xFFFF;
  _width    = TFTWIDTH;
  _height   = TFTHEIGHT;
}

void Adafruit_ILI9486::reset(void) 
{
  CS_IDLE;
//  CD_DATA;
  WR_IDLE;
  RD_IDLE;

  digitalWrite(LCD_RST, LOW);
  delay(2);
  digitalWrite(LCD_RST, HIGH);

  CS_ACTIVE;
  CD_COMMAND;

  write8(0x00);
  for(uint8_t i=0; i<3; i++) WR_STROBE; // Three extra 0x00s
  CS_IDLE;
}

void Adafruit_ILI9486::setWriteDir(void) 
{
  for (uint8_t i=0; i<8; i++) {
    pinMode(LCD_DATA[i], OUTPUT);
  }
}

void Adafruit_ILI9486::setReadDir(void) 
{
  for (uint8_t i=0; i<8; i++) {
    pinMode(LCD_DATA[i], INPUT);
  }
}

void Adafruit_ILI9486::write8(uint8_t value)
{
  for (uint8_t i=0; i<8; i++) {
    digitalWrite(LCD_DATA[i], (value >> i) & 0x01);
  }
  WR_STROBE;
}

void Adafruit_ILI9486::writeRegister8(uint8_t a, uint8_t d) 
{
  CD_COMMAND; 
  write8(a); 
  CD_DATA; 
  write8(d);
}

void Adafruit_ILI9486::writeRegister32(uint8_t r, uint32_t d) 
{
  CS_ACTIVE;
  CD_COMMAND;
  write8(r);
  CD_DATA;
  delayMicroseconds(10);
  write8(d >> 24);
  delayMicroseconds(10);
  write8(d >> 16);
  delayMicroseconds(10);
  write8(d >> 8);
  delayMicroseconds(10);
  write8(d);
  CS_IDLE;
}

void Adafruit_ILI9486::begin(void) 
{
  uint8_t i = 0;
  reset();
  delay(200);

  // 9486
  CS_ACTIVE;

  while(i < sizeof(ILI9486_regValues)) {
    uint8_t data = ILI9486_regValues[i++];
    uint8_t len = ILI9486_regValues[i++];
    if(data == TFTLCD_DELAY) {
      delay(len);
    } 
    else {
      CS_ACTIVE;
      CD_COMMAND;
      write8(data);
      CD_DATA;
      for (uint8_t j=0; j<len; j++) {
        write8(ILI9486_regValues[i++]);
      }
      CS_IDLE;
    }
  }
}

void Adafruit_ILI9486::setAddrWindow(int x1, int y1, int x2, int y2) 
{
  CS_ACTIVE;
  uint32_t t;

  t = x1;
  t <<= 16;
  t |= x2;
  writeRegister32(ILI9486_COLADDRSET, t);  
  t = y1;
  t <<= 16;
  t |= y2;
  writeRegister32(ILI9486_PAGEADDRSET, t); 
  CS_IDLE;
}

void Adafruit_ILI9486::setRotation(uint8_t x) 
{
  uint16_t t;

  // Call parent rotation func first -- sets up rotation flags, etc.
  Adafruit_GFX::setRotation(x);
  // Then perform hardware-specific rotation operations...

  CS_ACTIVE;

  switch (rotation) {
    case 1:
      t = ILI9486_MADCTL_MX | ILI9486_MADCTL_MY | ILI9486_MADCTL_MV | ILI9486_MADCTL_BGR;
      break;
    case 2:
      t = ILI9486_MADCTL_MX | ILI9486_MADCTL_BGR;
      break;
    case 3:
      t = ILI9486_MADCTL_MV | ILI9486_MADCTL_BGR;
      break;
    default:
      t = ILI9486_MADCTL_MY | ILI9486_MADCTL_BGR;
      break;
  }

  writeRegister8(ILI9486_MADCTL, t); // MADCTL
  // For 9486, init default full-screen address window:
  setAddrWindow(0, 0, _width - 1, _height - 1); // CS_IDLE happens here
}

void Adafruit_ILI9486::fillScreen(uint16_t color) 
{
  setAddrWindow(0, 0, _width - 1, _height - 1);
  flood(color, (long)TFTWIDTH * (long)TFTHEIGHT);
}

// Fast block fill operation for fillScreen, fillRect, H/V line, etc.
// Requires setAddrWindow() has previously been called to set the fill
// bounds.  'len' is inclusive, MUST be >= 1.
void Adafruit_ILI9486::flood(uint16_t color, uint32_t len) 
{
  uint16_t blocks;
  uint8_t  i, hi = color >> 8,
              lo = color;

  CS_ACTIVE;
  CD_COMMAND;
  write8(0x2C);

  // Write first pixel normally, decrement counter by 1
  CD_DATA;
  write8(hi);
  write8(lo);
  len--;

  blocks = (uint16_t)(len / 64); // 64 pixels/block 
  if(hi == lo) {
    // High and low bytes are identical.  Leave prior data
    // on the port(s) and just toggle the write strobe.
    while(blocks--) {
      i = 16; // 64 pixels/block / 4 pixels/pass
      do {
        WR_STROBE; WR_STROBE; WR_STROBE; WR_STROBE; // 2 bytes/pixel
        WR_STROBE; WR_STROBE; WR_STROBE; WR_STROBE; // x 4 pixels
      } while(--i);
    }
    // Fill any remaining pixels (1 to 64)
    for(i = (uint8_t)len & 63; i--; ) {
      WR_STROBE;
      WR_STROBE;
    }
  } else {
    while(blocks--) {
      i = 16; // 64 pixels/block / 4 pixels/pass
      do {
        write8(hi); write8(lo); write8(hi); write8(lo);
        write8(hi); write8(lo); write8(hi); write8(lo);
      } while(--i);
    }
    for(i = (uint8_t)len & 63; i--; ) {
      write8(hi);
      write8(lo);
    }
  }
  CS_IDLE;
}

void Adafruit_ILI9486::drawPixel(int16_t x, int16_t y, uint16_t color) 
{
  // Clip
  if((x < 0) || (y < 0) || (x >= _width) || (y >= _height)) return;

  CS_ACTIVE;
  setAddrWindow(x, y, _width-1, _height-1);
  CS_ACTIVE;
  CD_COMMAND; 
  write8(0x2C);
  CD_DATA; 
  write8(color >> 8); write8(color); 

  CS_IDLE;
}

uint16_t Adafruit_ILI9486::color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

//Draw XBitMap Files (*.xbm), exported from GIMP,
//Usage: Export from GIMP to *.xbm, rename *.xbm to *.c and open in editor.
//C Array can be directly used with this function
void Adafruit_ILI9486::drawXBitmap(int16_t x, int16_t y,
                              const uint8_t *bitmap, int16_t w, int16_t h,
                              uint16_t color) 
{
  int16_t i, j, byteWidth = (w + 7) / 8;
  
  for(j=0; j<h; j++) {
    for(i=0; i<w; i++ ) {
      if(pgm_read_byte(bitmap + j * byteWidth + i / 8) & (1 << (i % 8))) {
        drawPixel(x+i, y+j, color);
      }
    }
  }
}