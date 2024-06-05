/* RisibleRadar --- remake of Arduino game on STM32/SSD1351 2024-06-01 */

#include <stm32f4xx.h>

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>

#include "arrows.h"
#include "font.h"

#define ADC_RANGE    (4096)            // Range of 12-bit ADC (0-4095)
#define ADC_CENTRE   (ADC_RANGE / 2)   // Middle of range
#define ADC_DEADBAND (ADC_RANGE / 8)   // Deadband for joystick neutral

// Size of 128x128 OLED screen
#define MAXX 128
#define MAXY 128

// Co-ord of centre of screen
#define CENX (MAXX / 2)
#define CENY (MAXY / 2)

#define FONT_NCOLS   (5)
#define FONT_NROWS   (8)

#define SCANNER_RADIUS  (33)  // Radius of scanner display -- could increase with a power-up?

#define NTARGETS  10    // Number of randomly-placed radar targets on playfield
#define NECHOES   10    // Maximum number of echoes displayed

#define DEFGAMEDURATION  (40)  // Number of radar scanner sweeps allowed
#define MAXGAMEDURATION  (60)  // Maximum number of sweeps via power-up targets

#define TIMERX  (MAXX - 9)
#define TIMERY   4

#define MAXPLAYX (MAXX * 2)
#define MAXPLAYY (MAXY * 2)

#define RADTODEG  57.29578

enum cardinalDirections {
   NORTH = 1,
   SOUTH,
   EAST,
   WEST
};

// SSD1351 command bytes
#define SSD1351_SETCOLUMN      (0x15)
#define SSD1351_SETROW         (0x75)
#define SSD1351_WRITERAM       (0x5C)
#define SSD1351_READRAM        (0x5D)
#define SSD1351_SETREMAP       (0xA0)
#define SSD1351_STARTLINE      (0xA1)
#define SSD1351_DISPLAYOFFSET  (0xA2)
#define SSD1351_DISPLAYALLOFF  (0xA4)
#define SSD1351_DISPLAYALLON   (0xA5)
#define SSD1351_NORMALDISPLAY  (0xA6)
#define SSD1351_INVERTDISPLAY  (0xA7)
#define SSD1351_FUNCTIONSELECT (0xAB)
#define SSD1351_DISPLAYOFF     (0xAE)
#define SSD1351_DISPLAYON      (0xAF)
#define SSD1351_PRECHARGE      (0xB1)
#define SSD1351_DISPLAYENHANCE (0xB2)
#define SSD1351_CLOCKDIV       (0xB3)
#define SSD1351_SETVSL         (0xB4)
#define SSD1351_SETGPIO        (0xB5)
#define SSD1351_PRECHARGE2     (0xB6)
#define SSD1351_SETGRAY        (0xB8)
#define SSD1351_USELUT         (0xB9)
#define SSD1351_PRECHARGELEVEL (0xBB)
#define SSD1351_VCOMH          (0xBE)
#define SSD1351_CONTRASTABC    (0xC1)
#define SSD1351_CONTRASTMASTER (0xC7)
#define SSD1351_MUXRATIO       (0xCA)
#define SSD1351_COMMANDLOCK    (0xFD)
#define SSD1351_HORIZSCROLL    (0x96)
#define SSD1351_STOPSCROLL     (0x9E)
#define SSD1351_STARTSCROLL    (0x9F)

// SSD1351 16-bit 5/6/5 colours
#define SSD1351_BLACK          (0x0000)
#define SSD1351_RED            (0x001f)
#define SSD1351_GREEN          (0x07e0)
#define SSD1351_BLUE           (0xf800)
#define SSD1351_CYAN           (SSD1351_BLUE | SSD1351_GREEN)
#define SSD1351_MAGENTA        (SSD1351_RED  | SSD1351_BLUE)
#define SSD1351_YELLOW         (SSD1351_RED  | SSD1351_GREEN)
#define SSD1351_WHITE          (SSD1351_RED  | SSD1351_GREEN | SSD1351_BLUE)
#define SSD1351_GREY50         (0x000f | 0x03e0 | 0x7800)
#define SSD1351_GREY25         (0x0007 | 0x01e0 | 0x3800)

#define VFD_COLOUR             SSD1351_CYAN
#define LED_COLOUR             SSD1351_RED
#define PANAPLEX_COLOUR        (SSD1351_RED | 0x03e0)
#define PETROL_STATION_COLOUR  SSD1351_RED   // But some petrol stations use green

#define UART_RX_BUFFER_SIZE  (128)
#define UART_RX_BUFFER_MASK (UART_RX_BUFFER_SIZE - 1)
#if (UART_RX_BUFFER_SIZE & UART_RX_BUFFER_MASK) != 0
#error UART_RX_BUFFER_SIZE must be a power of two and <= 256
#endif

#define UART_TX_BUFFER_SIZE  (128)
#define UART_TX_BUFFER_MASK (UART_TX_BUFFER_SIZE - 1)
#if (UART_TX_BUFFER_SIZE & UART_TX_BUFFER_MASK) != 0
#error UART_TX_BUFFER_SIZE must be a power of two and <= 256
#endif

struct UART_RX_BUFFER
{
    volatile uint8_t head;
    volatile uint8_t tail;
    uint8_t buf[UART_RX_BUFFER_SIZE];
};

struct UART_TX_BUFFER
{
    volatile uint8_t head;
    volatile uint8_t tail;
    uint8_t buf[UART_TX_BUFFER_SIZE];
};

struct UART_BUFFER
{
    struct UART_TX_BUFFER tx;
    struct UART_RX_BUFFER rx;
};

// What style digits would we prefer?
enum STYLE {
   PANAPLEX_STYLE,
   LED_BAR_STYLE,
   LED_DOT_STYLE,
   PETROL_STATION_STYLE,
   VFD_STYLE
};


// UART buffers
struct UART_BUFFER U1Buf;

// The current echoes
struct echo_t {
   unsigned int x;
   unsigned int y;
   int age;
   int rad;
};

struct echo_t Echo[NECHOES];

// The targets
struct target_t {
   unsigned int x;
   unsigned int y;
   float bearing;
   float range;
   unsigned char siz;
   int active:1;
   int rings:1;
   int axes:1;
   int time:1;
};

struct target_t Target[NTARGETS];

struct coord_t {
   unsigned int x;
   unsigned int y;
};

struct coord_t Player;

int Gather_y = 3;

unsigned int GameDuration = DEFGAMEDURATION;

// Some attributes to "pick up" which are enhancements to the rather
// crude radar. Other things we could add here would be: longer echo
// persistence; faster sweep.
bool Rings = false;
bool Axes = false;

// The colour frame buffer, 32k bytes
uint16_t Frame[MAXY][MAXX];

volatile uint32_t Milliseconds = 0;
volatile uint8_t Tick = 0;
volatile uint8_t RtcTick = 0;


/* USART1_IRQHandler --- ISR for USART1, used for Rx and Tx */

void USART1_IRQHandler(void)
{
   if (USART1->SR & USART_SR_RXNE) {
      const uint8_t tmphead = (U1Buf.rx.head + 1) & UART_RX_BUFFER_MASK;
      const uint8_t ch = USART1->DR;  // Read received byte from UART
      
      if (tmphead == U1Buf.rx.tail)   // Is receive buffer full?
      {
          // Buffer is full; discard new byte
      }
      else
      {
         U1Buf.rx.head = tmphead;
         U1Buf.rx.buf[tmphead] = ch;   // Store byte in buffer
      }
   }
   
   if (USART1->SR & USART_SR_TXE) {
      if (U1Buf.tx.head != U1Buf.tx.tail) // Is there anything to send?
      {
         const uint8_t tmptail = (U1Buf.tx.tail + 1) & UART_TX_BUFFER_MASK;
         
         U1Buf.tx.tail = tmptail;

         USART1->DR = U1Buf.tx.buf[tmptail];    // Transmit one byte
      }
      else
      {
         USART1->CR1 &= ~USART_CR1_TXEIE; // Nothing left to send; disable Tx Empty interrupt
      }
   }
}


/* TIM4_IRQHandler --- ISR for TIM4, used for one-second real-time clock */

void TIM4_IRQHandler(void)
{
   TIM4->SR &= ~TIM_SR_UIF;   // Clear timer interrupt flag
   
   RtcTick = 1;
}


/* millis --- return milliseconds since reset */

uint32_t millis(void)
{
   return (Milliseconds);
}


/* SysTick_Handler --- ISR for System Timer overflow, used for 1ms ticker */

void SysTick_Handler(void)
{
   static uint8_t flag = 0;
   
   Milliseconds++;
   Tick = 1;
   
   // DEBUG: 500Hz on PC14 pin
   if (flag)
      GPIOC->BSRR = GPIO_BSRR_BR14; // GPIO pin PC14 LOW
   else
      GPIOC->BSRR = GPIO_BSRR_BS14; // GPIO pin PC14 HIGH
      
   flag = !flag;
}


/* UART1RxByte --- read one character from UART1 via the circular buffer */

uint8_t UART1RxByte(void)
{
   const uint8_t tmptail = (U1Buf.rx.tail + 1) & UART_RX_BUFFER_MASK;
   
   while (U1Buf.rx.head == U1Buf.rx.tail)  // Wait, if buffer is empty
       ;
   
   U1Buf.rx.tail = tmptail;
   
   return (U1Buf.rx.buf[tmptail]);
}


/* UART1RxAvailable --- return true if a byte is available in UART1 circular buffer */

int UART1RxAvailable(void)
{
   return (U1Buf.rx.head != U1Buf.rx.tail);
}


/* UART1TxByte --- send one character to UART1 via the circular buffer */

void UART1TxByte(const uint8_t data)
{
   const uint8_t tmphead = (U1Buf.tx.head + 1) & UART_TX_BUFFER_MASK;
   
   while (tmphead == U1Buf.tx.tail)   // Wait, if buffer is full
       ;

   U1Buf.tx.buf[tmphead] = data;
   U1Buf.tx.head = tmphead;

   USART1->CR1 |= USART_CR1_TXEIE;   // Enable UART1 Tx Empty interrupt
}


/* analogRead --- read the ADC */

uint16_t analogRead(const int channel)
{
   ADC1->SQR3 = channel;
   
   ADC1->CR2 |= ADC_CR2_SWSTART;
  
   while ((ADC1->SR & ADC_SR_EOC) == 0)
      ;
  
   return (ADC1->DR);
}


static void spi_cs(const int cs)
{
   if (cs)
      GPIOA->BSRR = GPIO_BSRR_BS4; // GPIO pin PA4 HIGH
   else
      GPIOA->BSRR = GPIO_BSRR_BR4; // GPIO pin PA4 LOW
}


static void spi_dc(const int dc)
{
   if (dc)
      GPIOA->BSRR = GPIO_BSRR_BS3; // GPIO pin PA3 HIGH
   else
      GPIOA->BSRR = GPIO_BSRR_BR3; // GPIO pin PA3 LOW
}


static uint8_t spi_txd(const uint8_t data)
{
   SPI1->DR = data;
   
   while ((SPI1->SR & SPI_SR_TXE) == 0)
      ;
      
   while ((SPI1->SR & SPI_SR_RXNE) == 0)
      ;
      
   return (SPI1->DR);
}


/* oledCmd --- send a command byte to the OLED by SPI */

static void oledCmd(const uint8_t c)
{
   spi_dc(0);
   spi_cs(0);
   spi_txd(c);
   spi_cs(1);
   spi_dc(1);
}


/* oledCmd1b --- send two command bytes to the OLED by SPI */

static void oledCmd1b(const uint8_t c, const uint8_t b)
{
   spi_dc(0);
   spi_cs(0);
   spi_txd(c);
   spi_dc(1);
   spi_txd(b);
   spi_cs(1);
}


/* oledCmd2b --- send three command bytes to the OLED by SPI */

static void oledCmd2b(const uint8_t c, const uint8_t b1, const uint8_t b2)
{
   spi_dc(0);
   spi_cs(0);
   spi_txd(c);
   spi_dc(1);
   spi_txd(b1);
   spi_txd(b2);
   spi_cs(1);
}


/* oledCmd3b --- send four command bytes to the OLED by SPI */

static void oledCmd3b(const uint8_t c, const uint8_t b1, const uint8_t b2, const uint8_t b3)
{
   spi_dc(0);
   spi_cs(0);
   spi_txd(c);
   spi_dc(1);
   spi_txd(b1);
   spi_txd(b2);
   spi_txd(b3);
   spi_cs(1);
}



/* updscreen --- update the physical screen from the buffer */

static void __attribute__((optimize("O3"))) updscreen(const uint8_t y1, const uint8_t y2)
{
    int x, y;
    volatile uint16_t __attribute__((unused)) junk;
    
    oledCmd2b(SSD1351_SETCOLUMN, 0, MAXX - 1);
    oledCmd2b(SSD1351_SETROW, y1, y2);
    
    oledCmd(SSD1351_WRITERAM);
    
    SPI1->CR1 |= SPI_CR1_DFF;    // 16-bit mode for just a bit more speed
    spi_cs(0);
    
    for (y = y1; y <= y2; y++)
        for (x = 0; x < MAXX; x++) {
            SPI1->DR = Frame[y][x];
   
            while ((SPI1->SR & SPI_SR_TXE) == 0)
               ;
      
            while ((SPI1->SR & SPI_SR_RXNE) == 0)
               ;
      
            junk = SPI1->DR;
        }
     
    spi_cs(1);
    SPI1->CR1 &= ~SPI_CR1_DFF;    // Back to 8-bit mode
}


/* OLED_begin --- initialise the SSD1351 OLED */

void OLED_begin(const int wd, const int ht)
{
    uint8_t remap = 0x60;
    
    // Init sequence for SSD1351 128x128 colour OLED module
    oledCmd1b(SSD1351_COMMANDLOCK, 0x12);
    oledCmd1b(SSD1351_COMMANDLOCK, 0xB1);
    
    oledCmd(SSD1351_DISPLAYOFF);
    
    oledCmd1b(SSD1351_CLOCKDIV, 0xF1);
    oledCmd1b(SSD1351_MUXRATIO, 127);
    oledCmd1b(SSD1351_DISPLAYOFFSET, 0x0);
    oledCmd1b(SSD1351_SETGPIO, 0x00);
    oledCmd1b(SSD1351_FUNCTIONSELECT, 0x01);
    oledCmd1b(SSD1351_PRECHARGE, 0x32);
    oledCmd1b(SSD1351_VCOMH, 0x05);
    oledCmd(SSD1351_NORMALDISPLAY);
    oledCmd3b(SSD1351_CONTRASTABC, 0xC8, 0x80, 0xC8);
    oledCmd1b(SSD1351_CONTRASTMASTER, 0x0F);
    oledCmd3b(SSD1351_SETVSL, 0xA0, 0xB5, 0x55);
    oledCmd1b(SSD1351_PRECHARGE2, 0x01);
    
    remap |= 0x10;   // Flip display vertically
    
    oledCmd1b(SSD1351_SETREMAP, remap);
    
    oledCmd(SSD1351_DISPLAYON); // Turn on OLED panel
}


/* drawPixel --- draw a single pixel */

void drawPixel(const unsigned int x, const unsigned int y, const uint16_t c)
{
    if ((x < MAXX) && (y < MAXY))
        Frame[y][x] = c;
    else
    {
//      Serial.print("drawPixel(");
//      Serial.print(x);
//      Serial.print(",");
//      Serial.print(y);
//      Serial.println(")");
    }
}


/* setText --- draw text into buffer using predefined font */

void setText(const int x, const int y, const char *str)
{
   // This function does not, as yet, allow for foreground and background
   // text colours to be passed in as parameters. It just draws white text
   // on a black background.
   int row, col;
   int i, j;
   const uint16_t fg = SSD1351_WHITE;
   const uint16_t bg = SSD1351_BLACK;
   
   col = x;

   for ( ; *str; str++) {
      const int d = (*str - ' ') * FONT_NCOLS;
    
      for (i = 0; i < FONT_NCOLS; i++) {
         const int bits = Font_data[d + i];
         row = y;
         
         for (j = 0; j < FONT_NROWS; j++) {
            if (bits & (1 << j))
               Frame[row][col] = fg;
            else
               Frame[row][col] = bg;
            
            row++;
         }
         
         col++;
      }
      
      row = y;
      
      for (j = 0; j < FONT_NROWS; j++) {
         Frame[row++][col] = bg;
      }
      
      col++;
   }
}


/* drawLine --- draw a line between any two absolute co-ords */

void drawLine(int x1, int y1, int x2, int y2, const int c)
{
   // Bresenham's line drawing algorithm. Originally coded on the IBM PC
   // with EGA card in 1986.
   int d;
   int i1, i2;
   int x, y;
   int xend, yend;
   int yinc, xinc;

   const int dx = abs(x2 - x1);
   const int dy = abs(y2 - y1);

   if (((y1 > y2) && (dx < dy)) || ((x1 > x2) && (dx > dy))) {
      int temp;
      
      temp = y1;
      y1 = y2;
      y2 = temp;

      temp = x1;
      x1 = x2;
      x2 = temp;
   }

   if (dy > dx) {
      d = (2 * dx) - dy;     /* Slope > 1 */
      i1 = 2 * dx;
      i2 = 2 * (dx - dy);

      if (y1 > y2) {
         x = x2;
         y = y2;
         yend = y1;
      }
      else {
         x = x1;
         y = y1;
         yend = y2;
      }

      if (x1 > x2)
         xinc = -1;
      else
         xinc = 1;

      drawPixel(x, y, c);

      while (y < yend) {
         y++;    
         if (d < 0)
            d += i1;
         else {
            x += xinc;
            d += i2;
         }

         drawPixel(x, y, c);
      }
   }
   else {          
      d = (2 * dy) - dx;  /* Slope < 1 */
      i1 = 2 * dy;
      i2 = 2 * (dy - dx);

      if (x1 > x2) {
         x = x2;
         y = y2;
         xend = x1;
      }
      else {
         x = x1;
         y = y1;
         xend = x2;
      }

      if (y1 > y2)
         yinc = -1;
      else
         yinc = 1;

      drawPixel(x, y, c);

      while (x < xend) {
         x++;
         if (d < 0)
            d += i1;
         else {
            y += yinc;
            d += i2;
         }

         drawPixel(x, y, c);
      }
   }
}


/* drawVline --- draw vertical line */

void drawVline(const unsigned int x, const unsigned int y1, const unsigned int y2, const uint16_t c)
{
   unsigned int y;

   for (y = y1; y <= y2; y++)
      Frame[y][x] = c;
}


/* drawHline --- draw pixels in a horizontal line */

void drawHline(const unsigned int x1, const unsigned int x2, const unsigned int y, const uint16_t c)
{
   unsigned int x;

   for (x = x1; x <= x2; x++)
      Frame[y][x] = c;
}


/* cfill --- draw horizontal lines to fill a circle */

static void cfill(const int x0, const int y0, const int x, const int y, const int c)
{
   drawHline(x0 - x, x0 + x, y0 + y, c);
   drawHline(x0 - x, x0 + x, y0 - y, c);
   drawHline(x0 - y, x0 + y, y0 + x, c);
   drawHline(x0 - y, x0 + y, y0 - x, c);
}


/* cpts4 --- draw four pixels to form the edge of a circle */

static void cpts4(const int x0, const int y0, const int x, const int y, const int c)
{
   drawPixel(x0 + x, y0 + y, c);

//  if (x != 0)
   drawPixel(x0 - x, y0 + y, c);

//  if (y != 0)  
   drawPixel(x0 + x, y0 - y, c);

//  if ((x != 0) && (y != 0))
   drawPixel(x0 - x, y0 - y, c);
}


/* cpts8 --- draw eight pixels to form the edge of a circle */

static void cpts8(const int x0, const int y0, const int x, const int y, const int c)
{
  cpts4 (x0, y0, x, y, c);

// if (x != y)
    cpts4 (x0, y0, y, x, c);
}


/* splitcfill --- draw horizontal lines to fill a circle */

static void splitcfill(const int x0, const int y0, const int x1, const int y1, const int x, const int y, const int c)
{
   drawHline(x0 - x, x1 + x, y1 + y, c);
   drawHline(x0 - x, x1 + x, y0 - y, c);
   drawHline(x0 - y, x1 + y, y1 + x, c);
   drawHline(x0 - y, x1 + y, y0 - x, c);
}


/* splitcpts4 --- draw four pixels to form the edge of a split circle */

static void splitcpts4(const int x0, const int y0, const int x1, const int y1, const int x, const int y, const int c)
{
   drawPixel(x1 + x, y1 + y, c);

//  if (x != 0)
   drawPixel(x0 - x, y1 + y, c);

//  if (y != 0)  
   drawPixel(x1 + x, y0 - y, c);

//  if ((x != 0) && (y != 0))
   drawPixel(x0 - x, y0 - y, c);
}


/* splitcpts8 --- draw eight pixels to form the edge of a split circle */

static void splitcpts8(const int x0, const int y0, const int x1, const int y1, const int x, const int y, const int c)
{
   splitcpts4(x0, y0, x1, y1, x, y, c);

// if (x != y)
      splitcpts4(x0, y0, x1, y1, y, x, c);
}


/* circle --- draw a circle with edge and fill colours */

void circle(const int x0, const int y0, const int r, const int ec, const int fc)
{
   // Michener's circle algorithm. Originally coded on the IBM PC
   // with EGA card in 1986.
   int x, y;
   int d;

   x = 0;
   y = r;
   d = 3 - (2 * r);

   if (fc >= 0) {
      while (x < y) {
         cfill(x0, y0, x, y, fc);
         if (d < 0) {
            d += (4 * x) + 6;
         }
         else {
            d += (4 * (x - y)) + 10;
            y--;
         }
         x++;
      }

      if (x == y)
         cfill(x0, y0, x, y, fc);
   }

   x = 0;
   y = r;
   d = 3 - (2 * r);

   while (x < y) {
      cpts8(x0, y0, x, y, ec);
      if (d < 0) {
         d += (4 * x) + 6;
      }
      else {
         d += (4 * (x - y)) + 10;
         y--;
      }
      x++;
   }

   if (x == y)
      cpts8(x0, y0, x, y, ec);
}


/* drawSplitCircle --- draw a split circle with edge and fill colours */

void drawSplitCircle(const int x0, const int y0, const int x1, const int y1, const int r, const int ec, const int fc)
{
   // Michener's circle algorithm. Originally coded on the IBM PC
   // with EGA card in 1986.
   int x, y;
   int d;

   x = 0;
   y = r;
   d = 3 - (2 * r);

   if (fc >= 0) {
      while (x < y) {
         splitcfill(x0, y0, x1, y1, x, y, fc);
         if (d < 0) {
            d += (4 * x) + 6;
         }
         else {
            d += (4 * (x - y)) + 10;
            y--;
         }
         x++;
      }

      if (x == y)
         splitcfill(x0, y0, x1, y1, x, y, fc);
   }

   x = 0;
   y = r;
   d = 3 - (2 * r);

   while (x < y) {
      splitcpts8(x0, y0, x1, y1, x, y, ec);
      if (d < 0) {
         d += (4 * x) + 6;
      }
      else {
         d += (4 * (x - y)) + 10;
         y--;
      }
      x++;
   }

   if (x == y)
      splitcpts8(x0, y0, x1, y1, x, y, ec);
}


/* fillRoundRect --- fill a rounded rectangle */

void fillRoundRect(const int x0, const int y0, const int x1, const int y1, const int r, const uint16_t ec, const uint16_t fc)
{
   int y;

   drawSplitCircle(x0 + r, y0 + r, x1 - r, y1 - r, r, ec, fc);

   drawHline(x0 + r, x1 - r, y0, ec);
   drawHline(x0 + r, x1 - r, y1, ec);
   drawVline(x0, y0 + r, y1 - r, ec);
   drawVline(x1, y0 + r, y1 - r, ec);

   for (y = y0 + r; y < (y1 - r); y++)
      drawHline (x0 + 1, x1 - 1, y, fc);
}


/* textRoundRect --- draw text centralised in a rounded rectangle */

void textRoundRect(const char *const str, const int ec, const int fc, const int tc)
{
   const int len = strlen(str);
   const int x1 = CENX - (3 * len);
   const int x2 = CENX + (3 * len);
   
   fillRoundRect(x1 - 2, CENY - 8, x2 + 2, CENY + 12, 7, ec, fc);
   setText(x1, CENY, str);
}


/* greyFrame --- clear entire frame to checkerboard pattern */

void greyFrame(void)
{
    int r, c;

    for (r = 0; r < MAXY; r += 2)
    {
        for (c = 0; c < MAXX; c += 2)
        {
            Frame[r][c] = SSD1351_BLACK;
            Frame[r][c + 1] = SSD1351_WHITE;
        }
        
        for (c = 0; c < MAXX; c += 2)
        {
            Frame[r + 1][c] = SSD1351_WHITE;
            Frame[r + 1][c + 1] = SSD1351_BLACK;
        }
    }
}


/* setRect --- set pixels in a (non-filled) rectangle */

void setRect(const int x1, const int y1, const int x2, const int y2, const uint16_t c)
{
    drawHline(x1, x2, y1, c);
    drawVline(x2, y1, y2, c);
    drawHline(x1, x2, y2, c);
    drawVline(x1, y1, y2, c);
}


/* fillRect --- set pixels in a filled rectangle */

void fillRect(const int x1, const int y1, const int x2, const int y2, const uint16_t ec, const uint16_t fc)
{
    int y;

    for (y = y1; y <= y2; y++)
        drawHline(x1, x2, y, fc);

    drawHline(x1, x2, y1, ec);
    drawVline(x2, y1, y2, ec);
    drawHline(x1, x2, y2, ec);
    drawVline(x1, y1, y2, ec);
}


/* renderBitmap --- render pixels into the framebuffer according to a bitmap */

void renderBitmap(const int x1, const int y1, const int wd, const int ht, const uint8_t *bitmap, const int stride, const uint16_t fg, const uint16_t bg)
{
    int x, y;
    int i, j;
    const uint8_t *row;
    const int x2 = x1 + wd - 1;
    const int y2 = y1 + ht - 1;
    
    for (y = y1, i = 0; y <= y2; y++, i++) {
        row = bitmap + (stride * (i / 8));
        
        for (x = x1, j = 0; x <= x2; x++, j++)
            if (row[j] & (1 << (i % 8)))
                Frame[y][x] = fg;
            else
                Frame[y][x] = bg;
    }
}


/* drawBackground --- draw the screen background */

void drawBackground(void)
{
   int x, y;

   // Checkerboard background
   for (y = 0; y < MAXY; y++) {
      for (x = 0; x < MAXX; x += 2) {
         if (y & 1) {
            drawPixel(x, y, SSD1351_WHITE);
            drawPixel(x + 1, y, SSD1351_BLACK);
         }
         else {
            drawPixel(x, y, SSD1351_BLACK);
            drawPixel(x + 1, y, SSD1351_WHITE);
         }  
      }
   }

   // Four cases where the edge of the playing area is visible
   if (Player.x < CENX) {
      fillRect(0, 0, CENX - Player.x, MAXY - 1, SSD1351_BLACK, SSD1351_BLACK);
   }

   if (Player.y < CENY) {
      fillRect(0, 0, MAXX - 1, CENY - Player.y, SSD1351_BLACK, SSD1351_BLACK);
   }

   if ((MAXPLAYX - Player.x) < CENX) {
      fillRect((MAXPLAYX - Player.x) + CENX, 0, MAXX - 1, MAXY - 1, SSD1351_BLACK, SSD1351_BLACK);
   }

   if ((MAXPLAYY - Player.y) < CENY) {
      fillRect(0, (MAXPLAYY - Player.y) + CENY, MAXX - 1, MAXY - 1, SSD1351_BLACK, SSD1351_BLACK);
   }
}


/* drawRadarScreen --- draw the basic circular radar scope */

void drawRadarScreen(const int radius, const bool rings, const bool axes)
{
//  unsigned long int before, after;

  // 1108us
//  before = micros ();
   circle(CENX, CENY, radius, SSD1351_WHITE, SSD1351_BLACK);
//  after = micros ();
  
//  Serial.print (after - before);
//  Serial.println ("us. circle 33");
   
// circle(11, 11, 11, SSD1351_RED, -1);
// circle((MAXX - 1) - 11, 11, 11, SSD1351_RED, -1);
// circle((MAXX - 1) - 11, (MAXY - 1) - 11, 11, SSD1351_RED, -1);
// circle(11, (MAXY - 1) - 11, 11, SSD1351_RED, -1);
   
   // Range rings
   if (rings) {
      circle(CENX, CENY, radius / 3, SSD1351_GREEN, -1);
      circle(CENX, CENY, (radius * 2) / 3, SSD1351_GREEN, -1);
   }
  
   // Cardinal directions
   if (axes) {
      drawVline(CENX, CENY - radius, CENY + radius, SSD1351_WHITE);
      drawHline(CENX - radius, CENX + radius, CENY, SSD1351_WHITE);
   }
}


/* drawGatheredTargets --- draw the targets that we've walked over */

void drawGatheredTargets(void)
{
   int t;
   
   for (t = 0; t < NTARGETS; t++) {
      if (Target[t].active == false) {
         circle(6, Target[t].y, Target[t].siz, SSD1351_WHITE, 1);

      if (Target[t].rings)
         circle(12, Target[t].y, 2, SSD1351_WHITE, -1);

      if (Target[t].axes)
         drawPixel(12, Target[t].y, SSD1351_WHITE);
      }
   }
}


/* drawTimer --- visualise the game timer */

void drawTimer(const unsigned int sweeps)
{
   // Very simple vertical bar on RHS of screen.
   unsigned int y;

   fillRect(MAXX - 10, TIMERY, MAXX - 1, MAXGAMEDURATION + TIMERY, 1, 0);
  
   for (y = 1; y <= GameDuration; y++)
      if (y < sweeps)
         drawHline(MAXX - 9, MAXX - 2, y + TIMERY, SSD1351_BLACK);
      else
         drawHline(MAXX - 9, MAXX - 2, y + TIMERY, SSD1351_WHITE);
  
   drawHline(MAXX - 9, MAXX - 2, GameDuration + TIMERY, SSD1351_WHITE);
}


/* drawRadarVector --- draw the radial line representing the current scan vector */

void drawRadarVector(const int radius, const int angle)
{
   // This function draws the radial line three times to make it
   // appear more clearly on the rather slow LCD. A better way
   // would be to draw a narrow sector (pie-slice) so that the
   // pixels have time to fully darken before they get switched
   // back to white. But that would require an efficient sector
   // drawing routine, which we don't have (yet).
   int x, y;
   const float r = (float)radius;

   // 252us
   x = (r * cos ((double)angle / RADTODEG)) + 0.49;
   y = (r * sin ((double)angle / RADTODEG)) + 0.49;

   // 232us
   drawLine(CENX, CENY, CENX + x, CENY + y, SSD1351_GREEN);

   x = (r * cos ((double)(angle + 2) / RADTODEG)) + 0.49;
   y = (r * sin ((double)(angle + 2) / RADTODEG)) + 0.49;

   // 232us
   drawLine(CENX, CENY, CENX + x, CENY + y, SSD1351_GREEN);

   x = (r * cos ((double)(angle + 4) / RADTODEG)) + 0.49;
   y = (r * sin ((double)(angle + 4) / RADTODEG)) + 0.49;

   drawLine(CENX, CENY, CENX + x, CENY + y, SSD1351_GREEN);
}


/* reCalculateBearings --- update array of target bearings from new player position */

void reCalculateBearings(void)
{
   // We need to know the bearing from the player to each of the radar
   // targets, so that we can rapidly update the display as the "beam" rotates.
   // In this function, we update the array of bearings and ranges after the
   // player position has changed. 'atan2' computes the arctangent, giving a
   // bearing, without the risk of dividing by zero. The result is -180 to +180
   // degrees, so we add 360 to any negative bearings. Range is worked out by
   // Pythagoras' theorem.
   int i;

   for (i = 0; i < NTARGETS; i++) {
      if (Target[i].active) {
         const int dx = Target[i].x - Player.x;
         const int dy = Target[i].y - Player.y;

         Target[i].bearing = atan2(dy, dx) * RADTODEG;
         Target[i].range = sqrt((dx * dx) + (dy * dy));

         if (Target[i].bearing < 0.0)
            Target[i].bearing += 360.0;
      }
   }
}


/* findEchoSlot --- search the Echo array for an unused slot */

int findEchoSlot(void)
{
   int e;
   
   for (e = 0; e < NECHOES; e++) {
      if (Echo[e].age <= 0)
         return (e);
   }
   
   // We didn't find an empty slot, so overwrite slot 0  
   return (0);
}


/* findNewEchoes --- search the Target array for anything that will cause an echo */

void findNewEchoes(const int r, const int range, const int nt)
{
   // Targets have an 'active' flag so that we can make them disappear
   // after the user walks over them -- currently unimplemented.
   // Some potential additions here: targets that are close to the radar
   // appear larger; make some echoes fade more quickly; give echoes
   // shapes other than circles.
   int t;
   int e;
   const float frange = (float)range;
   const float pickup = frange / 3.0;

   for (t = 0; t < nt; t++) {
      if (Target[t].active) {                            // Currently active?
         if (abs(Target[t].bearing - (float)r) < 6.0) {  // In the right direction?
           if (Target[t].range < frange) {               // Close enough?
              // Make a new echo
              e = findEchoSlot();
              Echo[e].x = CENX + (Target[t].x - Player.x);  // Make player-relative co-ordinates
              Echo[e].y = CENY + (Target[t].y - Player.y);
              Echo[e].age = 90;                             // Echoes last 3/4 of a revolution
              Echo[e].rad = Target[t].siz;                  // Target size affects echo size
             
              if (Target[t].range < pickup) {  // Pick it up?
                 Target[t].active = false;
                 Target[t].y = Gather_y;
                 Gather_y += 6;
              }
           }
         }

         if (Target[t].range < pickup) {  // Close enough for bonus (regardless of bearing)?
            if (Target[t].rings)
               Rings = true;      // Enable range rings
             
            if (Target[t].axes)
               Axes = true;       // Enable axes
             
            if (Target[t].time) {
               if (GameDuration < MAXGAMEDURATION)    // Give user more time
                  GameDuration += 5;
             
               Target[t].time = false;  // Only trigger once!
            }
         }
      }
   }
}


/* getPlayerMove --- read the analog joystick */

int getPlayerMove(void)
{
   // The analog joystick is on STM32 analog pins 1 and 8 for
   // X and Y respectively. The range of an analog input is
   // 0-4095 (12 bits), so the middle position is about 2048.
   // At present, only four movement directions are possible.
   int x, y;
   int dir = 0;

   x = analogRead(1);
   y = analogRead(8);

   if (x < (ADC_CENTRE - ADC_DEADBAND))
     dir = WEST;
   else if (x > (ADC_CENTRE + ADC_DEADBAND))
     dir = EAST;

   if (y < (ADC_CENTRE - ADC_DEADBAND))
     dir = NORTH;
   else if (y > (ADC_CENTRE + ADC_DEADBAND))
     dir = SOUTH;
    
   //printf("Joy: %d (%d,%d)\n", dir, x, y);

   switch (dir) {
   case 0:
      break;
   case NORTH:
      renderBitmap(0, 0, 24, 24, &Arrows[0][0], 240, SSD1351_GREEN, SSD1351_BLACK);
      //setText(0, 0, "North");
      break;
   case SOUTH:
      renderBitmap(0, 0, 24, 24, &Arrows[0][24], 240, SSD1351_GREEN, SSD1351_BLACK);
      //setText(0, 0, "South");
      break;
   case EAST:
      setText(0, 0, "East");
      break;
   case WEST:
      setText(0, 0, "West");
      break;
   }

   return (dir);
}


/* movePlayer --- make player move and update bearings */

void movePlayer(const int dir)
{
   switch (dir) {
   case NORTH:
      if (Player.y > 0)
         Player.y--;
      break;
   case SOUTH:
      if (Player.y < (MAXPLAYY - 1))
         Player.y++;
      break;
   case WEST:
      if (Player.x > 0)
         Player.x--;
      break;
   case EAST:
      if (Player.x < (MAXPLAYX - 1))
         Player.x++;
      break;
   }
  
   //printf("New player pos: (%d,%d)\n", Player.x, Player.y);
}


/* _write --- connect stdio functions to UART1 */

int _write(const int fd, const char *ptr, const int len)
{
   int i;

   for (i = 0; i < len; i++) {
      if (*ptr == '\n')
         UART1TxByte('\r');
      
      UART1TxByte(*ptr++);
   }
  
   return (len);
}


/* delay --- Arduino-like function to delay for miliiseconda */

void delay(const int milliSeconds)
{
   const int end = millis() + milliSeconds;
   
   while (millis() < end)
      ;
}


/* random --- Arduino-like function to return a random number */

int random(const int low, const int high)
{
   const int span = high - low;
   const float r = (float)rand() / (float)RAND_MAX;
   
   return ((int)((low + (r * span)) + 0.5));
}


/* game_setup --- initialise the game logic */

void game_setup(void)
{
   int i;
   
   OLED_begin(MAXX, MAXY);
   
   greyFrame();
    
   updscreen(0, MAXY - 1);
   
   printf("RisibleRadar\n");
   printf("John Honniball, June 2024\n");
   printf("Ludum Dare MiniLD #34: Aspect\n");
   
   // Targets scattered on playfield at random
   for (i = 0; i < NTARGETS; i++) {
      do {
         Target[i].x = random(0, MAXPLAYX);
         Target[i].y = random(0, MAXPLAYY);
      
         Target[i].active = true;
         Target[i].rings = false;
         Target[i].axes = false;
         Target[i].time = false;
         Target[i].siz = random(1, 3);
         printf("%d: (%d, %d) siz: %d\n", i, Target[i].x, Target[i].y, Target[i].siz);
         // TODO: make sure no two targets are too close together
      } while (0);
   }
  
   // Place the 'bonus' targets somewhere
   i = random (0, NTARGETS - 1);
   Target[i].rings = true;

   i = random (0, NTARGETS - 1);
   Target[i].axes = true;

   i = random (0, NTARGETS - 1);
   Target[i].time = true;

   i = random (0, NTARGETS - 1);
   Target[i].time = true;

   // Start the player in centre of playfield
   Player.x = MAXPLAYX / 2;
   Player.y = MAXPLAYY / 2;
   
   reCalculateBearings();

   drawBackground();

   drawRadarScreen(SCANNER_RADIUS, true, true);

   textRoundRect("Risible Radar", SSD1351_WHITE, SSD1351_BLACK, SSD1351_WHITE);

   updscreen(0, MAXY - 1);

   delay(2000);

   drawBackground();

   drawRadarScreen(SCANNER_RADIUS, true, true);

   textRoundRect("READY", SSD1351_WHITE, SSD1351_BLACK, SSD1351_WHITE);

   updscreen(0, MAXY - 1);
   
   // Wait here for user to press Start
}


/* game_loop --- main loop for the RisibleRadar game */

void game_loop(void)
{
   static unsigned int sweeps = 0;
   int r;
   int dir;
   int e;
   long int start, now;
   int elapsed;

   for (r = 0; r < 360; r += 3) {
      // Record timer in milliseconds at start of frame cycle
      start = millis();

      // Draw empty radar scope
      drawBackground();

      drawRadarScreen(SCANNER_RADIUS, Rings, Axes);

      drawGatheredTargets();

      if (sweeps < GameDuration) {
         dir = getPlayerMove();

         if (dir != 0) {
            movePlayer(dir);
            reCalculateBearings();
         }
      }
    
      // Draw current scan vector
      drawRadarVector(SCANNER_RADIUS, r);

      // Do we have any new echoes for this scanner bearing?
      findNewEchoes(r, SCANNER_RADIUS, NTARGETS);

      // Add un-faded echoes
      for (e = 0; e < NECHOES; e++) {
         if (Echo[e].age > 0)
            circle(Echo[e].x, Echo[e].y, Echo[e].rad, SSD1351_GREEN, SSD1351_GREEN);

         Echo[e].age--;
      }
    
      if (r == 180)
         sweeps++;
      
      if (sweeps < GameDuration) {
         drawTimer(sweeps);
      }
      else {
         textRoundRect("GAME OVER", SSD1351_WHITE, SSD1351_BLACK, SSD1351_WHITE);
      }
      
      // Update LCD for this frame
      updscreen(0, MAXY - 1);
      
      // Work out timing for this frame
      now = millis();
      elapsed = now - start;
    
//    printf("%dms.\n", elapsed);
    
      if (elapsed < 40)
         delay(40 - elapsed);
   }

   sweeps++;
}


/* initMCU --- set up the microcontroller in general */

static void initMCU(void)
{
   RCC->CR      = 0x00000081;
   RCC->CFGR    = 0x00000000;
   RCC->PLLCFGR = 0x24003010;
   
   FLASH->ACR |= FLASH_ACR_LATENCY_2WS;   // Set Flash latency to 2 wait states
   FLASH->ACR |= FLASH_ACR_ICEN;          // Cache enable
   FLASH->ACR |= FLASH_ACR_DCEN;
   FLASH->ACR |= FLASH_ACR_PRFTEN;        // Prefetch enable
   
   RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;   // Set APB1 bus to not exceed 50MHz
   //RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;    // Set APB2 bus to not exceed 100MHz
   
#if 0
   // Switch on MCO1 for debugging
   RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;         // Enable clock to GPIO A peripherals on AHB1 bus

   // Configure PA8, the GPIO pin with alternative function MCO1
   GPIOA->MODER |= GPIO_MODER_MODER8_1;          // PA8 in Alternative Function mode
   GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR8_1 | GPIO_OSPEEDER_OSPEEDR8_0;

   RCC->CFGR |= RCC_CFGR_MCO1_1 | RCC_CFGR_MCO1_0;                 // Send PLL to MCO1
   RCC->CFGR |= RCC_CFGR_MCO1PRE_0 | RCC_CFGR_MCO1PRE_1 | RCC_CFGR_MCO1PRE_2; // Prescale divide-by-5
#endif
   
   RCC->CR |= RCC_CR_HSEON;    // Switch on High Speed External clock (25MHz on the Black Pill)

   // Wait for HSE to start up
   while ((RCC->CR & RCC_CR_HSERDY) == 0)
      ;
   
   // Configure PLL
   RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLQ;
   RCC->PLLCFGR |= RCC_PLLCFGR_PLLQ_2 | RCC_PLLCFGR_PLLQ_1 | RCC_PLLCFGR_PLLQ_0;   // Set Q to divide-by 7

   RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLP;            // P = divide-by-2

   RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLN;
   RCC->PLLCFGR |= 200 << RCC_PLLCFGR_PLLN_Pos;    // N = multiply-by-200

   RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM;
   RCC->PLLCFGR |= 25 << RCC_PLLCFGR_PLLM_Pos;     // M = divide-by-25 for 1MHz PLL input

   RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;    // Select HSE as PLL input
   
   RCC->CR |= RCC_CR_PLLON;             // Switch on the PLL

   // Wait for PLL to start up
   while ((RCC->CR & RCC_CR_PLLRDY) == 0)
      ;

   uint32_t reg = RCC->CFGR;
   reg &= ~RCC_CFGR_SW;
   reg |= RCC_CFGR_SW_PLL;          // Select PLL as system clock (100MHz)
   RCC->CFGR = reg;
   
   // Wait for PLL to select
   while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
      ;
   
   RCC->CSR |= RCC_CSR_RMVF;
}


/* initGPIOs --- set up the GPIO pins */

static void initGPIOs(void)
{
   // Configure Reset and Clock Control
   RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;      // Enable clock to GPIO A peripherals on AHB1 bus
   RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;      // Enable clock to GPIO B peripherals on AHB1 bus
   RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;      // Enable clock to GPIO C peripherals on AHB1 bus
   
   // Configure PA0, the GPIO pin with the button
   GPIOA->PUPDR |= GPIO_PUPDR_PUPD0_0;       // Enable pull-up on PA0
   
   // Configure PB12, the GPIO pin with the red LED
   GPIOB->MODER |= GPIO_MODER_MODER12_0;     // Configure PB12 as output for the red LED
   
   // Configure PB13, the GPIO pin with the green LED
   GPIOB->MODER |= GPIO_MODER_MODER13_0;     // Configure PB13 as output for the green LED
   
   // Configure PB14, the GPIO pin with the blue LED
   GPIOB->MODER |= GPIO_MODER_MODER14_0;     // Configure PB14 as output for the blue LED
   
   // Configure PC13, the GPIO pin with the on-board blue LED
   GPIOC->MODER |= GPIO_MODER_MODER13_0;     // Configure PC13 as output for the on-board LED
   
   // Configure PC14, the GPIO pin with 500Hz square wave
   GPIOC->MODER |= GPIO_MODER_MODER14_0;     // Configure PC14 as output, 500Hz square wave
}


/* initUARTs --- set up UART(s) and buffers, and connect to 'stdout' */

static void initUARTs(void)
{
   RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;        // Enable clock to GPIO A peripherals on AHB1 bus
   RCC->APB2ENR |= RCC_APB2ENR_USART1EN;       // Enable USART1 clock
   
   // Set up UART1 and associated circular buffers
   U1Buf.tx.head = 0;
   U1Buf.tx.tail = 0;
   U1Buf.rx.head = 0;
   U1Buf.rx.tail = 0;
   
   // Configure PA9, the GPIO pin with alternative function TxD2
   GPIOA->MODER |= GPIO_MODER_MODER9_1;        // PA9 in Alternative Function mode
   GPIOA->AFR[1] |= 7 << 4;                    // Configure PA9 as alternate function, AF7, UART1
  
   // Configure PA10, the GPIO pin with alternative function RxD2
   GPIOA->MODER |= GPIO_MODER_MODER10_1;       // PA10 in Alternative Function mode
   GPIOA->AFR[1] |= 7 << 8;                    // Configure PA10 as alternate function, AF7, UART1
   
   // Configure UART1 - defaults are 1 start bit, 8 data bits, 1 stop bit, no parity
   USART1->CR1 |= USART_CR1_UE;           // Switch on the UART
   USART1->BRR |= (651<<4) | 1;           // Set for 9600 baud (reference manual page 518) 100000000 / (16 * 9600)
   USART1->CR1 |= USART_CR1_RXNEIE;       // Enable Rx Not Empty interrupt
   USART1->CR1 |= USART_CR1_TE;           // Enable transmitter (sends a junk character)
   USART1->CR1 |= USART_CR1_RE;           // Enable receiver
   
   NVIC_EnableIRQ(USART1_IRQn);
}


/* initTimers --- set up timer for regular 1Hz interrupts */

static void initTimers(void)
{
   // The STM32F103 does not have Basic Timers 6 and 7. So, we'll use TIM4 to
   // generate regular interrupts. TIM4 has a 16-bit prescaler and a 16-bit counter register
   
   RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;        // Enable Timer 4 clock
   
   TIM4->CR1 = 0;               // Start with default CR1 and CR2
   TIM4->CR2 = 0;
   TIM4->CCMR1 = 0;             // No output compare mode PWM
   TIM4->CCMR2 = 0;
   TIM4->CCER = 0;              // No PWM outputs enabled
   TIM4->PSC = 10000 - 1;       // Prescaler: 100MHz, divide-by-10000 to give 10kHz
   TIM4->ARR = 10000 - 1;       // Auto-reload: 10000 to give interrupts at 1Hz
   TIM4->CNT = 0;               // Counter: 0
   TIM4->DIER |= TIM_DIER_UIE;  // Enable interrupt
   TIM4->CR1 |= TIM_CR1_CEN;    // Enable counter
   
   NVIC_EnableIRQ(TIM4_IRQn);
}


/* initSPI --- set up the SPI interface */

static void initSPI(void)
{
   // Configure Reset and Clock Control
   RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;                    // Enable clock to SPI1 peripheral on APB2 bus
   RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;                   // Enable clock to GPIO A peripherals on AHB1 bus
   
   // Configure PA5, the GPIO pin with alternative function SCK1
   GPIOA->MODER |= GPIO_MODER_MODER5_1;        // PA5 in Alternative Function mode
   GPIOA->AFR[0] |= 5 << 20;                   // Configure PA5 as alternate function, AF5, SCK1
   
   // Configure PA6, the GPIO pin with alternative function MISO1
   GPIOA->MODER |= GPIO_MODER_MODER6_1;        // PA6 in Alternative Function mode
   GPIOA->AFR[0] |= 5 << 24;                   // Configure PA6 as alternate function, AF5, MISO1
   
   // Configure PA7, the GPIO pin with alternative function MOSI1
   GPIOA->MODER |= GPIO_MODER_MODER7_1;        // PA7 in Alternative Function mode
   GPIOA->AFR[0] |= 5 << 28;                   // Configure PA7 as alternate function, AF5, MOSI1
   
   // Configure PA4, the GPIO pin with function CS
   GPIOA->MODER |= GPIO_MODER_MODER4_0;      // Configure PA4 as output for CS
   
   // Configure PA3, the GPIO pin with function DC
   GPIOA->MODER |= GPIO_MODER_MODER3_0;      // Configure PA3 as output for DC
   
   // Set up SPI1
   SPI1->CR1 = 0;
   SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR;
   SPI1->CR1 |= SPI_CR1_CPOL | SPI_CR1_CPHA;
   SPI1->CR1 |= SPI_CR1_BR_1; // 100MHz divide-by-8 gives 12.5MHz
   SPI1->CR1 |= SPI_CR1_SPE;  // Enable SPI
   
   spi_cs(1);
   spi_dc(1);
}


/* initADC --- set up the ADC */

static void initADC(void)
{
   // Configure Reset and Clock Control
   RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;                    // Enable clock to ADC peripheral on APB2 bus
   RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;                   // Enable clock to GPIO A peripherals on AHB1 bus
   
   // Configure PA1, the GPIO pin with alternative function ADC1
   GPIOA->MODER |= GPIO_MODER_MODER1_1 | GPIO_MODER_MODER1_0;    // PA1 in Analog mode
   
   // Configure PB0, the GPIO pin with alternative function ADC8
   GPIOB->MODER |= GPIO_MODER_MODER0_1 | GPIO_MODER_MODER0_0;    // PB0 in Analog mode
   
   ADC1->CR1 = 0x0;  // Default set-up: 12 bit
   ADC1->CR2 = 0x0;
   ADC1->SMPR1 = 0x0;
   ADC1->SMPR2 = 0x0;
   ADC1->SQR1 = 0x0;
   ADC1->SQR2 = 0x0;
   ADC1->SQR3 = 0x0;
   
   ADC1->CR2 |= ADC_CR2_ADON; // Enable ADC
   
   ADC1->SMPR2 |= 4 << ADC_SMPR2_SMP1_Pos;
   ADC1->SMPR2 |= 4 << ADC_SMPR2_SMP8_Pos;
   
   ADC1->SQR3 |= 0x1;   // Convert just channel 1
}


/* initMillisecondTimer --- set up a timer to interrupt every millisecond */

static void initMillisecondTimer(void)
{
   // Set up timer for regular 1ms interrupt
   if (SysTick_Config(100000)) { // 100MHz divided by 1000  (SystemCoreClock / 1000)
      while (1)
         ;
   }
}


int main(void)
{
   uint32_t end;
   uint32_t frame;
   uint8_t flag = 0;
   
   initMCU();
   initGPIOs();
   initUARTs();
   initSPI();
   initADC();
   initTimers();
   initMillisecondTimer();
   
   __enable_irq();   // Enable all interrupts
   
   printf("\nHello from the STM%dF%d\n", 32, 411);
   
   game_setup();
   
   end = millis() + 500u;
   frame = millis() + 40u;
   
   while (1)
      game_loop();
   
   while (1) {
      if (Tick) {
         if (millis() >= end) {
            end = millis() + 500u;
            
            if (flag) {
               GPIOC->BSRR = GPIO_BSRR_BR13; // GPIO pin PC13 LOW, LED on
            }
            else {
               GPIOC->BSRR = GPIO_BSRR_BS13; // GPIO pin PC13 HIGH, LED off
            }
            
            flag = !flag;
            
            printf("millis() = %ld\n", millis());
         }
         
         if (millis() >= frame) {
            frame = millis() + 40u;
            
            const uint16_t ana1 = analogRead(1) / 32;
            const uint16_t ana2 = analogRead(8) / 32;
            fillRect(0, 32, 127, 63, SSD1351_WHITE, SSD1351_BLACK);
            fillRect(1, 33, ana1, 47, SSD1351_BLUE, SSD1351_BLUE);
            fillRect(1, 48, ana2, 62, SSD1351_BLUE, SSD1351_BLUE);
            updscreen(32, 63);
         }
         
         Tick = 0;
      }
      
      if (RtcTick) {
         // Do nothing
         RtcTick = 0;
      }
      
      if (UART1RxAvailable()) {
         const uint8_t ch = UART1RxByte();
         
         printf("UART1: %02x\n", ch);
         switch (ch) {
         case 'r':
         case 'R':
            setRect(0, 0, MAXX - 1, MAXY - 1, SSD1351_WHITE);
            updscreen(0, MAXY - 1);
            break;
         case 'q':
         case 'Q':
            drawVline(MAXX / 4,       0, MAXY - 1, SSD1351_WHITE);
            drawVline(MAXX / 2,       0, MAXY - 1, SSD1351_WHITE);
            drawVline((MAXX * 3) / 4, 0, MAXY - 1, SSD1351_WHITE);
            drawHline(0, MAXX - 1, MAXY / 4, SSD1351_WHITE);
            drawHline(0, MAXX - 1, MAXY / 2, SSD1351_WHITE);
            drawHline(0, MAXX - 1, (MAXY * 3) / 4, SSD1351_WHITE);
            updscreen(0, MAXY - 1);
            break;
         case '/':
            printf("analogRead = %d, %d\n", analogRead(1), analogRead(8));
            break;
         case 'z':
         case 'Z':
            memset(Frame, 0, sizeof (Frame));
            updscreen(0, MAXY - 1);
            break;
         }
      }
   }
}

