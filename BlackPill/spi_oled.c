/* spi_oled --- SPI OLED SSD1351 on Black Pill STM32 board  2023-01-27 */

#include <stm32f4xx.h>

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

// Size of 128x128 OLED screen
#define MAXX 128
#define MAXY 128

#include "image.h"
#include "petrol.h"
#include "P1030550_tiny.h"

#define DIGIT_WIDTH  (21)
#define DIGIT_HEIGHT (32)
#define DIGIT_STRIDE (210)

// Co-ord of centre of screen
#define CENX (MAXX / 2)
#define CENY (MAXY / 2)

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

// What mode should the display operate in?
enum MODE {
   MANUAL_MODE,
   AUTO_HMS_MODE,
   AUTO_HEX_MODE
};

// What state is the command parser operating in?
enum STATE {
   NOT_SETTING_TIME,
   SETTING_TIME_1,
   SETTING_TIME_2,
   SETTING_TIME_3,
   SETTING_TIME_4,
   SETTING_TIME_5,
   SETTING_TIME_6
};

// UART buffers
struct UART_BUFFER U1Buf;

// The colour frame buffer, 32k bytes
uint16_t Frame[MAXY][MAXX];

volatile uint32_t Milliseconds = 0;
volatile uint8_t Tick = 0;
volatile uint8_t RtcTick = 0;
volatile uint8_t Hour = 0;
volatile uint8_t Minute = 0;
volatile uint8_t Second = 0;


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
   
   if (Second >= 59) {
      if (Minute >= 59) {
         if (Hour >= 23)
            Hour = 0;
         else
            Hour++;
            
         Minute = 0;
      }
      else
         Minute++;
      
      Second = 0;
   }
   else
      Second++;
   
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


/* sendImg --- send an RGB565 image from a pixel array to the display */

static void __attribute__((optimize("O3"))) sendImg(const uint8_t x1, const uint8_t y1, const uint8_t wd, const uint8_t ht, const uint16_t *image)
{
    int x, y;
    volatile uint16_t __attribute__((unused)) junk;
    
    oledCmd2b(SSD1351_SETCOLUMN, x1, x1 + wd - 1);
    oledCmd2b(SSD1351_SETROW, y1, y1 + ht - 1);
    
    oledCmd(SSD1351_WRITERAM);
    
    SPI1->CR1 |= SPI_CR1_DFF;    // 16-bit mode for just a bit more speed
    spi_cs(0);
    
    for (y = 0; y < ht; y++)
        for (x = 0; x < wd; x++) {
            SPI1->DR = *image++;
   
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


/* setPixel --- set a single pixel */

void setPixel(const unsigned int x, const unsigned int y, const uint16_t c)
{
    if ((x < MAXX) && (y < MAXY))
        Frame[y][x] = c;
    else
    {
//      Serial.print("setPixel(");
//      Serial.print(x);
//      Serial.print(",");
//      Serial.print(y);
//      Serial.println(")");
    }
}


/* setVline --- draw vertical line */

void setVline(const unsigned int x, const unsigned int y1, const unsigned int y2, const uint16_t c)
{
    unsigned int y;

    for (y = y1; y <= y2; y++)
        Frame[y][x] = c;
}


/* setHline --- set pixels in a horizontal line */

void setHline(const unsigned int x1, const unsigned int x2, const unsigned int y, const uint16_t c)
{
    unsigned int x;

    for (x = x1; x <= x2; x++)
        Frame[y][x] = c;
}


/* setRect --- set pixels in a (non-filled) rectangle */

void setRect(const int x1, const int y1, const int x2, const int y2, const uint16_t c)
{
    setHline(x1, x2, y1, c);
    setVline(x2, y1, y2, c);
    setHline(x1, x2, y2, c);
    setVline(x1, y1, y2, c);
}


/* fillRect --- set pixels in a filled rectangle */

void fillRect(const int x1, const int y1, const int x2, const int y2, const uint16_t ec, const uint16_t fc)
{
    int y;

    for (y = y1; y <= y2; y++)
        setHline(x1, x2, y, fc);

    setHline(x1, x2, y1, ec);
    setVline(x2, y1, y2, ec);
    setHline(x1, x2, y2, ec);
    setVline(x1, y1, y2, ec);
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


#define  WD  (15)    // Width of digit (X-coord of rightmost pixel of segments 'b' and 'c')
#define  GY  (13)    // Y-coord of 'g' segment of Panaplex (slightly above half-way)

void drawLed(const int x0, int x, int y, const uint16_t c)
{
   x *= 4;
   y *= 4;
   
   x += x0;
   y += 3;
   
   setHline(x, x + 2, y + 0, c);
   setHline(x, x + 2, y + 1, c);
   setHline(x, x + 2, y + 2, c);
}


void drawSegA(const int x, const int style, const uint16_t c)
{
   switch (style) {
   case PANAPLEX_STYLE:
      setHline(x, x + WD, 0, c);
      setHline(x, x + WD, 1, c);
      break;
   case LED_DOT_STYLE:
      drawLed(x, 1, 0, c);
      drawLed(x, 2, 0, c);
      break;
   case LED_BAR_STYLE:
      setHline(x + 3, x + WD - 3, 0, c);
      setHline(x + 3, x + WD - 3, 1, c);
      setHline(x + 3, x + WD - 3, 2, c);
      break;
   case VFD_STYLE:
      setHline(x + 1, x + WD - 1, 0, c);
      setHline(x + 2, x + WD - 2, 1, c);
      setHline(x + 3, x + WD - 3, 2, c);
      break;
   }
}


void drawSegB(const int x, const int style, const uint16_t c)
{
   switch (style) {
   case PANAPLEX_STYLE:
      setVline(x + WD,     0, GY, c);
      setVline(x + WD - 1, 0, GY, c);
      break;
   case LED_DOT_STYLE:
      drawLed(x, 3, 1, c);
      drawLed(x, 3, 2, c);
      break;
   case LED_BAR_STYLE:
      setVline(x + WD,     3, 14, c);
      setVline(x + WD - 1, 3, 14, c);
      setVline(x + WD - 2, 3, 14, c);
      break;
   case VFD_STYLE:
      setVline(x + WD,     1, 13, c);
      setVline(x + WD - 1, 2, 14, c);
      setVline(x + WD - 2, 3, 13, c);
      break;
   }
}


void drawSegC(const int x, const int style, const uint16_t c)
{
   switch (style) {
   case PANAPLEX_STYLE:
      setVline(x + WD,     GY, 31, c);
      setVline(x + WD - 1, GY, 31, c);
      break;
   case LED_DOT_STYLE:
      drawLed(x, 3, 4, c);
      drawLed(x, 3, 5, c);
      break;
   case LED_BAR_STYLE:
      setVline(x + WD,     18, 28, c);
      setVline(x + WD - 1, 18, 28, c);
      setVline(x + WD - 2, 18, 28, c);
      break;
   case VFD_STYLE:
      setVline(x + WD,     19, 30, c);
      setVline(x + WD - 1, 18, 29, c);
      setVline(x + WD - 2, 19, 28, c);
      break;
   }
}


void drawSegD(const int x, const int style, const uint16_t c)
{
   switch (style) {
   case PANAPLEX_STYLE:
      setHline(x, x + WD, 31, c);
      setHline(x, x + WD, 30, c);
      break;
   case LED_DOT_STYLE:
      drawLed(x, 1, 6, c);
      drawLed(x, 2, 6, c);
      break;
   case LED_BAR_STYLE:
      setHline(x + 3, x + WD - 3, 31, c);
      setHline(x + 3, x + WD - 3, 30, c);
      setHline(x + 3, x + WD - 3, 29, c);
      break;
   case VFD_STYLE:
      setHline(x + 1, x + WD - 1, 31, c);
      setHline(x + 2, x + WD - 2, 30, c);
      setHline(x + 3, x + WD - 3, 29, c);
      break;
   }
}


void drawSegE(const int x, const int style, const uint16_t c)
{
   switch (style) {
   case PANAPLEX_STYLE:
      setVline(x + 0, GY, 31, c);
      setVline(x + 1, GY, 31, c);
      break;
   case LED_DOT_STYLE:
      drawLed(x, 0, 4, c);
      drawLed(x, 0, 5, c);
      break;
   case LED_BAR_STYLE:
      setVline(x + 0, 18, 28, c);
      setVline(x + 1, 18, 28, c);
      setVline(x + 2, 18, 28, c);
      break;
   case VFD_STYLE:
      setVline(x + 0, 17, 30, c);
      setVline(x + 1, 18, 29, c);
      setVline(x + 2, 19, 28, c);
      break;
   }
}


void drawSegF(const int x, const int style, const uint16_t c)
{
   switch (style) {
   case PANAPLEX_STYLE:
      setVline(x + 0, 0, GY, c);
      setVline(x + 1, 0, GY, c);
      break;
   case LED_DOT_STYLE:
      drawLed(x, 0, 1, c);
      drawLed(x, 0, 2, c);
      break;
   case LED_BAR_STYLE:
      setVline(x + 0, 3, 14, c);
      setVline(x + 1, 3, 14, c);
      setVline(x + 2, 3, 14, c);
      break;
   case VFD_STYLE:
      setVline(x + 0, 1, 15, c);
      setVline(x + 1, 2, 14, c);
      setVline(x + 2, 3, 13, c);
      break;
   }
}


void drawSegG(const int x, const int style, const uint16_t c)
{
   switch (style) {
   case PANAPLEX_STYLE:
      setHline(x, x + WD, GY, c);
      setHline(x, x + WD, GY + 1, c);
      break;
   case LED_DOT_STYLE:
      drawLed(x, 1, 3, c);
      drawLed(x, 2, 3, c);
      break;
   case LED_BAR_STYLE:
      setHline(x + 3, x + WD - 3, 15, c);
      setHline(x + 3, x + WD - 3, 16, c);
      setHline(x + 3, x + WD - 3, 17, c);
      break;
   case VFD_STYLE:
      setHline(x + 2, x + WD - 2, 15, c);
      setHline(x + 1, x + WD - 1, 16, c);
      setHline(x + 2, x + WD - 2, 17, c);
      break;
   }
}


void drawSegH(const int x, const int style, const uint16_t c)
{
   switch (style) {
   case PANAPLEX_STYLE:
      setHline(x + WD, x + WD + 3, GY, c);
      setHline(x + WD, x + WD + 3, GY + 1, c);
      break;
   case LED_DOT_STYLE:
      drawLed(x, 4, 3, c);
      break;
   case LED_BAR_STYLE:
      setHline(x + WD + 1, x + WD + 3, 15, c);
      setHline(x + WD + 1, x + WD + 3, 16, c);
      setHline(x + WD + 1, x + WD + 3, 17, c);
      break;
   case VFD_STYLE:
      setHline(x + WD,     x + WD + 3, 15, c);
      setHline(x + WD - 1, x + WD + 3, 16, c);
      setHline(x + WD,     x + WD + 3, 17, c);
      break;
   }
}


void drawSegI(const int x, const int style, const uint16_t c)
{
   switch (style) {
   case LED_DOT_STYLE:
      drawLed(x, 0, 0, c);
      break;
   }
}


void drawSegJ(const int x, const int style, const uint16_t c)
{
   switch (style) {
   case LED_DOT_STYLE:
      drawLed(x, 3, 0, c);
      break;
   }
}


void drawSegK(const int x, const int style, const uint16_t c)
{
   switch (style) {
   case LED_DOT_STYLE:
      drawLed(x, 3, 3, c);
      break;
   }
}


void drawSegL(const int x, const int style, const uint16_t c)
{
   switch (style) {
   case LED_DOT_STYLE:
      drawLed(x, 3, 6, c);
      break;
   }
}


void drawSegM(const int x, const int style, const uint16_t c)
{
   switch (style) {
   case LED_DOT_STYLE:
      drawLed(x, 0, 6, c);
      break;
   }
}


void drawSegN(const int x, const int style, const uint16_t c)
{
   switch (style) {
   case LED_DOT_STYLE:
      drawLed(x, 0, 3, c);
      break;
   }
}


void drawSegDP(const int x, const int style, const uint16_t c)
{
   switch (style) {
   case PANAPLEX_STYLE:
      setHline(x + WD + 2, x + WD + 4, 29, c);
      setHline(x + WD + 2, x + WD + 4, 30, c);
      setHline(x + WD + 2, x + WD + 4, 31, c);
      break;
   case LED_DOT_STYLE:
      drawLed(x, 4, 6, c);
      break;
   case LED_BAR_STYLE:
   case PETROL_STATION_STYLE:
   case VFD_STYLE:
      setHline(x + WD + 2, x + WD + 4, 29, c);
      setHline(x + WD + 2, x + WD + 4, 30, c);
      setHline(x + WD + 2, x + WD + 4, 31, c);
      break;
   }
}


void drawSegCN(const int x, const int style, const uint16_t c)
{
   switch (style) {
   case PANAPLEX_STYLE:
      setHline(x + WD + 2, x + WD + 3,  9, c);
      setHline(x + WD + 2, x + WD + 3, 10, c);
      setHline(x + WD + 2, x + WD + 3, 17, c);
      setHline(x + WD + 2, x + WD + 3, 18, c);
      break;
   case LED_DOT_STYLE:
      drawLed(x, 4, 2, c);
      drawLed(x, 4, 4, c);
      break;
   case LED_BAR_STYLE:
   case PETROL_STATION_STYLE:
   case VFD_STYLE:
      setHline(x + WD + 2, x + WD + 4, 11, c);
      setHline(x + WD + 2, x + WD + 4, 12, c);
      setHline(x + WD + 2, x + WD + 4, 13, c);
      setHline(x + WD + 2, x + WD + 4, 19, c);
      setHline(x + WD + 2, x + WD + 4, 20, c);
      setHline(x + WD + 2, x + WD + 4, 21, c);
      break;
   }
}


/* renderHexDigit --- draw a single digit into the frame buffer in a given style */

void renderHexDigit(const int x, const int digit, const int style, const uint16_t c)
{
   if (style == PETROL_STATION_STYLE) {
      renderBitmap(x, 0, DIGIT_WIDTH, DIGIT_HEIGHT, &PetrolDigits[0][digit * DIGIT_WIDTH], DIGIT_STRIDE, c, SSD1351_BLACK);
   }
   else {
      switch (digit) {
      case 0:
         drawSegA(x, style, c);
         drawSegB(x, style, c);
         drawSegC(x, style, c);
         drawSegD(x, style, c);
         drawSegE(x, style, c);
         drawSegF(x, style, c);
         drawSegK(x, style, c);
         drawSegN(x, style, c);
         break;
      case 1:
         drawSegB(x, style, c);
         drawSegC(x, style, c);
         drawSegJ(x, style, c);
         drawSegK(x, style, c);
         drawSegL(x, style, c);
         break;
      case 2:
         drawSegA(x, style, c);
         drawSegB(x, style, c);
         drawSegD(x, style, c);
         drawSegE(x, style, c);
         drawSegG(x, style, c);
         drawSegI(x, style, c);
         drawSegL(x, style, c);
         drawSegM(x, style, c);
         break;
      case 3:
         drawSegA(x, style, c);
         drawSegB(x, style, c);
         drawSegC(x, style, c);
         drawSegD(x, style, c);
         drawSegG(x, style, c);
         drawSegI(x, style, c);
         drawSegM(x, style, c);
         break;
      case 4:
         drawSegB(x, style, c);
         drawSegC(x, style, c);
         drawSegF(x, style, c);
         drawSegG(x, style, c);
         drawSegH(x, style, c);  // Special segment just for 4
         drawSegI(x, style, c);
         drawSegJ(x, style, c);
         drawSegK(x, style, c);
         drawSegL(x, style, c);
         break;
      case 5:
         drawSegA(x, style, c);
         drawSegC(x, style, c);
         drawSegD(x, style, c);
         drawSegF(x, style, c);
         drawSegG(x, style, c);
         drawSegI(x, style, c);
         drawSegJ(x, style, c);
         drawSegM(x, style, c);
         break;
      case 6:
         drawSegA(x, style, c);
         drawSegC(x, style, c);
         drawSegD(x, style, c);
         drawSegE(x, style, c);
         drawSegF(x, style, c);
         drawSegG(x, style, c);
         drawSegJ(x, style, c);
         drawSegN(x, style, c);
         break;
      case 7:
         drawSegA(x, style, c);
         drawSegB(x, style, c);
         drawSegC(x, style, c);
         drawSegF(x, style, c);  // Hooked 7
         drawSegI(x, style, c);
         drawSegJ(x, style, c);
         drawSegK(x, style, c);
         drawSegL(x, style, c);
         break;
      case 8:
         drawSegA(x, style, c);
         drawSegB(x, style, c);
         drawSegC(x, style, c);
         drawSegD(x, style, c);
         drawSegE(x, style, c);
         drawSegF(x, style, c);
         drawSegG(x, style, c);
         break;
      case 9:
         drawSegA(x, style, c);
         drawSegB(x, style, c);
         drawSegC(x, style, c);
         drawSegD(x, style, c);
         drawSegF(x, style, c);
         drawSegG(x, style, c);
         drawSegK(x, style, c);
         drawSegM(x, style, c);
         break;
      case 0xA:
         drawSegA(x, style, c);
         drawSegB(x, style, c);
         drawSegC(x, style, c);
         drawSegE(x, style, c);
         drawSegF(x, style, c);
         drawSegG(x, style, c);
         drawSegK(x, style, c);
         drawSegL(x, style, c);
         drawSegM(x, style, c);
         drawSegN(x, style, c);
         break;
      case 0xB:
         drawSegC(x, style, c);     // Lowercase 'b'
         drawSegD(x, style, c);
         drawSegE(x, style, c);
         drawSegF(x, style, c);
         drawSegG(x, style, c);
         if (style == LED_DOT_STYLE) {
            drawSegA(x, style, c);  // Uppercase 'B'
            drawSegB(x, style, c);
            drawSegI(x, style, c);
            drawSegM(x, style, c);
            drawSegN(x, style, c);
         }
         break;
      case 0xC:
         drawSegA(x, style, c);
         drawSegD(x, style, c);
         drawSegE(x, style, c);
         drawSegF(x, style, c);
         drawSegJ(x, style, c);
         drawSegL(x, style, c);
         drawSegN(x, style, c);
         break;
      case 0xD:
         if (style == LED_DOT_STYLE) {
            drawSegA(x, style, c);  // Uppercase 'D'
            drawSegB(x, style, c);
            drawSegC(x, style, c);
            drawSegD(x, style, c);
            drawSegE(x, style, c);
            drawSegF(x, style, c);
            drawSegI(x, style, c);
            drawSegK(x, style, c);
            drawSegM(x, style, c);
            drawSegN(x, style, c);
         }
         else {
            drawSegB(x, style, c);  // Lowercase 'd'
            drawSegC(x, style, c);
            drawSegD(x, style, c);
            drawSegE(x, style, c);
            drawSegG(x, style, c);
         }
         break;
      case 0xE:
         drawSegA(x, style, c);
         drawSegD(x, style, c);
         drawSegE(x, style, c);
         drawSegF(x, style, c);
         drawSegG(x, style, c);
         drawSegI(x, style, c);
         drawSegJ(x, style, c);
         drawSegL(x, style, c);
         drawSegM(x, style, c);
         drawSegN(x, style, c);
         break;
      case 0xF:
         drawSegA(x, style, c);
         drawSegE(x, style, c);
         drawSegF(x, style, c);
         drawSegG(x, style, c);
         drawSegI(x, style, c);
         drawSegJ(x, style, c);
         drawSegM(x, style, c);
         drawSegN(x, style, c);
         break;
      }
   }
}


/* renderClockDisplay --- draw the six clock digits into the frame buffer */

void renderClockDisplay(const int width, const int style, const uint16_t c)
{
   renderHexDigit(0 * width, Hour / 10, style, c);
   renderHexDigit(1 * width, Hour % 10, style, c);
   
   renderHexDigit(2 * width, Minute / 10, style, c);
   renderHexDigit(3 * width, Minute % 10, style, c);
   
   renderHexDigit(4 * width, Second / 10, style, c);
   renderHexDigit(5 * width, Second % 10, style, c);
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
   uint32_t colon;   // Time in milliSeconds at which to draw the colon separators
   uint8_t flag = 0;
   const int width = WD + 6;
   int digit = 0;
   int x = digit * width;
   int style = VFD_STYLE;           // Initially draw digits in Vacuum Fluorescent Display style
   uint16_t colour = VFD_COLOUR;    // Initially draw in cyan
   int displayMode = MANUAL_MODE;   // Initially operate the display manually
   int state = NOT_SETTING_TIME;
   int hour = 0, minute = 0, second = 0;
   
   initMCU();
   initGPIOs();
   initUARTs();
   initSPI();
   initTimers();
   initMillisecondTimer();
   
   __enable_irq();   // Enable all interrupts
   
   OLED_begin(MAXX, MAXY);
   
   greyFrame();
    
   updscreen(0, MAXY - 1);
   
   printf("\nHello from the STM%dF%d\n", 32, 411);
   
   end = millis() + 500u;
   colon = 0xffffffff;
   
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
         
         if ((displayMode == AUTO_HMS_MODE) && (millis() >= colon)) {
            drawSegCN(1 * width, style, colour);
            drawSegCN(3 * width, style, colour);
            
            updscreen(0, 31);
            
            colon += 600u;
         }
         
         Tick = 0;
      }
      
      if (RtcTick) {
         printf("RTC: %02d:%02d:%02d\n", Hour, Minute, Second);
         
         if (displayMode == AUTO_HMS_MODE) {
            memset(Frame, 0, sizeof (Frame) / 4);
            
            renderClockDisplay(width, style, colour);
            
            updscreen(0, 31);
            
            colon = millis() + 500u;
         }
         
         RtcTick = 0;
      }
      
      if (UART1RxAvailable()) {
         const uint8_t ch = UART1RxByte();
         
         printf("UART1: %02x\n", ch);
         switch (state) {
         case SETTING_TIME_1:
            if (isdigit(ch)) {
               hour = (ch - '0') * 10;
               state = SETTING_TIME_2;
            }
            else
               state = NOT_SETTING_TIME;
            break;
         case SETTING_TIME_2:
            if (isdigit(ch)) {
               hour += ch - '0';
               state = SETTING_TIME_3;
            }
            else
               state = NOT_SETTING_TIME;
            break;
         case SETTING_TIME_3:
            if (isdigit(ch)) {
               minute = (ch - '0') * 10;
               state = SETTING_TIME_4;
            }
            else
               state = NOT_SETTING_TIME;
            break;
         case SETTING_TIME_4:
            if (isdigit(ch)) {
               minute += ch - '0';
               state = SETTING_TIME_5;
            }
            else
               state = NOT_SETTING_TIME;
            break;
         case SETTING_TIME_5:
            if (isdigit(ch)) {
               second = (ch - '0') * 10;
               state = SETTING_TIME_6;
            }
            else
               state = NOT_SETTING_TIME;
            break;
         case SETTING_TIME_6:
            if (isdigit(ch)) {
               second += ch - '0';
               state = NOT_SETTING_TIME;
               printf("NEW: %02d:%02d:%02d\n", hour, minute, second);
               Hour = hour;
               Minute = minute;
               Second = second;
            }
            else
               state = NOT_SETTING_TIME;
            break;
         case NOT_SETTING_TIME:
            switch (ch) {
            case 'r':
            case 'R':
               setRect(0, 0, MAXX - 1, MAXY - 1, SSD1351_WHITE);
               updscreen(0, MAXY - 1);
               break;
            case 'q':
            case 'Q':
               setVline(MAXX / 4,       0, MAXY - 1, SSD1351_WHITE);
               setVline(MAXX / 2,       0, MAXY - 1, SSD1351_WHITE);
               setVline((MAXX * 3) / 4, 0, MAXY - 1, SSD1351_WHITE);
               setHline(0, MAXX - 1, MAXY / 4, SSD1351_WHITE);
               setHline(0, MAXX - 1, MAXY / 2, SSD1351_WHITE);
               setHline(0, MAXX - 1, (MAXY * 3) / 4, SSD1351_WHITE);
               updscreen(0, MAXY - 1);
               break;
            case 'g':
               digit = 0;
               x = digit * width;
               break;
            case 'h':
               digit = 1;
               x = digit * width;
               break;
            case 'i':
               digit = 2;
               x = digit * width;
               break;
            case 'j':
               digit = 3;
               x = digit * width;
               break;
            case 'k':
               digit = 4;
               x = digit * width;
               break;
            case 'l':
               digit = 5;
               x = digit * width;
               break;
            case '0':
               renderHexDigit(x, 0, style, colour);
               updscreen(0, 31);
               break;
            case '1':
               renderHexDigit(x, 1, style, colour);
               updscreen(0, 31);
               break;
            case '2':
               renderHexDigit(x, 2, style, colour);
               updscreen(0, 31);
               break;
            case '3':
               renderHexDigit(x, 3, style, colour);
               updscreen(0, 31);
               break;
            case '4':
               renderHexDigit(x, 4, style, colour);
               updscreen(0, 31);
               break;
            case '5':
               renderHexDigit(x, 5, style, colour);
               updscreen(0, 31);
               break;
            case '6':
               renderHexDigit(x, 6, style, colour);
               updscreen(0, 31);
               break;
            case '7':
               renderHexDigit(x, 7, style, colour);
               updscreen(0, 31);
               break;
            case '8':
               renderHexDigit(x, 8, style, colour);
               updscreen(0, 31);
               break;
            case '9':
               renderHexDigit(x, 9, style, colour);
               updscreen(0, 31);
               break;
            case 'a':
            case 'A':
               renderHexDigit(x, 0xA, style, colour);
               updscreen(0, 31);
               break;
            case 'b':
            case 'B':
               renderHexDigit(x, 0xB, style, colour);
               updscreen(0, 31);
               break;
            case 'c':
            case 'C':
               renderHexDigit(x, 0xC, style, colour);
               updscreen(0, 31);
               break;
            case 'd':
            case 'D':
               renderHexDigit(x, 0xD, style, colour);
               updscreen(0, 31);
               break;
            case 'e':
            case 'E':
               renderHexDigit(x, 0xE, style, colour);
               updscreen(0, 31);
               break;
            case 'f':
            case 'F':
               renderHexDigit(x, 0xF, style, colour);
               updscreen(0, 31);
               break;
            case 'o':
            case 'O':
               renderBitmap(0, 32, 128, 32, &OLEDImage[0][0], 128, SSD1351_BLUE, SSD1351_GREY25);
               renderBitmap(0, 64, 128, 32, &OLEDImage[0][0], 128, SSD1351_YELLOW, SSD1351_GREY50);
               updscreen(32, 127);
               break;
            case '\r':
               renderBitmap(0, 64, 128, DIGIT_HEIGHT, &PetrolDigits[0][0], DIGIT_STRIDE, SSD1351_GREEN, SSD1351_BLACK);
               updscreen(64, 95);
               break;
            case ']':
               sendImg(32, 64, 64, 64, &Copen64[0][0]);
               break;
            case '.':
               drawSegDP(x, style, colour);
               updscreen(0, 31);
               break;
            case ':':
               drawSegCN(x, style, colour);
               updscreen(0, 31);
               break;
            case 's':
               state = SETTING_TIME_1;
               printf("OLD: %02d:%02d:%02d\n", Hour, Minute, Second);
               break;
            case 't':
               memset(Frame, 0, sizeof (Frame) / 4);
               
               renderClockDisplay(width, style, colour);
               drawSegCN(1 * width, style, colour);
               drawSegCN(3 * width, style, colour);
               
               updscreen(0, 31);
               
               colon = millis() + 1100u;
               break;
            case 'm':
            case 'M':
               displayMode = MANUAL_MODE;
               break;
            case 'n':
            case 'N':
               style = PETROL_STATION_STYLE;
               colour = PETROL_STATION_COLOUR;
               break;
            case 'u':
            case 'U':
               displayMode = AUTO_HMS_MODE;
               break;
            case 'v':
            case 'V':
               style = VFD_STYLE;
               colour = VFD_COLOUR;
               break;
            case 'w':
            case 'W':
               style = LED_DOT_STYLE;
               colour = LED_COLOUR;
               break;
            case 'x':
            case 'X':
               style = PANAPLEX_STYLE;
               colour = PANAPLEX_COLOUR;
               break;
            case 'y':
            case 'Y':
               style = LED_BAR_STYLE;
               colour = LED_COLOUR;
               break;
            case 'z':
            case 'Z':
               memset(Frame, 0, sizeof (Frame));
               updscreen(0, MAXY - 1);
               break;
            }
            break;
         }
      }
   }
}

