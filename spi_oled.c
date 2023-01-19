/* spi_oled --- SPI OLED SSD1351 on Blue Pill STM32 board   2023-01-16 */

#include <stm32f1xx.h>

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

// Size of 128x128 OLED screen
#define MAXX 128
#define MAXY 32   // Just one quarter of the screen for now, same as the I2C OLED
#define MAXROWS 4

//#include "image.h"

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

// The frame buffer, 512 bytes
uint8_t Frame[MAXROWS][MAXX];

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
      GPIOA->BSRR = GPIO_BSRR_BS12; // GPIO pin PA12 HIGH
   else
      GPIOA->BSRR = GPIO_BSRR_BR12; // GPIO pin PA12 LOW
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

static void updscreen(void)
{
    int r, c;
    int x1, x2, y1, y2;
    uint16_t pixel;
    volatile uint16_t __attribute__((unused)) junk;
    
    x1 = 0;
    x2 = MAXX - 1;
    y1 = 0;
    y2 = MAXY - 1;
    
    oledCmd2b(SSD1351_SETCOLUMN, x1, x2);
    oledCmd2b(SSD1351_SETROW, y1, y2);
    
    oledCmd(SSD1351_WRITERAM);
    
    SPI1->CR1 |= SPI_CR1_DFF;    // 16-bit mode for just a bit more speed
    spi_cs(0);
    
    for (r = 0; r < MAXY; r++)
        for (c = 0; c < MAXX; c++) {
            if (Frame[r / 8][c] & (1 << (r % 8))) {
                pixel = 0xffff;
            }
            else {
                pixel = 0x0000;
            }
            
            SPI1->DR = pixel;
   
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
    
    oledCmd(SSD1351_DISPLAYON); // Turn on OLED panel
}


/* greyFrame --- clear entire frame to checkerboard pattern */

void greyFrame(void)
{
    int r, c;

    for (r = 0; r < MAXROWS; r++)
    {
        for (c = 0; c < MAXX; c += 2)
        {
            Frame[r][c] = 0xaa;
            Frame[r][c + 1] = 0x55;
        }
    }
}


/* setPixel --- set a single pixel */

void setPixel(const unsigned int x, const unsigned int y)
{
    if ((x < MAXX) && (y < MAXY))
        Frame[y / 8][x] |= 1 << (y & 7);
    else
    {
//      Serial.print("setPixel(");
//      Serial.print(x);
//      Serial.print(",");
//      Serial.print(y);
//      Serial.println(")");
    }
}


/* clrPixel --- clear a single pixel */

void clrPixel(const unsigned int x, const unsigned int y)
{
    if ((x < MAXX) && (y < MAXY))
        Frame[y / 8][x] &= ~(1 << (y & 7));
    else
    {
//      Serial.print("clrPixel(");
//      Serial.print(x);
//      Serial.print(",");
//      Serial.print(y);
//      Serial.println(")");
    }
}


/* setVline --- draw vertical line */

void setVline(const unsigned int x, const unsigned int y1, const unsigned int y2)
{
    unsigned int y;

    for (y = y1; y <= y2; y++)
        setPixel(x, y);
}


/* clrVline --- draw vertical line */

void clrVline(const unsigned int x, const unsigned int y1, const unsigned int y2)
{
    unsigned int y;

    for (y = y1; y <= y2; y++)
        clrPixel(x, y);
}


/* setHline --- set pixels in a horizontal line */

void setHline(const unsigned int x1, const unsigned int x2, const unsigned int y)
{
    unsigned int x;
    const unsigned int row = y / 8;
    const uint8_t b = 1 << (y  & 7);

    for (x = x1; x <= x2; x++)
        Frame[row][x] |= b;
}


/* clrHline --- clear pixels in a horizontal line */

void clrHline(const unsigned int x1, const unsigned int x2, const unsigned int y)
{
    unsigned int x;
    const unsigned int row = y / 8;
    const uint8_t b = ~(1 << (y  & 7));

    for (x = x1; x <= x2; x++)
      Frame[row][x] &= b;
}


/* setRect --- set pixels in a (non-filled) rectangle */

void setRect(const int x1, const int y1, const int x2, const int y2)
{
    setHline(x1, x2, y1);
    setVline(x2, y1, y2);
    setHline(x1, x2, y2);
    setVline(x1, y1, y2);
}


/* fillRect --- set pixels in a filled rectangle */

void fillRect(const int x1, const int y1, const int x2, const int y2, const int ec, const int fc)
{
    int y;

    for (y = y1; y <= y2; y++)
        if (fc == 0)
            clrHline(x1, x2, y);
        else if (fc == 1)
            setHline(x1, x2, y);

    if (ec == 1)
    {
        setHline(x1, x2, y1);
        setVline(x2, y1, y2);
        setHline(x1, x2, y2);
        setVline(x1, y1, y2);
    }
    else if (ec == 0)
    {
        clrHline(x1, x2, y1);
        clrVline(x2, y1, y2);
        clrHline(x1, x2, y2);
        clrVline(x1, y1, y2);
    }
}

#define  WD  (15)    // Width of digit (X-coord of rightmost pixel of segments 'b' and 'c')
#define  GY  (13)    // Y-coord of 'g' segment of Panaplex (slightly above half-way)

void drawLed(const int x0, int x, int y)
{
   x *= 4;
   y *= 4;
   
   x += x0;
   y += 3;
   
   setHline(x, x + 2, y + 0);
   setHline(x, x + 2, y + 1);
   setHline(x, x + 2, y + 2);
}


void drawSegA(const int x, const int style)
{
   switch (style) {
   case PANAPLEX_STYLE:
      setHline(x, x + WD, 0);
      setHline(x, x + WD, 1);
      break;
   case LED_DOT_STYLE:
      drawLed(x, 1, 0);
      drawLed(x, 2, 0);
      break;
   case LED_BAR_STYLE:
      setHline(x + 3, x + WD - 3, 0);
      setHline(x + 3, x + WD - 3, 1);
      setHline(x + 3, x + WD - 3, 2);
      break;
   case VFD_STYLE:
      setHline(x + 1, x + WD - 1, 0);
      setHline(x + 2, x + WD - 2, 1);
      setHline(x + 3, x + WD - 3, 2);
      break;
   }
}


void drawSegB(const int x, const int style)
{
   switch (style) {
   case PANAPLEX_STYLE:
      setVline(x + WD,     0, GY);
      setVline(x + WD - 1, 0, GY);
      break;
   case LED_DOT_STYLE:
      drawLed(x, 3, 1);
      drawLed(x, 3, 2);
      break;
   case LED_BAR_STYLE:
      setVline(x + WD,     3, 14);
      setVline(x + WD - 1, 3, 14);
      setVline(x + WD - 2, 3, 14);
      break;
   case VFD_STYLE:
      setVline(x + WD,     1, 13);
      setVline(x + WD - 1, 2, 14);
      setVline(x + WD - 2, 3, 13);
      break;
   }
}


void drawSegC(const int x, const int style)
{
   switch (style) {
   case PANAPLEX_STYLE:
      setVline(x + WD,     GY, 31);
      setVline(x + WD - 1, GY, 31);
      break;
   case LED_DOT_STYLE:
      drawLed(x, 3, 4);
      drawLed(x, 3, 5);
      break;
   case LED_BAR_STYLE:
      setVline(x + WD,     18, 28);
      setVline(x + WD - 1, 18, 28);
      setVline(x + WD - 2, 18, 28);
      break;
   case VFD_STYLE:
      setVline(x + WD,     19, 30);
      setVline(x + WD - 1, 18, 29);
      setVline(x + WD - 2, 19, 28);
      break;
   }
}


void drawSegD(const int x, const int style)
{
   switch (style) {
   case PANAPLEX_STYLE:
      setHline(x, x + WD, 31);
      setHline(x, x + WD, 30);
      break;
   case LED_DOT_STYLE:
      drawLed(x, 1, 6);
      drawLed(x, 2, 6);
      break;
   case LED_BAR_STYLE:
      setHline(x + 3, x + WD - 3, 31);
      setHline(x + 3, x + WD - 3, 30);
      setHline(x + 3, x + WD - 3, 29);
      break;
   case VFD_STYLE:
      setHline(x + 1, x + WD - 1, 31);
      setHline(x + 2, x + WD - 2, 30);
      setHline(x + 3, x + WD - 3, 29);
      break;
   }
}


void drawSegE(const int x, const int style)
{
   switch (style) {
   case PANAPLEX_STYLE:
      setVline(x + 0, GY, 31);
      setVline(x + 1, GY, 31);
      break;
   case LED_DOT_STYLE:
      drawLed(x, 0, 4);
      drawLed(x, 0, 5);
      break;
   case LED_BAR_STYLE:
      setVline(x + 0, 18, 28);
      setVline(x + 1, 18, 28);
      setVline(x + 2, 18, 28);
      break;
   case VFD_STYLE:
      setVline(x + 0, 17, 30);
      setVline(x + 1, 18, 29);
      setVline(x + 2, 19, 28);
      break;
   }
}


void drawSegF(const int x, const int style)
{
   switch (style) {
   case PANAPLEX_STYLE:
      setVline(x + 0, 0, GY);
      setVline(x + 1, 0, GY);
      break;
   case LED_DOT_STYLE:
      drawLed(x, 0, 1);
      drawLed(x, 0, 2);
      break;
   case LED_BAR_STYLE:
      setVline(x + 0, 3, 14);
      setVline(x + 1, 3, 14);
      setVline(x + 2, 3, 14);
      break;
   case VFD_STYLE:
      setVline(x + 0, 1, 15);
      setVline(x + 1, 2, 14);
      setVline(x + 2, 3, 13);
      break;
   }
}


void drawSegG(const int x, const int style)
{
   switch (style) {
   case PANAPLEX_STYLE:
      setHline(x, x + WD, GY);
      setHline(x, x + WD, GY + 1);
      break;
   case LED_DOT_STYLE:
      drawLed(x, 1, 3);
      drawLed(x, 2, 3);
      break;
   case LED_BAR_STYLE:
      setHline(x + 3, x + WD - 3, 15);
      setHline(x + 3, x + WD - 3, 16);
      setHline(x + 3, x + WD - 3, 17);
      break;
   case VFD_STYLE:
      setHline(x + 2, x + WD - 2, 15);
      setHline(x + 1, x + WD - 1, 16);
      setHline(x + 2, x + WD - 2, 17);
      break;
   }
}


void drawSegH(const int x, const int style)
{
   switch (style) {
   case PANAPLEX_STYLE:
      setHline(x + WD, x + WD + 3, GY);
      setHline(x + WD, x + WD + 3, GY + 1);
      break;
   case LED_DOT_STYLE:
      drawLed(x, 4, 3);
      break;
   case LED_BAR_STYLE:
      setHline(x + WD + 1, x + WD + 3, 15);
      setHline(x + WD + 1, x + WD + 3, 16);
      setHline(x + WD + 1, x + WD + 3, 17);
      break;
   case VFD_STYLE:
      setHline(x + WD,     x + WD + 3, 15);
      setHline(x + WD - 1, x + WD + 3, 16);
      setHline(x + WD,     x + WD + 3, 17);
      break;
   }
}


void drawSegI(const int x, const int style)
{
   switch (style) {
   case LED_DOT_STYLE:
      drawLed(x, 0, 0);
      break;
   }
}


void drawSegJ(const int x, const int style)
{
   switch (style) {
   case LED_DOT_STYLE:
      drawLed(x, 3, 0);
      break;
   }
}


void drawSegK(const int x, const int style)
{
   switch (style) {
   case LED_DOT_STYLE:
      drawLed(x, 3, 3);
      break;
   }
}


void drawSegL(const int x, const int style)
{
   switch (style) {
   case LED_DOT_STYLE:
      drawLed(x, 3, 6);
      break;
   }
}


void drawSegM(const int x, const int style)
{
   switch (style) {
   case LED_DOT_STYLE:
      drawLed(x, 0, 6);
      break;
   }
}


void drawSegN(const int x, const int style)
{
   switch (style) {
   case LED_DOT_STYLE:
      drawLed(x, 0, 3);
      break;
   }
}


void drawSegDP(const int x, const int style)
{
   switch (style) {
   case PANAPLEX_STYLE:
      setHline(x + WD + 2, x + WD + 4, 29);
      setHline(x + WD + 2, x + WD + 4, 30);
      setHline(x + WD + 2, x + WD + 4, 31);
      break;
   case LED_DOT_STYLE:
      drawLed(x, 4, 6);
      break;
   case LED_BAR_STYLE:
   case VFD_STYLE:
      setHline(x + WD + 2, x + WD + 4, 29);
      setHline(x + WD + 2, x + WD + 4, 30);
      setHline(x + WD + 2, x + WD + 4, 31);
      break;
   }
}


void drawSegCN(const int x, const int style)
{
   switch (style) {
   case PANAPLEX_STYLE:
      setHline(x + WD + 2, x + WD + 3,  9);
      setHline(x + WD + 2, x + WD + 3, 10);
      setHline(x + WD + 2, x + WD + 3, 17);
      setHline(x + WD + 2, x + WD + 3, 18);
      break;
   case LED_DOT_STYLE:
      drawLed(x, 4, 2);
      drawLed(x, 4, 4);
      break;
   case LED_BAR_STYLE:
   case VFD_STYLE:
      setHline(x + WD + 2, x + WD + 4, 11);
      setHline(x + WD + 2, x + WD + 4, 12);
      setHline(x + WD + 2, x + WD + 4, 13);
      setHline(x + WD + 2, x + WD + 4, 19);
      setHline(x + WD + 2, x + WD + 4, 20);
      setHline(x + WD + 2, x + WD + 4, 21);
      break;
   }
}


/* renderHexDigit --- draw a single digit into the frame buffer in a given style */

void renderHexDigit(const int x, const int digit, const int style)
{
   switch (digit) {
   case 0:
      drawSegA(x, style);
      drawSegB(x, style);
      drawSegC(x, style);
      drawSegD(x, style);
      drawSegE(x, style);
      drawSegF(x, style);
      drawSegK(x, style);
      drawSegN(x, style);
      break;
   case 1:
      drawSegB(x, style);
      drawSegC(x, style);
      drawSegJ(x, style);
      drawSegK(x, style);
      drawSegL(x, style);
      break;
   case 2:
      drawSegA(x, style);
      drawSegB(x, style);
      drawSegD(x, style);
      drawSegE(x, style);
      drawSegG(x, style);
      drawSegI(x, style);
      drawSegL(x, style);
      drawSegM(x, style);
      break;
   case 3:
      drawSegA(x, style);
      drawSegB(x, style);
      drawSegC(x, style);
      drawSegD(x, style);
      drawSegG(x, style);
      drawSegI(x, style);
      drawSegM(x, style);
      break;
   case 4:
      drawSegB(x, style);
      drawSegC(x, style);
      drawSegF(x, style);
      drawSegG(x, style);
      drawSegH(x, style);  // Special segment just for 4
      drawSegI(x, style);
      drawSegJ(x, style);
      drawSegK(x, style);
      drawSegL(x, style);
      break;
   case 5:
      drawSegA(x, style);
      drawSegC(x, style);
      drawSegD(x, style);
      drawSegF(x, style);
      drawSegG(x, style);
      drawSegI(x, style);
      drawSegJ(x, style);
      drawSegM(x, style);
      break;
   case 6:
      drawSegA(x, style);
      drawSegC(x, style);
      drawSegD(x, style);
      drawSegE(x, style);
      drawSegF(x, style);
      drawSegG(x, style);
      drawSegJ(x, style);
      drawSegN(x, style);
      break;
   case 7:
      drawSegA(x, style);
      drawSegB(x, style);
      drawSegC(x, style);
      drawSegF(x, style);  // Hooked 7
      drawSegI(x, style);
      drawSegJ(x, style);
      drawSegK(x, style);
      drawSegL(x, style);
      break;
   case 8:
      drawSegA(x, style);
      drawSegB(x, style);
      drawSegC(x, style);
      drawSegD(x, style);
      drawSegE(x, style);
      drawSegF(x, style);
      drawSegG(x, style);
      break;
   case 9:
      drawSegA(x, style);
      drawSegB(x, style);
      drawSegC(x, style);
      drawSegD(x, style);
      drawSegF(x, style);
      drawSegG(x, style);
      drawSegK(x, style);
      drawSegM(x, style);
      break;
   case 0xA:
      drawSegA(x, style);
      drawSegB(x, style);
      drawSegC(x, style);
      drawSegE(x, style);
      drawSegF(x, style);
      drawSegG(x, style);
      drawSegK(x, style);
      drawSegL(x, style);
      drawSegM(x, style);
      drawSegN(x, style);
      break;
   case 0xB:
      drawSegC(x, style);     // Lowercase 'b'
      drawSegD(x, style);
      drawSegE(x, style);
      drawSegF(x, style);
      drawSegG(x, style);
      if (style == LED_DOT_STYLE) {
         drawSegA(x, style);  // Uppercase 'B'
         drawSegB(x, style);
         drawSegI(x, style);
         drawSegM(x, style);
         drawSegN(x, style);
      }
      break;
   case 0xC:
      drawSegA(x, style);
      drawSegD(x, style);
      drawSegE(x, style);
      drawSegF(x, style);
      drawSegJ(x, style);
      drawSegL(x, style);
      drawSegN(x, style);
      break;
   case 0xD:
      if (style == LED_DOT_STYLE) {
         drawSegA(x, style);  // Uppercase 'D'
         drawSegB(x, style);
         drawSegC(x, style);
         drawSegD(x, style);
         drawSegE(x, style);
         drawSegF(x, style);
         drawSegI(x, style);
         drawSegK(x, style);
         drawSegM(x, style);
         drawSegN(x, style);
      }
      else {
         drawSegB(x, style);  // Lowercase 'd'
         drawSegC(x, style);
         drawSegD(x, style);
         drawSegE(x, style);
         drawSegG(x, style);
      }
      break;
   case 0xE:
      drawSegA(x, style);
      drawSegD(x, style);
      drawSegE(x, style);
      drawSegF(x, style);
      drawSegG(x, style);
      drawSegI(x, style);
      drawSegJ(x, style);
      drawSegL(x, style);
      drawSegM(x, style);
      drawSegN(x, style);
      break;
   case 0xF:
      drawSegA(x, style);
      drawSegE(x, style);
      drawSegF(x, style);
      drawSegG(x, style);
      drawSegI(x, style);
      drawSegJ(x, style);
      drawSegM(x, style);
      drawSegN(x, style);
      break;
   }
}


/* renderClockDisplay --- draw the six clock digits into the frame buffer */

void renderClockDisplay(const int width, const int style)
{
   renderHexDigit(0 * width, Hour / 10, style);
   renderHexDigit(1 * width, Hour % 10, style);
   
   renderHexDigit(2 * width, Minute / 10, style);
   renderHexDigit(3 * width, Minute % 10, style);
   
   renderHexDigit(4 * width, Second / 10, style);
   renderHexDigit(5 * width, Second % 10, style);
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
   FLASH->ACR |= FLASH_ACR_LATENCY_2;  // Set Flash latency
   
   RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;   // Set APB1 bus to not exceed 36MHz
   
   RCC->CR |= RCC_CR_HSEON;   // Switch on High Speed External clock (8MHz on the Blue Pill)
   
   // Wait for HSE to start up
   while ((RCC->CR & RCC_CR_HSERDY) == 0)
      ;
   
   RCC->CFGR |= RCC_CFGR_PLLSRC;       // Select HSE as input to PLL
   RCC->CFGR |= RCC_CFGR_PLLMULL9;     // Select multiply-by-9 to go from 8MHz to 72MHz
   
   RCC->CR |= RCC_CR_PLLON;            // Switch on PLL
   
   // Wait for PLL to start up
   while ((RCC->CR & RCC_CR_PLLRDY) == 0)
      ;
   
   RCC->CFGR |= RCC_CFGR_SW_PLL;       // Select PLL as system clock (72MHz)
   
   // Wait for PLL to select
   while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
      ;
   
   RCC->CSR |= RCC_CSR_RMVF;
}


/* initGPIOs --- set up the GPIO pins */

static void initGPIOs(void)
{
   // Configure Reset and Clock Control
   RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;                    // Enable clock to GPIO B peripherals on APB2 bus
   RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;                    // Enable clock to GPIO C peripherals on APB2 bus
   
   // Configure PB12, the GPIO pin with the red LED
   GPIOB->CRH &= ~(GPIO_CRH_MODE12 | GPIO_CRH_CNF12);     // Set PB12 to push-pull output mode
   GPIOB->CRH |= (GPIO_CRH_MODE12_0 | GPIO_CRH_MODE12_1); // Configure PB12 as output, 50MHz
   
   // Configure PB13, the GPIO pin with the green LED
   GPIOB->CRH &= ~(GPIO_CRH_MODE13 | GPIO_CRH_CNF13);     // Set PB13 to push-pull output mode
   GPIOB->CRH |= (GPIO_CRH_MODE13_0 | GPIO_CRH_MODE13_1); // Configure PB13 as output, 50MHz
   
   // Configure PB14, the GPIO pin with the blue LED
   GPIOB->CRH &= ~(GPIO_CRH_MODE14 | GPIO_CRH_CNF14);     // Set PB14 to push-pull output mode
   GPIOB->CRH |= (GPIO_CRH_MODE14_0 | GPIO_CRH_MODE14_1); // Configure PB14 as output, 50MHz
   
   // Configure PC13, the GPIO pin with the LED
   GPIOC->CRH &= ~(GPIO_CRH_MODE13 | GPIO_CRH_CNF13);     // Set PC13 to push-pull output mode
   GPIOC->CRH |= (GPIO_CRH_MODE13_0 | GPIO_CRH_MODE13_1); // Configure PC13 as output, 50MHz
   
   // Configure PC14, the GPIO pin with 500Hz square wave
   GPIOC->CRH &= ~(GPIO_CRH_MODE14 | GPIO_CRH_CNF14);     // Set PC14 to push-pull output mode
   GPIOC->CRH |= (GPIO_CRH_MODE14_0 | GPIO_CRH_MODE14_1); // Configure PC14 as output, 50MHz
}


/* initUARTs --- set up UART(s) and buffers, and connect to 'stdout' */

static void initUARTs(void)
{
   RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;                    // Enable clock to GPIO A peripherals on APB2 bus
   RCC->APB2ENR |= RCC_APB2ENR_USART1EN;                  // Enable USART1 clock
   
   // Set up UART1 and associated circular buffers
   U1Buf.tx.head = 0;
   U1Buf.tx.tail = 0;
   U1Buf.rx.head = 0;
   U1Buf.rx.tail = 0;
   
   // Configure PA9, the GPIO pin with alternative function TxD1
   GPIOA->CRH &= ~(GPIO_CRH_MODE9 | GPIO_CRH_CNF9);       // Clear configuration bits for PA9
   GPIOA->CRH |= GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9_0 | GPIO_CRH_MODE9_1;     // Configure PA9 as alternate function, push-pull
   
   // Configure PA10, the GPIO pin with alternative function RxD1
   GPIOA->CRH &= ~(GPIO_CRH_MODE10 | GPIO_CRH_CNF10);     // Clear configuration bits for PA10
   GPIOA->CRH |= GPIO_CRH_CNF10_1;                        // Configure PA10 as alternate function, floating input
   
   // Configure UART1 - defaults are 1 start bit, 8 data bits, 1 stop bit, no parity
   USART1->CR1 |= USART_CR1_UE;           // Switch on the UART
   USART1->BRR |= (467<<4) | 12;          // Set for 9600 baud (reference manual page 799) 72000000 / (16 * 9600)
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
   TIM4->PSC = 7200 - 1;        // Prescaler: 72MHz, divide-by-7200 to give 10kHz
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
   RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;                    // Enable clock to GPIO A peripherals on APB2 bus
   RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;                    // Enable clock to GPIO B peripherals on APB2 bus
   
   // Configure PA5, the GPIO pin with alternative function SCK1
   GPIOA->CRL &= ~(GPIO_CRL_MODE5 | GPIO_CRL_CNF5);       // Clear configuration bits for PA5
   GPIOA->CRL |= (GPIO_CRL_CNF5_1 | GPIO_CRL_MODE5_0 | GPIO_CRL_MODE5_1);   // Set PA5 to push-pull 50MHz alternate output mode
   
   // Configure PA6, the GPIO pin with alternative function MISO1
   GPIOA->CRL &= ~(GPIO_CRL_MODE6 | GPIO_CRL_CNF6);       // Clear configuration bits for PA6
   GPIOA->CRL |= GPIO_CRL_CNF6_0;                         // Set PA6 to floating input mode
   
   // Configure PA7, the GPIO pin with alternative function MOSI1
   GPIOA->CRL &= ~(GPIO_CRL_MODE7 | GPIO_CRL_CNF7);       // Clear configuration bits for PA7
   GPIOA->CRL |= (GPIO_CRL_CNF7_1 | GPIO_CRL_MODE7_0 | GPIO_CRL_MODE7_1);   // Set PA7 to push-pull 50MHz alternate output mode
   
   // Configure PA4, the GPIO pin with function CS
   GPIOA->CRL &= ~(GPIO_CRL_MODE4 | GPIO_CRL_CNF4);       // Clear configuration bits for PA4
   GPIOA->CRL |= (GPIO_CRL_MODE4_0 | GPIO_CRL_MODE4_1);   // Set PA4 to push-pull 50MHz output mode
   
   // Configure PA12, the GPIO pin with function DC
   GPIOA->CRH &= ~(GPIO_CRH_MODE12 | GPIO_CRH_CNF12);     // Clear configuration bits for PA12
   GPIOA->CRH |= (GPIO_CRH_MODE12_0 | GPIO_CRH_MODE12_1); // Set PA12 to push-pull 50MHz output mode
   
   // Set up SPI1
   SPI1->CR1 = 0;
   SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR;
   SPI1->CR1 |= SPI_CR1_CPOL | SPI_CR1_CPHA;
   SPI1->CR1 |= SPI_CR1_BR_0; // 72MHz divide-by-4 gives 18MHz
   SPI1->CR1 |= SPI_CR1_SPE;  // Enable SPI
   
   spi_cs(1);
   spi_dc(1);
}


/* initMillisecondTimer --- set up a timer to interrupt every millisecond */

static void initMillisecondTimer(void)
{
   // Set up timer for regular 1ms interrupt
   if (SysTick_Config(72000)) { // 72MHz divided by 1000  (SystemCoreClock / 1000)
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
    
   updscreen();
   
   printf("\nHello from the STM%dF%d\n", 32, 103);
   
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
            drawSegCN(1 * width, style);
            drawSegCN(3 * width, style);
            
            updscreen();
            
            colon += 600u;
         }
         
         Tick = 0;
      }
      
      if (RtcTick) {
         printf("RTC: %02d:%02d:%02d\n", Hour, Minute, Second);
         
         if (displayMode == AUTO_HMS_MODE) {
            memset(Frame, 0, sizeof (Frame));
            
            renderClockDisplay(width, style);
            
            updscreen();
            
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
               setRect(0, 0, MAXX - 1, MAXY - 1);
               updscreen();
               break;
            case 'q':
            case 'Q':
               setVline(MAXX / 4,       0, MAXY - 1);
               setVline(MAXX / 2,       0, MAXY - 1);
               setVline((MAXX * 3) / 4, 0, MAXY - 1);
               updscreen();
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
               renderHexDigit(x, 0, style);
               updscreen();
               break;
            case '1':
               renderHexDigit(x, 1, style);
               updscreen();
               break;
            case '2':
               renderHexDigit(x, 2, style);
               updscreen();
               break;
            case '3':
               renderHexDigit(x, 3, style);
               updscreen();
               break;
            case '4':
               renderHexDigit(x, 4, style);
               updscreen();
               break;
            case '5':
               renderHexDigit(x, 5, style);
               updscreen();
               break;
            case '6':
               renderHexDigit(x, 6, style);
               updscreen();
               break;
            case '7':
               renderHexDigit(x, 7, style);
               updscreen();
               break;
            case '8':
               renderHexDigit(x, 8, style);
               updscreen();
               break;
            case '9':
               renderHexDigit(x, 9, style);
               updscreen();
               break;
            case 'a':
            case 'A':
               renderHexDigit(x, 0xA, style);
               updscreen();
               break;
            case 'b':
            case 'B':
               renderHexDigit(x, 0xB, style);
               updscreen();
               break;
            case 'c':
            case 'C':
               renderHexDigit(x, 0xC, style);
               updscreen();
               break;
            case 'd':
            case 'D':
               renderHexDigit(x, 0xD, style);
               updscreen();
               break;
            case 'e':
            case 'E':
               renderHexDigit(x, 0xE, style);
               updscreen();
               break;
            case 'f':
            case 'F':
               renderHexDigit(x, 0xF, style);
               updscreen();
               break;
            case 'o':
            case 'O':
               //memcpy(Frame, OLEDImage, sizeof (Frame));
               updscreen();
               break;
            case '.':
               drawSegDP(x, style);
               updscreen();
               break;
            case ':':
               drawSegCN(x, style);
               updscreen();
               break;
            case 's':
               state = SETTING_TIME_1;
               printf("OLD: %02d:%02d:%02d\n", Hour, Minute, Second);
               break;
            case 't':
               memset(Frame, 0, sizeof (Frame));
               
               renderClockDisplay(width, style);
               drawSegCN(1 * width, style);
               drawSegCN(3 * width, style);
               
               updscreen();
               
               colon = millis() + 1100u;
               break;
            case 'm':
            case 'M':
               displayMode = MANUAL_MODE;
               break;
            case 'u':
            case 'U':
               displayMode = AUTO_HMS_MODE;
               break;
            case 'v':
            case 'V':
               style = VFD_STYLE;
               break;
            case 'w':
            case 'W':
               style = LED_DOT_STYLE;
               break;
            case 'x':
            case 'X':
               style = PANAPLEX_STYLE;
               break;
            case 'y':
            case 'Y':
               style = LED_BAR_STYLE;
               break;
            case 'z':
            case 'Z':
               memset(Frame, 0, sizeof (Frame));
               updscreen();
               break;
            }
            break;
         }
      }
   }
}

