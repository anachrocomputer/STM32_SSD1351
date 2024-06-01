![Static Badge](https://img.shields.io/badge/MCU-STM32-green "MCU:STM32")
![Static Badge](https://img.shields.io/badge/DISPLAY-SSD1351-green "DISPLAY:SSD1351")

# STM32_SSD1351 #

Some STM32 programs to display stuff on a 128x128 pixel
colour OLED display connected to the SPI interface.
There's a 9600 baud serial interface on USART1 (pins PA9 and PA10) which
is used to control the display.
The command 'z' will clear it,
while 'o' will show a pre-generated image.
The command 'u' will switch from manual updates to an automatically-updating
clock display with hours, minutes, and seconds.
'm' will switch back to manual updates.
The style of display is selected by 'v' for VFD, 'w' for LED dots,
'x' for Panaplex, and 'y' for LED bars.

The program is in C and may be compiled with GCC on Linux
(Windows may also work if you have a copy of GNU 'make' installed).

## Chips Supported ##

At present,
there's only support for the STM32F103 on the "Blue Pill" development board
and the STM32F411 on the "Black Pill".

## ARM Toolchain ##

A recent version of GCC for ARM,
such as that installed by the Arduino "stm32duino" toolchain.

You'll also need some files from the ST "STM32CubeF1" repo on GitHub,
such as the linker script.

