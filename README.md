# PIC32duino

This is the Arduino port to PIC32MX1xx / 2xx family of 32-bit chips, a total of 44 chips supported so far.

As an initial release, the functionalities supported are limited. Notably missing are analogRead(), analogWrite() and serial comm function calls, and there is no support for hardware I2C, SPI or other modules on the chips.

Compiler used is XC32 and the code as is should work on C32 as well.

Porting to other PIC32 chips should be easy.
