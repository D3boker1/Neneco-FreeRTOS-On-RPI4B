# Neneco

Neneco project is a unoficial port of FreeRTOS for Raspberry Pi 4 Model B.
Neneco provide an abstraction layer in order to make the developement of programs on RPI4B with FreeRTOS as easy as possible. This project 
began when the main developer needed it to create a program with real time requirements and has as constraint the RPI4B board.

    > Neneco project is currently on version beta (0.1). 
    
## Available Drivers

On currently version the supported drivers are:

-  [X] GPIO
-  [X] GCI-400 (Interrupt controller) 
-  [ ] UART

    The driver only implements UART2. In further versions will be added suport to all UART's.
-  [ ] I2C

    The driver only implements I2C device 1. In further versions will be added suport to all I2C devices.
-  [ ] SPI

    The driver only implements SPI device 0. In further versions will be added suport to all SPI devices.
-  [ ] PWM

    The driver only implements PWM device 0 channel 1. In further versions will be added suport to all PWM's and channels.
-  [ ] GPIO Clock Manager

    The driver only implements PWM clock. In further versions will be added suport to all clocks.
    
### Extra
Neneco implements some drivers for board comonly used in projects. For now, neneco give support to:
-  [X] ADS1115 - A four ADC channel to I²C.
-  [X] L298N - A dual H-Bridge motor driver which allows speed and direction control of two DC motors at the same time.

# Como usar

1. Copiar a imagem para o cartão sd através do comando dd com a imagem sdcard.img
2. copiar o ficheiro config.txt para a partição boot
3. copiar o ficheiro kernel8.img para a partição boot
4. copiar o ficheiro elf com a app freeRTOS para a partição boot
5. Para correr na rpi:

dcache off

fatload mmc 0 0x28000000 uart.elf

dcache flush

bootelf 0x28000000
