#ifndef _GPIO_H_
#define _GPIO_H_

#include <stdint.h>
#include <stddef.h>
/*
    This code only works for GPIO pin 20 for now.
*/
#define GPIO_BASE_REG (0xFE200000U)
#define GPFSEL2 (*(volatile unsigned int *)(GPIO_BASE_REG + 0x08U))
#define GPSET0  (*(volatile unsigned int *)(GPIO_BASE_REG + 0x1cU))
#define GPCLR0  (*(volatile unsigned int *)(GPIO_BASE_REG + 0x28U))
#define GPLEV0  (*(volatile unsigned int *)(GPIO_BASE_REG + 0x34U))
#define GPIO_PUP_PDN_CNTRL_REG1 (*(volatile unsigned int *)(GPIO_BASE_REG + 0xe8U))


int gpio_pin_init(void);

void gpio_pin_set(uint8_t value);

int gpio_pin_read(void);

#endif //_GPIO_H_