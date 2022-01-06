/**
 * @file i2c.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-01-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef _I2C_H_
#define _I2C_H_

#include "gpio.h"
#include "board.h"
#include <stdint.h>
#include <stddef.h>
#include "errono.h"



uint8_t init_i2c (uint8_t bFastMode);

void set_clock_i2c (unsigned nClockSpeed);

int read_i2c(uint8_t ucAddress, void *pBuffer, unsigned nCount);

int write_i2c(uint8_t ucAddress, const void *pBuffer, unsigned nCount);

#endif