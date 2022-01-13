/**
 * @file i2c.h
 * @author Francisco Marques (fmarques_00@protonmail.com)
 * @brief This module allow the programme to use the i2c device 1 on RPI.
 * @version 0.1
 * @date 2022-01-06
 * 
 * @copyright Copyright (c) 2022
 * 
 * Based on work of R. Stange <rsta2@o2online.de>
 */

#ifndef _I2C_H_
#define _I2C_H_

#include "gpio.h"
#include "board.h"
#include <stdint.h>
#include <stddef.h>
#include "errono.h"

typedef enum {
    FAST_MODE_100K = 0,
    FAST_MODE_400K = 1
} fastMode_t;

/**
 * @brief Before the I²C device could be used it must be firstly initialized.
 * 
 * @param bFastMode 
 * @return uint8_t 
 */
uint8_t init_i2c (fastMode_t bFastMode);

/**
 * @brief Set the clock i2c object
 * 
 * @param nClockSpeed 
 */
void set_clock_i2c (unsigned nClockSpeed);

/**
 * @brief This function is responsible for effectively receive data through the I²C device to be sent from the I²C slave
 * 
 * @param ucAddress 
 * @param pBuffer 
 * @param nCount 
 * @return int 
 */
int read_i2c(uint8_t ucAddress, void *pBuffer, unsigned nCount);

/**
 * @brief  This function is responsible for effectively send data to the I²C device to be sent to the I²C slave.
 * 
 * @param ucAddress 
 * @param pBuffer 
 * @param nCount 
 * @return int 
 */
int write_i2c(uint8_t ucAddress, const void *pBuffer, unsigned nCount);

#endif