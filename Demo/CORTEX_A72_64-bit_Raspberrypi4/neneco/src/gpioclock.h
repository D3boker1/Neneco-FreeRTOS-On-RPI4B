/**
 * @file gpioclock.h
 * @author Francisco Marques (fmarques_00@protonmail.com)
 * @brief This module allow the programme to use the gpio clocks on RPI 4 only.
 * @version 0.1
 * @date 2022-01-05
 * 
 * @copyright Copyright (c) 2022
 * 
 * Based on work of R. Stange <rsta2@o2online.de>
 */

#ifndef _GPIOCLocK_H_
#define _GPIOCLocK_H_

#include <stdint.h>
#include <stddef.h>
#include "errono.h"
#include "board.h"

/**< Clock Manager registers*/
#define ARM_CM_BASE		(ARM_IO_BASE + 0x101000)
#define ARM_CM_GP0CTL		(ARM_CM_BASE + 0x70)
#define ARM_CM_GP0DIV		(ARM_CM_BASE + 0x74)
#define ARM_CM_PASSWD 		(0x5A << 24)

/**
 * @brief Types of possible clocks to initialize in gpioclock module.
 * 
 * 0: on GPIO4 Alt0 or GPIO20 Alt5
 * 1: RPi 4: on GPIO5 Alt0 or GPIO21 Alt5
 * 2: on GPIO6 Alt0
 */
typedef enum GPIOClockType
{
	GPIOClock0   = 0,	
	GPIOClock1   = 1,			
	GPIOClock2   = 2,			
	GPIOClockPCM = 5,
	GPIOClockPWM = 6
}GPIOClockType_t;

/**
 * @brief Different source clocks for gpio clock manager
 * 
 * RPi 1-3:		RPi 4:
 * 19.2 MHz		54 MHz
 * 1000 MHz (varies)	1000 MHz (may vary)
 * 500 MHz		750 MHz
 * 216 MHz		unused
 */
typedef enum GPIOClockSource
{						               
	GPIOClockSourceOscillator = 1,		
	GPIOClockSourcePLLC       = 5,		
	GPIOClockSourcePLLD       = 6,
	GPIOClockSourceHDMI       = 7,		
	GPIOClockSourceUnknown    = 16
}GPIOClockSource_t;

/**
 * @brief Clock control structure. Keep tracking into the type and source of initialized clock
 * 
 */
typedef struct{
    GPIOClockType_t clock_type;
    GPIOClockSource_t clock_source;
}GPIOClockCTL_t;

/**
 * @brief Initialize the clock. It receive the type and source.
 * 
 * @param xType Type of clock that will be used.
 * @param xSource Clock source that will be used.
 * @return uint8_t 
 */
uint8_t init_gpio_clock(GPIOClockType_t xType, GPIOClockSource_t xSource);

/**
 * @brief This function enable the clock to run
 * 
 * @param nDivI 1..4095, Divider integer part. Allowed minimum depends on MASH
 * @param nDivF 0..4095 Divider fractional part 
 * @param nMASH 0..3
 */
void start_gpio_clock (unsigned	nDivI, unsigned	nDivF, unsigned	nMASH);	        

/**
 * @brief Responsible for stop the initialized clock.
 * 
 */
void stop_gpio_clock (void);

#endif  //_GPIOCLocK_H_