/**
 * @file gpioclock.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-01-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef _GPIOCLocK_H_
#define _GPIOCLocK_H_

#include <stdint.h>
#include <stddef.h>
#include "errono.h"

#define ARM_IO_BASE		0xFE000000
#define ARM_CM_BASE		(ARM_IO_BASE + 0x101000)

#define ARM_CM_GP0CTL		(ARM_CM_BASE + 0x70)
#define ARM_CM_GP0DIV		(ARM_CM_BASE + 0x74)

#define ARM_CM_PASSWD 		(0x5A << 24)


typedef enum GPIOClockType
{
	GPIOClock0   = 0,			// on GPIO4 Alt0 or GPIO20 Alt5
	GPIOClock1   = 1,			// RPi 4: on GPIO5 Alt0 or GPIO21 Alt5
	GPIOClock2   = 2,			// on GPIO6 Alt0
	GPIOClockPCM = 5,
	GPIOClockPWM = 6
}GPIOClockType_t;

typedef enum GPIOClockSource
{						                // RPi 1-3:		RPi 4:
	GPIOClockSourceOscillator = 1,		// 19.2 MHz		54 MHz
	GPIOClockSourcePLLC       = 5,		// 1000 MHz (varies)	1000 MHz (may vary)
	GPIOClockSourcePLLD       = 6,		// 500 MHz		750 MHz
	GPIOClockSourceHDMI       = 7,		// 216 MHz		unused
	GPIOClockSourceUnknown    = 16
}GPIOClockSource_t;

typedef struct{
    GPIOClockType_t clock_type;
    GPIOClockSource_t clock_source;
}GPIOClockCTL_t;

/**
 * @brief 
 * 
 * @param xType Type of clock that will be used.
 * @param xSource Clock source that will be used.
 * @return uint8_t 
 */
uint8_t init_gpio_clock(GPIOClockType_t xType, GPIOClockSource_t xSource);

/**
 * @brief 
 * 
 * @param nDivI 1..4095, Divider integer part. Allowed minimum depends on MASH
 * @param nDivF 0..4095 Divider fractional part 
 * @param nMASH 0..3
 */
void start_gpio_clock (unsigned	nDivI, unsigned	nDivF, unsigned	nMASH);	        

/**
 * @brief 
 * 
 */
void stop_gpio_clock (void);

#endif  //_GPIOCLocK_H_