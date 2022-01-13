/**
 * @file gpioclock.c
 * @author Francisco Marques (fmarques_00@protonmail.com)
 * @brief Implementation of gpioclock module.
 * @version 0.1
 * @date 2022-01-06
 * 
 * @copyright Copyright (c) 2022
 * 
 * Based on work of R. Stange <rsta2@o2online.de>
 * 
 */
#include "gpioclock.h"

#define CLK_CTL_MASH(x)		((x) << 9)
#define CLK_CTL_BUSY		(1 << 7)
#define CLK_CTL_KILL		(1 << 5)
#define CLK_CTL_ENAB		(1 << 4)
#define CLK_CTL_SRC(x)		((x) << 0)

#define CLK_DIV_DIVI(x)		((x) << 12)
#define CLK_DIV_DIVF(x)		((x) << 0)

GPIOClockCTL_t clock_control;

/**
 * @brief Write 32-bit value to MMIO address
 * 
 * @param nAddress 
 * @param nValue 
 */
static inline void write32 (unsigned long nAddress, uint32_t nValue)
{
	*(uint32_t volatile *) nAddress = nValue;
}

/**
 * @brief Read 32-bit value from MMIO address
 * 
 * @param nAddress 
 * @return uint32_t 
 */
static inline uint32_t read32 (unsigned long nAddress)
{
	return *(uint32_t volatile *) nAddress;
}

uint8_t init_gpio_clock(GPIOClockType_t xType, GPIOClockSource_t xSource){

    clock_control.clock_type = xType;
    clock_control.clock_source = xSource;

    return VALID;
}

void start_gpio_clock (unsigned	nDivI, unsigned	nDivF, unsigned	nMASH){

	unsigned nCtlReg = ARM_CM_GP0CTL + (clock_control.clock_type * 8);
	unsigned nDivReg  = ARM_CM_GP0DIV + (clock_control.clock_type * 8);

	stop_gpio_clock();

	write32 (nDivReg, ARM_CM_PASSWD | CLK_DIV_DIVI (nDivI) | CLK_DIV_DIVF (nDivF));

	write32 (nCtlReg, ARM_CM_PASSWD | CLK_CTL_MASH (nMASH) | CLK_CTL_SRC (clock_control.clock_source));

	write32 (nCtlReg, read32 (nCtlReg) | ARM_CM_PASSWD | CLK_CTL_ENAB);

}

void stop_gpio_clock (void){

	unsigned nCtlReg = ARM_CM_GP0CTL + (clock_control.clock_type * 8);

	write32 (nCtlReg, ARM_CM_PASSWD | CLK_CTL_KILL);
	while (read32 (nCtlReg) & CLK_CTL_BUSY)
	{
		// wait for clock to stop
	}
}