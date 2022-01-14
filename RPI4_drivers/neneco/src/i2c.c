/**
 * @file i2c.c
 * @author Francisco Marques (fmarques_00@protonmail.com)
 * @brief Implementation of i2c master module.
 * @version 0.1
 * @date 2022-01-06
 * 
 * @copyright Copyright (c) 2022
 * 
 * Based on work of R. Stange <rsta2@o2online.de>
 */

#include "i2c.h"

#define DEVICES			7
#define CONFIGS			2

#define GPIOS			2
#define GPIO_SDA		0
#define GPIO_SCL		1

/**< Control register */
#define C_I2CEN			(1 << 15)
#define C_INTR			(1 << 10)
#define C_INTT			(1 << 9)
#define C_INTD			(1 << 8)
#define C_ST			(1 << 7)
#define C_CLEAR			(1 << 5)
#define C_READ			(1 << 0)

/**< Status register*/
#define S_CLKT			(1 << 9)
#define S_ERR			(1 << 8)
#define S_RXF			(1 << 7)
#define S_TXE			(1 << 6)
#define S_RXD			(1 << 5)
#define S_TXD			(1 << 4)
#define S_RXR			(1 << 3)
#define S_TXW			(1 << 2)
#define S_DONE			(1 << 1)
#define S_TA			(1 << 0)

/**< FIFO register */
#define FIFO__MASK		0xFF
#define FIFO_SIZE		16

/**< I2C Master device*/
#define ARM_BSC0_BASE		(ARM_IO_BASE + 0x205000)
#define ARM_BSC1_BASE		(ARM_IO_BASE + 0x804000)

#define ARM_BSC_C__OFFSET		0x00
#define ARM_BSC_S__OFFSET		0x04
#define ARM_BSC_DLEN__OFFSET	0x08
#define ARM_BSC_A__OFFSET		0x0C
#define ARM_BSC_FIFO__OFFSET	0x10
#define ARM_BSC_DIV__OFFSET		0x14
#define ARM_BSC_DEL__OFFSET		0x18
#define ARM_BSC_CLKT__OFFSET	0x1C

/**< I2C device 1*/
#define I2C_DEVICE_1 ARM_IO_BASE + 0x804000

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

uint8_t init_i2c (fastMode_t bFastMode){

    /**< Initialize the I2C device 1*/
    gpio_pin_init(GPIO_2, ALT0, GPIO_PIN_PULL_NON);
    gpio_pin_init(GPIO_3, ALT0, GPIO_PIN_PULL_NON);

    if(bFastMode == FAST_MODE_400K){
        set_clock_i2c(400000);
    }
    else{
        set_clock_i2c(100000);
    }
    
    return VALID;
}

void set_clock_i2c (unsigned nClockSpeed){
    uint16_t nDivider = (uint16_t) (core_clock / nClockSpeed);
	write32 (I2C_DEVICE_1 + ARM_BSC_DIV__OFFSET, nDivider);

}

int read_i2c(uint8_t ucAddress, void *pBuffer, unsigned nCount){
	if (ucAddress >= 0x80)
	{
		return -ENOSYS;
	}

	if (nCount == 0)
	{
		return -ENOSYS;
	}

	uint8_t *pData = (uint8_t *) pBuffer;

	int nResult = 0;

	/**< setup transfer */
	write32 (I2C_DEVICE_1 + ARM_BSC_A__OFFSET, ucAddress);

	write32 (I2C_DEVICE_1 + ARM_BSC_C__OFFSET, C_CLEAR);
	write32 (I2C_DEVICE_1 + ARM_BSC_S__OFFSET, S_CLKT | S_ERR | S_DONE);

	write32 (I2C_DEVICE_1 + ARM_BSC_DLEN__OFFSET, nCount);

	write32 (I2C_DEVICE_1 + ARM_BSC_C__OFFSET, C_I2CEN | C_ST | C_READ);

	/**< transfer active */
	while (!(read32 (I2C_DEVICE_1 + ARM_BSC_S__OFFSET) & S_DONE))
	{
		while (read32 (I2C_DEVICE_1 + ARM_BSC_S__OFFSET) & S_RXD)
		{
			*pData++ = read32 (I2C_DEVICE_1 + ARM_BSC_FIFO__OFFSET) & FIFO__MASK;

			nCount--;
			nResult++;
		}
	}

	/**< transfer has finished, grab any remaining stuff from FIFO */
	while (   nCount > 0
	       && (read32 (I2C_DEVICE_1 + ARM_BSC_S__OFFSET) & S_RXD))
	{
		*pData++ = read32 (I2C_DEVICE_1 + ARM_BSC_FIFO__OFFSET) & FIFO__MASK;

		nCount--;
		nResult++;
	}

	uint32_t nStatus = read32 (I2C_DEVICE_1 + ARM_BSC_S__OFFSET);
	if (nStatus & S_ERR)
	{
		write32 (I2C_DEVICE_1 + ARM_BSC_S__OFFSET, S_ERR);

		nResult = -ENOMSG;
	}
	else if (nStatus & S_CLKT)
	{
		nResult = -ENOMSG;
	}
	else if (nCount > 0)
	{
		nResult = -ENOMSG;
	}

	write32 (I2C_DEVICE_1 + ARM_BSC_S__OFFSET, S_DONE);

	return nResult;
}

int write_i2c(uint8_t ucAddress, const void *pBuffer, unsigned nCount){
if (ucAddress >= 0x80)
	{
		return -ENOSYS;
	}

	if (nCount != 0 && pBuffer == 0)
	{
		return -ENOSYS;
	}

	uint8_t *pData = (uint8_t *) pBuffer;

	int nResult = 0;

	/**< setup transfer */
	write32 (I2C_DEVICE_1 + ARM_BSC_A__OFFSET, ucAddress);

	write32 (I2C_DEVICE_1 + ARM_BSC_C__OFFSET, C_CLEAR);
	write32 (I2C_DEVICE_1 + ARM_BSC_S__OFFSET, S_CLKT | S_ERR | S_DONE);

	write32 (I2C_DEVICE_1 + ARM_BSC_DLEN__OFFSET, nCount);

	/**< fill FIFO */
	for (unsigned i = 0; nCount > 0 && i < FIFO_SIZE; i++)
	{
		write32 (I2C_DEVICE_1 + ARM_BSC_FIFO__OFFSET, *pData++);

		nCount--;
		nResult++;
	}

	/**< start transfer */
	write32 (I2C_DEVICE_1 + ARM_BSC_C__OFFSET, C_I2CEN | C_ST);

	// transfer active
	while (!(read32 (I2C_DEVICE_1 + ARM_BSC_S__OFFSET) & S_DONE))
	{
		while (   nCount > 0
		       && (read32 (I2C_DEVICE_1 + ARM_BSC_S__OFFSET) & S_TXD))
		{
			write32 (I2C_DEVICE_1 + ARM_BSC_FIFO__OFFSET, *pData++);

			nCount--;
			nResult++;
		}
	}

	/**< check status */
	uint32_t nStatus = read32 (I2C_DEVICE_1 + ARM_BSC_S__OFFSET);
	if (nStatus & S_ERR)
	{
		write32 (I2C_DEVICE_1 + ARM_BSC_S__OFFSET, S_ERR);

		nResult = -ENOMSG;
	}
	else if (nStatus & S_CLKT)
	{
		nResult = -ENOMSG;
	}
	else if (nCount > 0)
	{
		nResult = -ENOMSG;
	}

	write32 (I2C_DEVICE_1 + ARM_BSC_S__OFFSET, S_DONE);


	return nResult;
}