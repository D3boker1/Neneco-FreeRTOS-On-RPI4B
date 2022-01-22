/**
 * @file spi.h
 * @author Francisco Marques (fmarques_00@protonmail.com)
 * @brief Contains all the public functions and data available for SPI module.
 * @version 0.1
 * @date 2021-12-29
 * 
 * @copyright Copyright (c) 2021
 * 
 * This module is currently in development. DO NOT USE IT.
 * 
 */
#ifndef _SPI_H_
#define _SPI_H_

#include <stdint.h>
#include <stddef.h>
#include "board.h"
#include "interrupt.h"
#include "gpio.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"
#include "errono.h"

/**
 * @brief SPI0 (For now) registers
 * 
 */

#define SPI_PRIORITY (0xA0)
#define SPI0_BASE  (0xFE204000U) /**< SPI0*/

#define SPI_CS   (*(volatile unsigned int *)(SPI0_BASE + 0x00U)) /**< SPI Master control and Status*/
#define CS 0 /**< Chip Select Bits (0:1)*/
#define CPHA 2 /**< Clock phase bit*/
#define CPOL 3 /**< Clock Polarity bit*/
#define CLEAR 4 /**< FIFO clear bits (4:5)*/
#define CSPOL 6 /**< Chip select polarity bit*/
#define TA 7 /**< Transfer active bit*/
#define INTD 9 /**< Interrupt on Done*/
#define INTR 10 /**< Interrupt on RXR*/
#define DONE 16 /**< Transfer Done*/
#define RXD 17 /**< Rx FIFO contains data*/
#define TXD 18 /**< Tx FIFO can accept data*/
#define RXR 19 /**< Rx FIFO needs reading 3/4 full*/
#define RXF 20 /**< RX FIFO full*/
#define CSPOL0 21 /**< Chip select 0 polarity*/
#define CSPOL1 22 /**< Chip select 1 polarity*/
#define CSPOL2 23 /**< Chip select 2 polarity*/


#define SPI_FIFO   (*(volatile unsigned int *)(SPI0_BASE + 0x04U)) /**< SPI master Tx and Rx FIFOs*/
#define SPI_CLK   (*(volatile unsigned int *)(SPI0_BASE + 0x08U)) /**< SPI master clock divider*/

/**< */
typedef enum SPI{ SPI0 = 0, SPI3, SPI4, SPI5, SPI6}SPI_t;

/**< */
typedef enum SPICPHA{
    SPICPHA_LOW = 0,
    SPICPHA_HIGH
}SPICPHA_t;

/**< */
typedef enum SPICPOL{
    SPICPOL_LOW = 0,
    SPICPOL_HIGH
} SPICPOL_t;

/**< */
typedef enum  SPICS{
    SPICS_0 = 0b00,
    SPICS_1 = 0b01,
    SPICS_2 = 0b10
}SPICS_t;

#define MAX_LEN 32

typedef struct{
    SemaphoreHandle_t tx_mux;
    QueueHandle_t tx_queue;
    SemaphoreHandle_t rx_mux;
    QueueHandle_t rx_queue;
}spictl_t;


/**
 * @brief Initialize the SPI device.
 * 
 * @param spix Device to be initialized
 * @param csx Chip select to be used
 * @param cpoolx Pool state
 * @param cphax  Phase state
 * @return uint8_t VALID in case of success. ENOSYS in case of failure.
 */
//uint8_t spi_init(SPI_t spix, SPICS_t csx, SPICPOL_t cpoolx, SPICPHA_t cphax);
uint8_t spi_init(SPICS_t csx, SPICPOL_t cpoolx, SPICPHA_t cphax);

/**
 * @brief 
 * 
 * @param spix 
 * @return uint8_t 
 */
//uint8_t spi_isr_init(SPI_t spix);

/**
 * @brief Insert data in a queue to be transmitted.
 * 
 * @param data Byte to be sent
 * @return uint8_t 
 * 
 * This function start calculating the amount of free slots on transmit queue. 
 * If there is at least one space free, the received data is inserted in transmit queue. 
 * To avoid race conditions the insert operation require a mutex control. 
 * Before leave the function, the Transmit Active bit is set to active the byte transmission.
 * In case of success the VALID macro is returned, in the other hand, 
 * if the queue is full the macro ENOBUFS is returned.
 */
uint8_t spi_send_data(const uint8_t data);

/**
 * @brief 
 * 
 * @param data_poll 
 * @return uint8_t 
 */
uint8_t spi_send_data_poll(const uint8_t data_poll);

/**
 * @brief 
 * 
 * @return int16_t 
 */
int16_t spi_receive_data_poll(void);

/**
 * @brief 
 * 
 * @return uint16_t 
 */
int16_t spi_receive_data(void);


#endif //_SPI_H_