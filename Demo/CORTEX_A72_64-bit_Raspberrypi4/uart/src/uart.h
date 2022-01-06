
/**
 * @file uart.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-12-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef _UART_H_
#define _UART_H_

#include "gpio.h"

#define UART_PRIORITY (0xA0)

/**
 * @brief Write a character on UART
 * 
 * @param c : character to write.
 */
void uart_putchar(uint8_t c);

/**
 * @brief Write a string.
 * 
 * @param str: pointer to string to be written
 */
void uart_puts(const char* str);

/**
 * @brief Write a hexadecimal number through UART
 * 
 * @param v: hexdecimal number to write
 */
void uart_puthex(uint64_t v);

/**
 * @brief Read a specied number of bytes form UART2
 * 
 * @param buf: buffer pointer
 * @param length: number of bytes to read 
 * @return uint32_t 
 */
uint32_t uart_read_bytes(uint8_t *buf, uint32_t length);

/**
 * @brief Initialize the UART 2 (for now) to receive and send characters.
 * 
 */
void uart_init(void);

#endif // _UART_H_