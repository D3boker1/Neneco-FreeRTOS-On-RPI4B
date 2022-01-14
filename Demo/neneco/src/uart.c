/**
 * @file uart.c
 * @author Francisco Marques (fmarques_00@protonmail.com)
 * @brief Contains UART module function implementation.
 * @version 0.1
 * @date 2022-01-14
 * 
 * @copyright Copyright (c) 2022
 * 
 * Currently only UART2 is being used.
 * 
 */

#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "board.h"
#include "interrupt.h"
#include "semphr.h"
#include "uart.h"

/**< PL011 UART on Raspberry pi 4B */
#define UART_BASE  (0xFE201400U) /* UART2 */
#define UART_DR   (*(volatile unsigned int *)(UART_BASE))
#define UART_FR   (*(volatile unsigned int *)(UART_BASE+0x18U))
#define UART_IBRD (*(volatile unsigned int *)(UART_BASE+0x24U))
#define UART_FBRD (*(volatile unsigned int *)(UART_BASE+0x28U))
#define UART_LCRH (*(volatile unsigned int *)(UART_BASE+0x2CU))
#define UART_CR   (*(volatile unsigned int *)(UART_BASE+0x30U))
#define UART_IFLS (*(volatile unsigned int *)(UART_BASE+0x34U))
#define UART_IMSC (*(volatile unsigned int *)(UART_BASE+0x38U))
#define UART_ICR  (*(volatile unsigned int *)(UART_BASE+0x44U))

/**< GPIO */
#define GPIO_BASE (0xFE200000U)
#define GPFSEL0   (*(volatile unsigned int *)(GPIO_BASE))
#define GPIO_PUP_PDN_CNTRL_REG0 (*(volatile unsigned int *)(GPIO_BASE+0xE4U))

#define MAX 100

struct UARTCTL {
	SemaphoreHandle_t tx_mux;
	QueueHandle_t     rx_queue;
};
struct UARTCTL *uartctl;

void uart_putchar(uint8_t c)
{
	xSemaphoreTake(uartctl->tx_mux, (portTickType) portMAX_DELAY);
	/* wait until tx becomes idle. */
	while ( UART_FR & (0x20) ) { }
	UART_DR = c;
    asm volatile ("isb");
	xSemaphoreGive(uartctl->tx_mux);
}
/*-----------------------------------------------------------*/

void uart_putchar_isr(uint8_t c)
{
	xSemaphoreTakeFromISR(uartctl->tx_mux, NULL);
	/* wait mini uart for tx idle. */
	while ( (UART_FR & 0x20) ) { }
	UART_DR = c;
    asm volatile ("isb");
	xSemaphoreGiveFromISR(uartctl->tx_mux, NULL);
}
/*-----------------------------------------------------------*/

void uart_puts(const char* str)
{
	for (size_t i = 0; str[i] != '\0'; i ++)
		uart_putchar((uint8_t)str[i]);
}
/*-----------------------------------------------------------*/

void uart_puthex(uint64_t v)
{
	const char *hexdigits = "0123456789ABCDEF";
	for (int i = 60; i >= 0; i -= 4)
		uart_putchar(hexdigits[(v >> i) & 0xf]);
}

static char which_num(uint8_t num){
    char send = '\0';
    switch (num)
		{
		case 0:
			send = '0';
			break;
		case 1:
			send = '1';
			break;
		case 2:
			send = '2';
			break;
		case 3:
			send = '3';
			break;
		case 4:
			send = '4';
			break;
		case 5:
			send = '5';
			break;
		case 6:
			send = '6';
			break;
		case 7:
			send = '7';
			break;
		case 8:
			send = '8';
			break;
		case 9:
			send = '9';
			break;
		default:
			break;
		}
    
    return send;
}

static uint8_t how_long(uint64_t num){
    
    uint8_t count = 0;
    
    while(num > 0){
        num = num / 10;
        if(num != 0)
            count ++;
    }

    return count+1;
}

void uart_putdec(float num)
{
    int64_t integer_part = num/1;
	int16_t fractional_part = (num-integer_part)*100;
    uint8_t next_num_to_print = 0;
    
    uint8_t count_int = 0;
    uint8_t count_frac = 0;
    char num_print = '\0';
    
    char final_num[MAX];

	if(num < 0){
		integer_part = integer_part * -1;
        fractional_part = fractional_part * 1; 
		uart_putchar('-');
	}
	
	count_int = how_long(integer_part);
    for(int i = 0; i < count_int; i++)
    {
        next_num_to_print = integer_part % 10;
		num_print = which_num(next_num_to_print);
        final_num[(count_int-1)-i] = num_print;
        integer_part = integer_part / 10;
    }
    
    
    if(fractional_part > 0){
        final_num[count_int] = ',';
        count_frac = how_long(fractional_part);
        for(int i = 0; i < count_frac; i++)
        {
            next_num_to_print = fractional_part % 10;
            num_print = which_num(next_num_to_print);
            final_num[(count_int + count_frac)-i] = num_print;
            fractional_part = fractional_part / 10;
        }
		final_num[count_int+count_frac+1] = '\0';
    }else{
        final_num[count_int] = '\0';
    }
    
    uart_puts(final_num);
}

/*-----------------------------------------------------------*/

/*
	This function is responsible for read the data present in the queue.
	First it check if there is data 'num', after that, copy the data in queue
	to the buffer received by argument.
*/
uint32_t uart_read_bytes(uint8_t *buf, uint32_t length)
{
	uint32_t num = uxQueueMessagesWaiting(uartctl->rx_queue);
	uint32_t i;

	for (i = 0; i < num || i < length; i++) {
		xQueueReceive(uartctl->rx_queue, &buf[i], (portTickType) portMAX_DELAY);
	}

	return i;
}
/*-----------------------------------------------------------*/

/*
	This function is responsible for receive the chars from uart 
	and put them into the queue.
*/
void uart_isr(void)
{
	/* RX data */
	if( !(UART_FR & (0x1U << 4)) ) {
		uint8_t c = (uint8_t) 0xFF & UART_DR;
		xQueueSendToBackFromISR(uartctl->rx_queue, &c, NULL);
	}
}
/*-----------------------------------------------------------*/

/* 
 * wait_linux()
 * This is a busy loop function to wait until Linux completes GIC initialization
 */
/*static void wait_linux(void)
{
	wait_gic_init();
	return;
}*/
/*-----------------------------------------------------------*/

void uart_init(void)
{

    /* GPIO0 GPIO1 settings for UART2 */
    gpio_pin_init(GPIO_0, ALT4, GPIO_PIN_PULL_NON);
    gpio_pin_init(GPIO_1, ALT4, GPIO_PIN_PULL_NON);

	/* PL011 settings with assumption of 48MHz clock */
    UART_ICR  = 0x7FFU;         /* Clears an interrupt */
    UART_IBRD = 0x1AU;          /* 115200 baud */
    UART_FBRD = 0x3U;
    UART_LCRH = ((0x3U << 5) | (0x0U << 4));    /* 8/n/1, FIFO disabled */
    UART_IMSC = (0x1U << 4);    /* RX interrupt enabled */
    UART_CR   = 0x301;          /* Enables Tx, Rx and UART */
    asm volatile ("isb");

	uartctl = pvPortMalloc(sizeof (struct UARTCTL));
	uartctl->tx_mux = xSemaphoreCreateMutex();
	uartctl->rx_queue = xQueueCreate(16, sizeof (uint8_t));

/*#if defined(__LINUX__)
	uart_puts("\r\nWaiting until Linux starts booting up ...\r\n");
	wait_linux();
#endif*/

	isr_register(IRQ_VC_UART, UART_PRIORITY, (0x1U << 0x3U), uart_isr);
    return;
}
/*-----------------------------------------------------------*/
