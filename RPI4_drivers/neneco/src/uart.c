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
#include "gpio.h"
#include "fifo.h"

/**
 * @brief Type of structure for Tx buffer
 * 
 * The available options are the fifo module created by Francisco Marques
 * or the messages queue provided by FreeRTOS port.
 */

//#define USE_TX_FIFO
#define USE_TX_QUEUE


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
#define UART_MIS  (*(volatile unsigned int *)(UART_BASE+0x40U))
#define UART_ICR  (*(volatile unsigned int *)(UART_BASE+0x44U))
#define UART_DMARC  (*(volatile unsigned int *)(UART_BASE+0x48U))
#define UART_ITOP  (*(volatile unsigned int *)(UART_BASE+0x88U))


/**< GPIO */
#define GPIO_BASE (0xFE200000U)
#define GPFSEL0   (*(volatile unsigned int *)(GPIO_BASE))
#define GPIO_PUP_PDN_CNTRL_REG0 (*(volatile unsigned int *)(GPIO_BASE+0xE4U))

#define MAX 100

#define TX 32

struct UARTCTL {
	SemaphoreHandle_t tx_mux;
	QueueHandle_t     rx_queue;
	/*#if defined(USE_TX_QUEUE)
	QueueHandle_t     tx_queue;
	#elif defined(USE_TX_FIFO)
	fifo_t tx_fifo;
	uint8_t tx_queue[TX];
	#endif*/
};
struct UARTCTL *uartctl;

/*
void triggerTxSent(){
	int16_t c;
	
	if(!(UART_IMSC & (0x1U << 5))){

		#if defined(USE_TX_QUEUE)
		if (xQueueReceive(uartctl->tx_queue, &c, (portTickType) portMAX_DELAY) == pdPASS)
		{
			xSemaphoreTake(uartctl->tx_mux, (portTickType) portMAX_DELAY);
			while ( UART_FR & ( 1 << 5) ) { }
			UART_DR = (uint8_t)0xFF & c;
			asm volatile ("isb");
			UART_IMSC |= ((0x1U << 5));
			UART_ITOP |= (1<<9);
			gpio_pin_set(GPIO_42, GPIO_PIN_SET);
    		asm volatile ("isb");
			xSemaphoreGive(uartctl->tx_mux);
		}
		#elif defined(USE_TX_FIFO)

		c = fifo_get_char(&uartctl->tx_fifo);
		
		if(c != -ENODATA){
			xSemaphoreTake(uartctl->tx_mux, (portTickType) portMAX_DELAY);
			while ( UART_FR & (1 << 5) ) { }
			UART_DR = (uint8_t)0xFF & c;
    		asm volatile ("isb");
			xSemaphoreGive(uartctl->tx_mux);
			UART_IMSC |= (0x1U << 5);
		}

		#endif
	}

}

void uart_putchar_isr(uint8_t c)
{

	#if defined(USE_TX_QUEUE)

	uint32_t num = uxQueueMessagesWaiting(uartctl->tx_queue);
	if(num <= 16){
		if (xQueueSendToBack(uartctl->tx_queue, &c, (portTickType) portMAX_DELAY) == pdPASS){
			triggerTxSent();
		}
	}

	#elif defined(USE_TX_FIFO)

	xSemaphoreTake(uartctl->tx_mux, (portTickType) portMAX_DELAY);
	if (fifo_put_char(&uartctl->tx_fifo, c) != -ENOBUFS){
		triggerTxSent();
	}
	xSemaphoreGive(uartctl->tx_mux);
	
	#endif
	
}

uint32_t uart_puts_isr(uint8_t *buf)
{
	uint32_t i = 0;

	#if defined(USE_TX_QUEUE)
	uint32_t num = TX - uxQueueMessagesWaiting(uartctl->tx_queue);

	while(buf[i] != '\0' && num != 0){
		if (xQueueSendToBack(uartctl->tx_queue, &buf[i], (portTickType) portMAX_DELAY) == pdPASS){
			i++;
		}
	}
	triggerTxSent();
	#elif defined(USE_TX_FIFO)
	xSemaphoreTake(uartctl->tx_mux, (portTickType) portMAX_DELAY);
	while(buf[i] != '\0'){
		if (fifo_put_char(&uartctl->tx_fifo, buf[i]) != -ENOBUFS){
			i++;
		}
	}
	xSemaphoreGive(uartctl->tx_mux);
	triggerTxSent();
	
	#endif

	return i;
}*/

void uart_putchar(uint8_t c)
{
	xSemaphoreTake(uartctl->tx_mux, (portTickType) portMAX_DELAY);

	while ( UART_FR & (0x20) ) { }
	UART_DR = c;
    asm volatile ("isb");
	xSemaphoreGive(uartctl->tx_mux);
}
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

void uart_puts(const char* str)
{
	for (size_t i = 0; str[i] != '\0'; i ++)
		uart_putchar((uint8_t)str[i]);
}
/*----------------------------------------------------------*/

void uart_puthex(uint64_t v)
{
	const char *hexdigits = "0123456789ABCDEF";
	for (int i = 60; i >= 0; i -= 4)
		uart_putchar(hexdigits[(v >> i) & 0xf]);
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
		num_print = next_num_to_print+0x30;
        final_num[(count_int-1)-i] = num_print;
        integer_part = integer_part / 10;
    }
    
    
    if(fractional_part > 0){
        final_num[count_int] = '.';
        count_frac = how_long(fractional_part);
        for(int i = 0; i < count_frac; i++)
        {
            next_num_to_print = fractional_part % 10;
            num_print = next_num_to_print+0x30;
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
	uint8_t c;
	
	UART_ICR = UART_MIS;
	gpio_pin_set(GPIO_42, GPIO_PIN_SET);

	/* RX data */
	if( !(UART_FR & (0x1U << 4)) ) {
		c = (uint8_t) 0xFF & UART_DR;
		xQueueSendToBackFromISR(uartctl->rx_queue, &c, NULL);
	}
	/* TX data */
	
	/*if( !(UART_FR & (0x1U << 5)) ) {

		#if defined(USE_TX_QUEUE)

		uint32_t num = uxQueueMessagesWaitingFromISR(uartctl->tx_queue);

		if( num > 0){
			xQueueReceiveFromISR(uartctl->tx_queue, &c, NULL);
			xSemaphoreTakeFromISR(uartctl->tx_mux, NULL);
			while ( UART_FR & (1 << 5) ) { }
			UART_DR = (uint8_t)0xFF & c;
    		asm volatile ("isb");
			xSemaphoreGiveFromISR(uartctl->tx_mux, NULL);
			gpio_pin_set(GPIO_42, GPIO_PIN_SET^gpio_pin_read(GPIO_42));
			
		}
		else{
			UART_IMSC &= ~(0x1U << 5);
			asm volatile ("isb");
			//gpio_pin_set(GPIO_42, GPIO_PIN_SET^gpio_pin_read(GPIO_42));
		}

		#elif defined(USE_TX_FIFO)

		xSemaphoreTakeFromISR(uartctl->tx_mux, NULL);
		c = fifo_get_char(&uartctl->tx_fifo);
		xSemaphoreGiveFromISR(uartctl->tx_mux, NULL);

		if( c != ENODATA){
			xSemaphoreTakeFromISR(uartctl->tx_mux, NULL);
			while ( UART_FR & (0x20) ) { }
			UART_DR = (uint8_t)0xFF & c;
    		asm volatile ("isb");
			xSemaphoreGiveFromISR(uartctl->tx_mux, NULL);
			gpio_pin_set(GPIO_42, GPIO_PIN_SET^gpio_pin_read(GPIO_42));
		}
		else{
			UART_IMSC &= ~(0x1U << 5);
			asm volatile ("isb");
		}

		#endif
	}*/
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
    UART_IBRD = 0x4EU;          /* 115200 baud */
    UART_FBRD = 0xDU;
    UART_LCRH = ((0x3U << 5) | (0x1U << 4));    /* 8/n/1, FIFO disabled */
    UART_IMSC = (0x1U << 4);    /* RX interrupt enabled */
    UART_CR   = 0x301;          /* Enables Tx, Rx and UART */
	UART_DMARC = (1 << 1);
    asm volatile ("isb");

	uartctl = pvPortMalloc(sizeof (struct UARTCTL));
	uartctl->tx_mux = xSemaphoreCreateMutex();
	uartctl->rx_queue = xQueueCreate(16, sizeof (uint8_t));

	/*#if defined(USE_TX_QUEUE)
	uartctl->tx_queue = xQueueCreate(TX, sizeof (uint8_t));
	#elif defined(USE_TX_FIFO)
	init_fifo(&uartctl->tx_fifo, uartctl->tx_queue, TX);
	#endif	*/

/*#if defined(__LINUX__)
	uart_puts("\r\nWaiting until Linux starts booting up ...\r\n");
	wait_linux();
#endif*/

	isr_register(IRQ_VC_UART, UART_PRIORITY, (0x1U << 0x3U), uart_isr);
    return;
}
/*-----------------------------------------------------------*/
