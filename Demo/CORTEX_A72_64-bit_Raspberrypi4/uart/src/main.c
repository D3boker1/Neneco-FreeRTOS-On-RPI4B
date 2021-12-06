#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

#include "uart.h"
#include "gpio.h"

/*
 * Prototypes for the standard FreeRTOS callback/hook functions implemented
 * within this file.
 */
void vApplicationMallocFailedHook( void );
void vApplicationIdleHook( void );

TimerHandle_t timer;


static inline void io_halt(void)
{
    asm volatile ("wfi");
	return;
}
/*-----------------------------------------------------------*/

void TaskA(void *pvParameters)
{
	(void) pvParameters;

    for( ;; )
    {
	    //uart_puts("a\r\n");
		uart_puthex(gpio_pin_read());
	    //uart_puts("Task A\r\n");
		vTaskDelay(1000 / portTICK_RATE_MS);
    }

	return; /* Never reach this line */
}

/*-----------------------------------------------------------*/

/*Test 1 - Turn on and turn off the LED in 1 sec intervals*/
/*void interval_func(TimerHandle_t pxTimer)
{
	static int first_time = 1;

		if(first_time == 1){
			gpio_pin_set(1);
			first_time = 0;
		}else if (first_time == 0){
			gpio_pin_set(0);
			first_time = 1;
		}
}*/


/*
		Test 2 - Turn on the LED when receives '1'
				 Turn off the LED when receives '0'
*/
void interval_func(TimerHandle_t pxTimer)
{
	(void) pxTimer;
	uint8_t buf[2] = {0};
	uint32_t len = 0;
	static int first_time = 1;

	len = uart_read_bytes(buf, sizeof(buf) - 1);
	if (len>0){
		uart_puts((char *)buf);
		if(buf[0] == '1'){
			gpio_pin_set(1);
		}else if (buf[0] == '0'){
			gpio_pin_set(0);
		}
	}
	
	return;
}
/*-----------------------------------------------------------*/

void main(void)
{
	TaskHandle_t task_a;

	uart_init();
	gpio_pin_init();
	uart_puts("\r\n FreeRTOS over RPI4 - UART2 + LED\r\n");

	xTaskCreate(TaskA, "Task A", 512, NULL, 0x10, &task_a);

	timer = xTimerCreate("print_every_10ms",(1000 / portTICK_RATE_MS), pdTRUE, (void *)0, interval_func);
	if(timer != NULL)
	{
		xTimerStart(timer, 0);
	}

	vTaskStartScheduler();
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
}

/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
}
