/**
 * @file main.c
 * @author Francisco Marques (fmarques_00@protonmail.com)
 * @brief 
 * @version 0.1
 * @date 2021-12-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */

/**< C libraries includes*/
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

/**< FreeRTOS port includes*/
#include "../../../../Source/include/FreeRTOS.h"
#include "../../../../Source/include/task.h"
#include "../../../../Source/include/queue.h"
#include "../../../../Source/include/timers.h"
#include "../../../../Source/include/semphr.h"

/**< Drivers includes*/
#include "uart.h"
#include "gpio.h"
#include "spi.h"
#include "pwm.h"
#include "AD1115.h"
#include "LN298.h"

/*
 * Prototypes for the standard FreeRTOS callback/hook functions implemented
 * within this file.
 */
void vApplicationMallocFailedHook( void );
void vApplicationIdleHook( void );

TimerHandle_t timer;
uint16_t dt = 0;

static inline void io_halt(void)
{
    asm volatile ("wfi");
	return;
}
/*-----------------------------------------------------------*/

/**
 * @brief UART test task
 * 
 * @param pvParameters 
 */
void TaskUART(void *pvParameters){
	(void) pvParameters;

    for( ;; )
    {
		uart_puts("UART2 Working!\r\n");

		/**< Wait 1000 ms*/
		vTaskDelay(1000 / portTICK_RATE_MS);
    }

	return; /* Never reach this line */
}


/**
 * @brief SPI test task
 * 
 * @param pvParameters 
 */
void TaskSPI(void *pvParameters){

	(void) pvParameters;
	int16_t rcv_data;
    for( ;; )
    {
		rcv_data = -ENODATA;
		spi_send_data_poll(0b00010000);
		rcv_data = spi_receive_data_poll();
		if ( rcv_data != -ENODATA ){
			uart_puts("Channel1:\r\n");
			uart_puthex((uint64_t)rcv_data);
			uart_puts("\r\n");

		}
		
		/**< Wait 500 ms*/
		vTaskDelay(500 / portTICK_RATE_MS);
    }

	return; /* Never reach this line */

}

uint16_t dt_calculus(uint16_t new_dt){

return (new_dt*1024)/100;
}

/**
 * @brief PWM test task
 * 
 * @param pvParameters 
 */
void TaskPWM(void *pvParameters){
	
	(void) pvParameters;
    for( ;; )
    {
		pwm_set_dt(PWM0, PWM_CHANNEL_1, dt_calculus(dt));
		if(dt >= 100)
			dt=0;
		dt += 10;

		/**< Wait 50 ms*/
		vTaskDelay(50 / portTICK_RATE_MS);
    }

	return; /* Never reach this line */

}

/**
 * @brief 
 * 
 * @param pvParameters 
 */
void TaskLN298 (void *pvParameters){
	LN298_DIR_t direction[3]={STOP, LEFT, RIGHT};
	uint8_t i = 0;
	(void) pvParameters;
    for( ;; )
    {
		for(i = 0; i < 3; i++){
			ln298_set_dir(direction[i]);
			for (size_t d = 0; d <= 100; d+=10)
			{
				ln298_set_velocity_per(d);
				vTaskDelay(50 / portTICK_RATE_MS);
			}
		}
		
		/**< Wait 50 ms*/
		vTaskDelay(100 / portTICK_RATE_MS);
    }

	return; /* Never reach this line */

}

/**
 * @brief I2C test task
 * 
 * @param pvParameters 
 */
void TaskI2C(void *pvParameters){
	static uint8_t counter = 0;
	(void) pvParameters;
    for( ;; )
    {
		int16_t adc0;
		int16_t volts0; 

		adc0 = readADC_SingleEnded(0);
		volts0 = computeVolts(adc0);

		uart_puts("Sample number: ");
		uart_putdec(counter);
		uart_puts(": ");
		uart_putdec(volts0);
		uart_puts(";\r\n");

		counter++;
		
		/**< Wait 500 ms*/
		vTaskDelay(50 / portTICK_RATE_MS);
    }

	return; /* Never reach this line */

}

/**
 * @brief GPIO test task
 * 
 * @param pvParameters 
 */
void TaskGPIO(void *pvParameters){
	(void) pvParameters;
    for( ;; )
    {
		uart_puts("encoder value: ");
		uart_putdec(encoder_counter);
		uart_puts(";\r\n");
		vTaskDelay(50 / portTICK_RATE_MS);
    }

	return; /* Never reach this line */

}

/*-----------------------------------------------------------*/

/*Test 1 - Turn on and turn off the LED in 1 sec intervals*/
/*void interval_func(TimerHandle_t pxTimer)
{
	static integrator = 0;

		if(gpio_pin_read(GPIO_21) == 0){
			if(integrator > 0){
				integrator--;
			}
		}else{
			if(integrator < MAXIMUM){
				integrator++;
			}
		}

		if(integrator == 0){
			virt_GPIO_21 = 0;
			encoder_counter--;
		}else if(integrator >= MAXIMUM){
			virt_GPIO_21 = 1;
			encoder_counter++;
			integrator = MAXIMUM;
		}
}*/


/*
		Test 2 - Turn on the LED when receives '1'
				 Turn off the LED when receives '0'
*/
/*void interval_func(TimerHandle_t pxTimer)
{
	(void) pxTimer;
	uint8_t buf[2] = {0};
	uint32_t len = 0;

	len = uart_read_bytes(buf, sizeof(buf) - 1);
	if (len>0){
		uart_puts((char *)buf);
		if(buf[0] == '1'){
			gpio_pin_set(GPIO_42, GPIO_PIN_SET);
		}else if (buf[0] == '0'){
			gpio_pin_set(GPIO_42, GPIO_PIN_CLEAR);
		}
	}
	
	return;
}*/
/*-----------------------------------------------------------*/

/**
 * @brief Dummy gpio function test
 * 
 * This function aims to test the gpio functionality.
 * Two types of functionalities are used: input and output.
 * The output is the pin 42 that represent a LED.
 * The input is the pin 21 that represents a button.
 * The input pin has the interrupt enable with the event GPIO Pin Async. Rising Edge Detect
 * Every time that a rising edge is detected the LED is toggle.
 */
TaskHandle_t task_gpio_test;
void gpio_test(void){
	//gpio0 isr enable
	if ( gpio_isr_init() != 0){
		uart_puts("\r\n gpio_isr_init error! \r\n");
		while(1);
	}
	//LED on rasp
	gpio_pin_init(GPIO_42, OUT, GPIO_PIN_PULL_UP);
	//Button
    gpio_pin_init(GPIO_21, IN, GPIO_PIN_PULL_NON);
	xTaskCreate(TaskGPIO, "Task GPIO Test", 512, NULL, 0x10, &task_gpio_test);
	//button interrupt enable
	if(gpio_pin_isr_init(GPIO_21, GPHEN) != 0){
		uart_puts("\r\n gpio__pin_isr_init error! \r\n");
		while(1);
	}
}

/**
 * @brief Dummy uart function test
 * 
 * This function initialize the UART2 (only UART available in uart module version 0.1) and
 * create a task (TaskUART) that will print "UART2 is working!" every 1000 ms
 * Finally this function print the hex num 0x30 and the string "\r\n FreeRTOS over RPI4 - UART2 + LED\r\n"
 * before the scheduler init.
 * 
 */
TaskHandle_t task_uart_test;
void uart_test(){
	//uart2 initialization
	uart_init();
	//xTaskCreate(TaskUART, "Task UART Test", 512, NULL, 0x10, &task_uart_test);

	//uart_puthex(0x30);
	uart_puts("\r\n FreeRTOS over RPI4 - UART2 + LED\r\n");
}

/**
 * @brief Dummy spi function test
 * 
 * This function initialize the spi module and print some binary numbers in poll mode.
 * 
 */
TaskHandle_t task_spi_test;
void spi_test(void){

	//SPI0 enable
	spi_init(SPICS_0, SPICPOL_LOW, SPICPHA_LOW);
	xTaskCreate(TaskSPI, "Task SPI Test", 512, NULL, 0x10, &task_spi_test);

}

/**
 * @brief Dummy pwm function test
 * 
 */
TaskHandle_t task_pwm_test;
void pwm_test(void){

	//PWM init
	pwm_init(PWM0, PWM_CHANNEL_1, 1024, dt);
	xTaskCreate(TaskPWM, "Task PWM Test", 512, NULL, 0x10, &task_pwm_test);
}

TaskHandle_t task_i2c_test;
void i2c_test(void){

	//I2C init
	init_i2c(0);
	init_ADS1115();
	xTaskCreate(TaskI2C, "Task I2C Test", 512, NULL, 0x10, &task_i2c_test);
}

/**
 * @brief 
 * 
 */
TaskHandle_t task_ln298_test;
void LN298_test(void){

	//LN298 init
	ln298_init();
	ln298_start();
	xTaskCreate(TaskLN298, "Task LN298 Test", 512, NULL, 0x10, &task_ln298_test);
}


void main(void)
{
	
	uart_test();
	//spi_test();
	//pwm_test();
	//i2c_test();
	//gpio_test();
	LN298_test();

	/*timer = xTimerCreate("print_every_50ms",(50 / portTICK_RATE_MS), pdTRUE, (void *)0, interval_func);
	if(timer != NULL)
	{
		xTimerStart(timer, 0);
	}*/
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
