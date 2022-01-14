/**
 * @file main.c
 * @author Francisco Marques (fmarques_00@protonmail.com)
 * @brief The main file contains functions to test each device driver. For each driver exists a tasks and a init function.
 * @version 0.1
 * @date 2021-12-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */

/**< C libraries includes*/
#include <stddef.h>
#include <stdint.h>
//#include <stdio.h>

/**< FreeRTOS port includes*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

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
 * 
 * This task will print every 1000ms (1s) the string: "UART2 Working!\r\n"
 */
void TaskUART(void *pvParameters){
	(void) pvParameters;

    for( ;; )
    {
		uart_puts("UART2 Working\r\n");
		uart_putchar('!');
		uart_puts("\r\n");
		uart_putdec(13);
		uart_puts("\r\n");
		uart_putdec(13.13);
		uart_puts("\r\n");
		uart_puthex(0x13);
		uart_puts("\r\n");

		/**< Wait 1000 ms*/
		vTaskDelay(1000 / portTICK_RATE_MS);
    }

	return; /* Never reach this line */
}


/**
 * @brief SPI test task
 * 
 * @param pvParameters 
 * 
 * Send 0b00010000 through spi polling mode and read the incoming message and print it through UART2.
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
			uart_puts("Data:\r\n");
			uart_puthex((uint64_t)rcv_data);
			uart_puts("\r\n");
		}
		
		/**< Wait 500 ms*/
		vTaskDelay(500 / portTICK_RATE_MS);
    }

	return; /* Never reach this line */

}

/**
 * @brief PWM test task
 * 
 * @param pvParameters 
 * 
 * Test the PWM0 device on channel 1. Every 50ms increase the duty cycle 10%.
 */
void TaskPWM(void *pvParameters){
	
	(void) pvParameters;
    for( ;; )
    {
		pwm_set_dt((dt*1024)/100);//PWM0, PWM_CHANNEL_1, (dt*1024)/100);
		if(dt >= 100)
			dt=0;
		dt += 10;

		/**< Wait 50 ms*/
		vTaskDelay(50 / portTICK_RATE_MS);
    }

	return; /* Never reach this line */

}

/**
 * @brief LN298 test task
 * 
 * @param pvParameters 
 * 
 * Test the LN298 module. It will test the directions and motor duty cycle.
 * For every direction the duty cycle go from 0 to 100 % with increases of 10% each 50 ms.
 * After that, waits 100 ms and repeat for the next direction.
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
 * @brief I2C & ADS1115 test task
 * 
 * @param pvParameters 
 * 
 * This task test the I2C and ADS1115 modules.
 * This happens because the ADS1115 module use all the I2C function.
 * Thus, every 50 ms this task runs and read the value on ADS1115
 * channel 0, after that convert the read value into voltage and 
 * print it on UART2.
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
 * 
 * This task do not test 100% of GPIO module.
 * Here is tested only the function not used by the other modules.
 * Thus, this task print the value of 'encoder_counter' a variable 
 * that is incremented every time a external interrupt arrive through PIN21.
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

/**
 * @brief Interval function to test the FreeRTOS timer.
 * 
 * @param pxTimer 
 * 
 * Every 50ms this function is executed. Read the uart and turn on
 * the LED on pin 42 (Green LED on RPI4 model b) if receives '1'.
 * Turn off the LED if receives '0'
 */
void interval_func(TimerHandle_t pxTimer)
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
}
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
	xTaskCreate(TaskUART, "Task UART Test", 512, NULL, 0x10, &task_uart_test);

	uart_puts("\r\n FreeRTOS over RPI4 - UART2 + LED\r\n");
}

/**
 * @brief Dummy spi function test
 * 
 * This function initialize the spi module and start the SPI task.
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
 * This function start the PWM module and create the PWM task
 */
TaskHandle_t task_pwm_test;
void pwm_test(void){

	//PWM init
	pwm_init(1024,dt);//PWM0, PWM_CHANNEL_1, 1024, dt);
	xTaskCreate(TaskPWM, "Task PWM Test", 512, NULL, 0x10, &task_pwm_test);
}

TaskHandle_t task_i2c_test;

/**
 * @brief Dummy i2c function test
 * 
 * This function initialize the ADS1115 (This function initialize the i2c)
 * Create the ADS1115 task.
 */
void i2c_test(void){

	
	//init_i2c(0);
	init_ADS1115();
	xTaskCreate(TaskI2C, "Task I2C Test", 512, NULL, 0x10, &task_i2c_test);
}

/**
 * @brief Dummy ln298 function test
 * 
 * Initialize and start the LN298 module.
 * Create the LN298 task.
 */
TaskHandle_t task_ln298_test;
void LN298_test(void){

	//LN298 init
	ln298_init();
	ln298_start();
	xTaskCreate(TaskLN298, "Task LN298 Test", 512, NULL, 0x10, &task_ln298_test);
}

/**
 * @brief Dummy FreeRTOS timer function test
 * 
 * Create a timer to execute 'interval_func' every 50ms.
 * Start the timer.
 */
TimerHandle_t timer;
void interval_test(void){

	timer = xTimerCreate("print_every_50ms",(50 / portTICK_RATE_MS), pdTRUE, (void *)0, interval_func);
	if(timer != NULL)
	{
		xTimerStart(timer, 0);
	}
}

void main(void)
{
	/**< Uncomment the folling line to test the desire driver.*/
	uart_test();
	//spi_test();
	//pwm_test();
	//i2c_test();
	//gpio_test();
	//interval_test();
	//LN298_test();

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
