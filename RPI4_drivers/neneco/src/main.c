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
#include <string.h>

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
#include "gearSwitch.h"

/*
 * Prototypes for the standard FreeRTOS callback/hook functions implemented
 * within this file.
 */
void vApplicationMallocFailedHook( void );
void vApplicationIdleHook( void );

#define steeringWheel_Channel 0
#define accelerator_Channel 1
#define break_Channel 2

uint8_t accelerator_percentage = 0;
uint8_t break_percentage = 0;
int percentage;

xSemaphoreHandle semOne;
xSemaphoreHandle semTwo;


uint16_t dt = 0;

static inline void io_halt(void)
{
    asm volatile ("wfi");
	return;
}
/*-----------------------------------------------------------*/

int16_t computeAngle(float voltage){
	/**< 360 degrees / max voltage = line inclination (m)*/
	return ( ((360/3.24)*voltage) - 180 );
}

void TasksteeringWheel(void *pvParameters){

	(void) pvParameters;
    for( ;; )
    {
		if( xSemaphoreTake( semOne, portMAX_DELAY ) == pdTRUE )
        {
			int16_t adc0;
			float volts0;
			//int16_t angle;

			adc0 = readADC_SingleEnded(steeringWheel_Channel);
			volts0 = computeVolts(adc0);
			//angle = computeAngle(volts0);

			uart_puts("SW: ");
			uart_putdec(volts0);
			uart_puts("?");
			//uart_puts("\r\n");

			/**< Wait 50 ms*/
			//vTaskDelay(100 / portTICK_RATE_MS);

			xSemaphoreGive(semTwo);
		}
	}
	return; /* Never reach this line */
}

uint8_t computePercentage(float voltage){
	return ( ((100/1.69) * voltage) + (100 - ((100/1.69) * 2.53)));
}

void TaskAccelerator(void *pvParameters){

	(void) pvParameters;
    for( ;; )
    {
		if( xSemaphoreTake( semTwo, portMAX_DELAY ) == pdTRUE )
        {
			int16_t adc0;
			float volts0;

			adc0 = readADC_SingleEnded(accelerator_Channel);
			volts0 = computeVolts(adc0);
			accelerator_percentage = computePercentage(volts0);
			
			adc0 = readADC_SingleEnded(break_Channel);
			volts0 = computeVolts(adc0);
			break_percentage = computePercentage(volts0);
			percentage = accelerator_percentage - break_percentage;
			if(percentage < 0)
				percentage = percentage *-1;

			ln298_set_velocity_per(percentage);

			xSemaphoreGive(semOne);

			uart_puts("A: ");
			uart_putdec(accelerator_percentage);
			uart_puts("?");
			uart_puts("B: ");
			uart_putdec(break_percentage);
			uart_puts("?");
			
			/**< Wait 50 ms*/
			//vTaskDelay(50 / portTICK_RATE_MS);
		}
	}
	return; /* Never reach this line */
}

void TaskBreak(void *pvParameters){

	(void) pvParameters;
    for( ;; )
    {
		int16_t adc0;
		float volts0;
		

		adc0 = readADC_SingleEnded(break_Channel);
		volts0 = computeVolts(adc0);
		break_percentage = computePercentage(volts0);
		percentage = accelerator_percentage - break_percentage;
		if(percentage < 0)
			percentage = percentage *-1;

		ln298_set_velocity_per(percentage);

		//uart_puts("Break value: ");
		//uart_putdec(percentage);
		//uart_puts(";\r\n");
		
		/**< Wait 50 ms*/
		vTaskDelay(1 / portTICK_RATE_MS);
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
		//uart_puts((char *)buf);
		if(buf[0] == '1'){
			gpio_pin_set(GPIO_42, GPIO_PIN_SET);
		}else if (buf[0] == '0'){
			gpio_pin_set(GPIO_42, GPIO_PIN_CLEAR);
		}
	}
	
	return;
}
/*-----------------------------------------------------------*/


void TaskGearSwitch(void *pvParameters){
	(void) pvParameters;
    for( ;; )
    {
		if( xSemaphoreTake( gearSem, portMAX_DELAY ) == pdTRUE )
        {
			switch (currentGearSwitchValue)
			{
			case GEAR_D:
				ln298_set_dir(RIGHT);
				break;
			case GEAR_P:
				ln298_set_dir(STOP);
				break;
			case GEAR_R:
				ln298_set_dir(LEFT);
				break;
		
			default:
				break;
			}
		}
    }

	return; /* Never reach this line */

}

void TaskUART(void *pvParameters){
	char oi[] = "ola";
	(void) pvParameters;
    for( ;; )
    {
		uart_puts(oi);
		//uart_putchar_isr('B');
		vTaskDelay(100 / portTICK_RATE_MS);
    }

	return; /* Never reach this line */

}

TaskHandle_t task_gear_switch;
void gear_switch_init(void){
	//gpio0 isr enable
	if ( gpio_isr_init() != 0){
		//uart_puts("\r\n gpio_isr_init error! \r\n");
		while(1);
	}

	currentGearSwitchValue = GEAR_P;
	//D
    gpio_pin_init(GPIO_7, IN, GPIO_PIN_PULL_NON);
	//P
	gpio_pin_init(GPIO_8, IN, GPIO_PIN_PULL_NON);
	//R
	gpio_pin_init(GPIO_25, IN, GPIO_PIN_PULL_NON);

	//D interrupt enable
	if(gpio_pin_isr_init(GPIO_7, GPAREN) != 0){
		//uart_puts("\r\n gpio__pin_isr_init error! \r\n");
		while(1);
	}
	//P interrupt enable
	if(gpio_pin_isr_init(GPIO_8, GPAREN) != 0){
		//uart_puts("\r\n gpio__pin_isr_init error! \r\n");
		while(1);
	}
	//R interrupt enable
	if(gpio_pin_isr_init(GPIO_25, GPAREN) != 0){
		//uart_puts("\r\n gpio__pin_isr_init error! \r\n");
		while(1);
	}

	gearSem = xSemaphoreCreateBinary();
	xTaskCreate(TaskGearSwitch, "Task GPIO Test", 512, NULL, 0x10, &task_gear_switch);
}


TaskHandle_t task_uart_test;
void uart_test(){
	//uart2 initialization
	uart_init();
	//xTaskCreate(TaskUART, "Task UART Test", 512, NULL, 0x10, &task_uart_test);
	gpio_pin_init(GPIO_42, OUT, GPIO_PIN_PULL_DOWN);
	uart_puts("Neneco - A FreeRTOS port For RPI4B\r\n");
	uart_puts("Version: 1 Tomoe Sharingan (v0.1)\r\n");
	
	//char oi[] = "oioi";

	//uart_puts(oi);
	//uart_putchar_isr('A');
}


TaskHandle_t task_steeringWheel;
TaskHandle_t task_accelerator;
TaskHandle_t task_break;
void analogDevicesInit(void){

	/**< Initialize the analog input reader*/
	init_ADS1115();

	semOne = xSemaphoreCreateBinary();
	semTwo = xSemaphoreCreateBinary();

	/**< Create the task to read the steering wheel values*/
	xTaskCreate(TasksteeringWheel, "Task steeringWheel control", 512, NULL, 5, &task_steeringWheel);
	/**< Create the task to read the accelerator values*/
	xTaskCreate(TaskAccelerator, "Task accelerator control", 512, NULL, 5, &task_accelerator);
	/**< Create the task to read the break values*/
	//xTaskCreate(TaskBreak, "Task break control", 512, NULL, 5, &task_break);

	xSemaphoreGive(semOne);
}


void LN298_initialization(void){
	ln298_init();
	ln298_start();
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
	LN298_initialization();
	gear_switch_init();
	analogDevicesInit();
	//interval_test();

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
