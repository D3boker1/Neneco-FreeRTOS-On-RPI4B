#ifndef _GPIO_H_
#define _GPIO_H_

#include <stdint.h>
#include <stddef.h>
#include "board.h"
#include "interrupt.h"

/*GPIO base registor.*/
#define GPIO_BASE_REG 0xFE200000U
/*GPIO interrupt priority*/
#define GPIO_PRIORITY 0xA0

#define ARMC 0xfe00b000

#define IRQ2_SET_EN_1 (*(volatile unsigned int *)(ARMC + 0x294))

/*Possible values for pin function. For ALT functionality see the bcm2711 datasheet page 77*/
typedef enum GPIO_function{IN=0, OUT, ALT0, ALT1, ALT2, ALT3, ALT4, ALT5}GPIO_function_t;
/*Possible values for pull resistor*/
typedef enum GPIO_PULLx{GPIO_PIN_PULL_NON=0, GPIO_PIN_PULL_UP, GPIO_PIN_PULL_DOWN}GPIO_PULLx_t;
/*Possible values for a pin: SET CLEAR*/
typedef enum GPIO_set_clear{GPIO_PIN_CLEAR=0, GPIO_PIN_SET}GPIO_set_clear_t;
/*Possible pins*/
typedef enum GPIO_pin{  GPIO_0=0, GPIO_1, GPIO_2, GPIO_3, GPIO_4, GPIO_5, GPIO_6, GPIO_7, GPIO_8, GPIO_9, 
                        GPIO_10, GPIO_11, GPIO_12, GPIO_13, GPIO_14, GPIO_15, GPIO_16, GPIO_17, GPIO_18, GPIO_19, 
                        GPIO_20, GPIO_21, GPIO_22, GPIO_23, GPIO_24, GPIO_25, GPIO_26, GPIO_27, GPIO_28, GPIO_29, 
                        GPIO_30, GPIO_31, GPIO_32, GPIO_33, GPIO_34, GPIO_35, GPIO_36, GPIO_37, GPIO_38, GPIO_39, 
                        GPIO_40, GPIO_41, GPIO_42, GPIO_43, GPIO_44, GPIO_45, GPIO_46, GPIO_47, GPIO_48, GPIO_49, 
                        GPIO_50, GPIO_51, GPIO_52, GPIO_53, GPIO_54, GPIO_55, GPIO_56, GPIO_57}GPIO_pin_t;
/*Possible types of events
    GPREN: GPIO Pin Rising Edge Detect Enable
    GPFEN: GPIO Pin Falling Edge Detect Enable
    GPHEN: GPIO Pin High Detect Enable
    GPLEN: GPIO Pin Low Detect Enable
    GPAREN: GPIO Pin Async. Rising Edge Detect
    GPAFEN: GPIO Pin Async. Falling Edge Detect
*/
typedef enum GPIO_event{GPREN=0, GPFEN, GPHEN, GPLEN, GPAREN, GPAFEN}GPIO_event_t;

int gpio_pin_init(GPIO_pin_t pin, GPIO_function_t pin_func, GPIO_PULLx_t pin_pull);

void gpio_pin_set(GPIO_pin_t pin, GPIO_set_clear_t value);

int gpio_pin_read(GPIO_pin_t pin);

int gpio_pin_isr_init(GPIO_pin_t pin, GPIO_event_t event_type);

int gpio_isr_init(void);

#endif //_GPIO_H_