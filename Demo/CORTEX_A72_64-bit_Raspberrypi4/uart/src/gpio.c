#include "gpio.h"

#define GPIO_PIN_OUTPUT 1
#define GPIO_PUP_PDN_CNTRL20 8
#define PULL_UP 1
#define SET20 20
#define CLR20 20
#define LEV20 20

int gpio_pin_init(void){

    /*Make the pin 20 an output*/
    GPFSEL2 |= GPIO_PIN_OUTPUT;
    /*Define pull up for pin 20*/
    GPIO_PUP_PDN_CNTRL_REG1 |= (PULL_UP << GPIO_PUP_PDN_CNTRL20);

    return 0;
}

void gpio_pin_set(uint8_t value){

    if(value == 1){
        GPSET0 |= (1 << SET20);
    }
    else if (value == 0){
        GPCLR0 |= (1 << CLR20);
    }

}

int gpio_pin_read(void){

    int32_t ret = -1;

    ret = GPLEV0 & (1 << LEV20);

    return ret;

}