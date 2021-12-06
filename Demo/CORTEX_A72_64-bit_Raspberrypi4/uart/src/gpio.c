#include "gpio.h"

#define LEV20 20

volatile unsigned int* GPFSELx[]={  (volatile unsigned int *)(GPIO_BASE_REG + 0x00U),
                                    (volatile unsigned int *)(GPIO_BASE_REG + 0x04U),
                                    (volatile unsigned int *)(GPIO_BASE_REG + 0x08U),
                                    (volatile unsigned int *)(GPIO_BASE_REG + 0x0cU),
                                    (volatile unsigned int *)(GPIO_BASE_REG + 0x10U),
                                    (volatile unsigned int *)(GPIO_BASE_REG + 0x14U)};

volatile unsigned int* GPIO_PUP_PDN_CNTRL_REGx[]={  (volatile unsigned int *)(GPIO_BASE_REG + 0xe4U),
                                                    (volatile unsigned int *)(GPIO_BASE_REG + 0xe8U),
                                                    (volatile unsigned int *)(GPIO_BASE_REG + 0xecU),
                                                    (volatile unsigned int *)(GPIO_BASE_REG + 0xf0U)};

volatile unsigned int* GPSETx[]={   (volatile unsigned int *)(GPIO_BASE_REG + 0x1cU),
                                    (volatile unsigned int *)(GPIO_BASE_REG + 0x20U)};

volatile unsigned int* GPCLRx[]={   (volatile unsigned int *)(GPIO_BASE_REG + 0x28U),
                                    (volatile unsigned int *)(GPIO_BASE_REG + 0x2cU)};

volatile unsigned int* GPLEVx[]={   (volatile unsigned int *)(GPIO_BASE_REG + 0x34U),
                                    (volatile unsigned int *)(GPIO_BASE_REG + 0x38U)};

int gpio_pin_init(GPIO_pin_t pin, GPIO_function_t pin_func, GPIO_PULLx_t pin_pull){

    uint8_t gpio_port = pin / 10;
    uint8_t shift_value = 3 * (pin - (10*gpio_port));

    /* Set the pin function*/
    /*  Options:
        IN=0, OUT, ALT0, ALT1, ALT2, ALT3, ALT4, ALT5

        For ALTx consult the bcm2711 datasheet.
    */
    *(GPFSELx[gpio_port]) |= (pin_func << shift_value);
    
    shift_value = 0;
    /*Define pull option*/
    if(pin <= 15){
        shift_value = (pin-0)*2;
        *GPIO_PUP_PDN_CNTRL_REGx[0] |= (pin_pull << shift_value);
    }else if(pin >= 16 && pin <= 31){
        shift_value = (pin-16)*2;
        *GPIO_PUP_PDN_CNTRL_REGx[1] |= (pin_pull << shift_value);
    }else if (pin >= 32 && pin <= 47){
        shift_value = (pin-32)*2;
        *GPIO_PUP_PDN_CNTRL_REGx[2] |= (pin_pull << shift_value);
    }else if (pin >= 48 && pin <= 57){
        shift_value = (pin-48)*2;
        *GPIO_PUP_PDN_CNTRL_REGx[3] |= (pin_pull << shift_value);
    }

    return 0;
}

void gpio_pin_set(GPIO_pin_t pin, GPIO_set_clear_t value){
    uint8_t offset = 0;
    uint8_t shift_value = pin;

    if(pin >= 32 && pin <= 57){
        offset = 1;
        shift_value = pin - 32;
    }

    if(value == 1){
        *GPSETx[offset] |= (1 << shift_value); 
    }
    else if (value == 0){
        *GPCLRx[offset] |= (1 << shift_value);
    }

}

int gpio_pin_read(GPIO_pin_t pin){

    int32_t ret = -1;
    uint8_t offset = 0;
    uint8_t shift_value = pin;

    if(pin >= 32 && pin <= 57){
        offset = 1;
        shift_value = pin - 32;
    }

    ret = *GPLEVx[offset];
    ret = ret >> shift_value;
    ret = ret & 0x01;

    return ret;

}