/**
 * @file pwm.c
 * @author your name (you@domain.com)
 * @brief Contains all the functions implementation for PWM module.
 * @version 0.1
 * @date 2022-01-03
 * 
 * @copyright Copyright (c) 2022
 * 
 * The PWM device and PWM channels are being ignored in version 0.1.
 * 
 */

#include "pwm.h"
#include "gpioclock.h"


uint8_t pwm_start(PWM_t pwmx, PWM_channel_t channelx){
    pwmx = PWM0;
    channelx = PWM_CHANNEL_1;

    start_gpio_clock(54, 0, 0);

    PWM_CTL |= (1 << PWMEN1);
    return VALID;
}

uint8_t pwm_stop(PWM_t pwmx, PWM_channel_t channelx){
    pwmx = PWM0;
    channelx = PWM_CHANNEL_1;

    PWM_CTL &= ~(1 << PWMEN1);
    return VALID;
}

uint8_t pwm_set_range(PWM_t pwmx, PWM_channel_t channelx, uint32_t range){
    pwmx = PWM0;
    channelx = PWM_CHANNEL_1;

    PWM_RNG1 = range;
    asm volatile ("isb");

    return VALID;
}

uint8_t pwm_set_dt(PWM_t pwmx, PWM_channel_t channelx, uint32_t duty_cycle){
    pwmx = PWM0;
    channelx = PWM_CHANNEL_1;

    //if(duty_cycle > 100)
    //    duty_cycle = 100;
    
    PWM_DAT1 = duty_cycle;
     asm volatile ("isb");

    return VALID;

}

/*uint8_t pwm_status(PWM_t pwmx, PWM_channel_t channelx){
    static flg = 0;

    if(((PWM_STA >> 9) & 0x01) != 0x01){
        if(flg == 0){
            gpio_pin_set(GPIO_42, GPIO_PIN_SET);
            flg = 1;
        }
        else{
            gpio_pin_set(GPIO_42, GPIO_PIN_CLEAR);
            flg = 0;
        }
        
    }
    return  VALID;
}*/

uint8_t pwm_init(PWM_t pwmx, PWM_channel_t channelx, uint32_t range, uint32_t duty_cycle){
    pwmx = PWM0;
    channelx = PWM_CHANNEL_1;

    /**< Initialize the PWM0 channel 0*/
    gpio_pin_init(GPIO_12, ALT0, GPIO_PIN_PULL_NON);

    /**<*/
    init_gpio_clock(GPIOClockPWM, GPIOClockSourceOscillator);

    PWM_CTL = 0;
    PWM_CTL |= ((1 << PWMEN1) | (1 << MSEN1)); /**< use M/S transmission*/
     asm volatile ("isb");

    pwm_set_range(pwmx, channelx, range);
    pwm_set_dt(pwmx, channelx, duty_cycle);
    pwm_start(pwmx, channelx);

    return VALID;

}