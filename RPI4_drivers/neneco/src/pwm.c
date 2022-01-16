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


//uint8_t pwm_start(PWM_t pwmx, PWM_channel_t channelx){
/**< Because the PWM device and channel cannot be specified in version 0.1
     * the compiler get warnings about this variables are not in use.
     * The following lines are just to avoid this warnings.
    */
uint8_t pwm_start(void){

    start_gpio_clock(54, 0, 0);

    PWM_CTL |= (1 << PWMEN1);
    return VALID;
}

//uint8_t pwm_stop(PWM_t pwmx, PWM_channel_t channelx){
/**< Because the PWM device and channel cannot be specified in version 0.1
     * the compiler get warnings about this variables are not in use.
     * The following lines are just to avoid this warnings.
    */
uint8_t pwm_stop(void){
    PWM_CTL &= ~(1 << PWMEN1);
    return VALID;
}

//uint8_t pwm_set_range(PWM_t pwmx, PWM_channel_t channelx, uint32_t range){
/**< Because the PWM device and channel cannot be specified in version 0.1
     * the compiler get warnings about this variables are not in use.
     * The following lines are just to avoid this warnings.
    */
uint8_t pwm_set_range(uint32_t range){
    
    PWM_RNG1 = range;
    asm volatile ("isb");

    return VALID;
}

//uint8_t pwm_set_dt(PWM_t pwmx, PWM_channel_t channelx, uint32_t duty_cycle){
    /**< Because the PWM device and channel cannot be specified in version 0.1
     * the compiler get warnings about this variables are not in use.
     * The following lines are just to avoid this warnings.
    */
uint8_t pwm_set_dt(uint32_t duty_cycle){  
    PWM_DAT1 = duty_cycle;
     //asm volatile ("isb");

    return VALID;

}

//uint8_t pwm_status(PWM_t pwmx, PWM_channel_t channelx){
 /**< Because the PWM device and channel cannot be specified in version 0.1
     * the compiler get warnings about this variables are not in use.
     * The following lines are just to avoid this warnings.
    */
uint8_t pwm_status(void){  
    uint8_t ret = 1;

    if(((PWM_STA >> 9) & 0x01) != 0x01){
        ret = VALID;
    }
    return  ret;
}

//uint8_t pwm_init(PWM_t pwmx, PWM_channel_t channelx, uint32_t range, uint32_t duty_cycle){
/**< Because the PWM device and channel cannot be specified in version 0.1
     * the compiler get warnings about this variables are not in use.
     * The following lines are just to avoid this warnings.
    */
uint8_t pwm_init(uint32_t range, uint32_t duty_cycle){   

    /**< Initialize the PWM0 channel 0*/
    gpio_pin_init(GPIO_12, ALT0, GPIO_PIN_PULL_NON);

    /**< Initialize the PWM clock*/
    init_gpio_clock(GPIOClockPWM, GPIOClockSourceOscillator);

    /**< Chose the PWM operation mode*/
    PWM_CTL = 0;
    PWM_CTL |= ((1 << PWMEN1) | (1 << MSEN1)); /**< use M/S transmission*/
     asm volatile ("isb");

    pwm_set_range(range);
    pwm_set_dt(duty_cycle);
    pwm_start();

    return VALID;

}