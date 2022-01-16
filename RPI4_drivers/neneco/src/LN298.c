/**
 * @file LN298.c
 * @author Francisco Marques (fmarques_00@protonmail.com)
 * @brief Implementation of LN298 module device functions.
 * @version 0.1
 * @date 2022-01-14
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "LN298.h"

#define RANGE 1024

static ln298_ctl_t LN298_control;
static uint8_t init = 0;

uint8_t ln298_init(void){

    /**< Initial condition for motor control*/
    LN298_control.state = DISABLE;
    LN298_control.DIR1_PIN = GPIO_16;
    LN298_control.DIR2_PIN = GPIO_20;
    LN298_control.PWM_device = PWM0;
    LN298_control.PWM_channel = PWM_CHANNEL_1;

    /**< Initial condition for motor status*/
    LN298_control.LN298_status.direction = STOP;
    LN298_control.LN298_status.velocity_per = 0;

    /**< Pins and devices initialization*/
    gpio_pin_init(LN298_control.DIR1_PIN, OUT, GPIO_PIN_PULL_DOWN);
    gpio_pin_init(LN298_control.DIR2_PIN, OUT, GPIO_PIN_PULL_DOWN);
    pwm_init(RANGE, 0);//LN298_control.PWM_device, LN298_control.PWM_channel, RANGE, 0);

    gpio_pin_set(LN298_control.DIR1_PIN, GPIO_PIN_CLEAR);
    gpio_pin_set(LN298_control.DIR2_PIN, GPIO_PIN_CLEAR);

    init = 1;

    return VALID;
}

uint8_t ln298_start(void){
    if(init != 1)
        return ENOSYS;

    LN298_control.state = ENABLE;
    return VALID;
}

uint8_t ln298_stop(void){
    if(LN298_control.state != ENABLE)
        return ENOSYS;

    ln298_set_dir(STOP);
    ln298_set_velocity_per(0);
    LN298_control.state = DISABLE;

    return VALID;
}

uint8_t ln298_set_dir(LN298_DIR_t dirx){
    if(LN298_control.state != ENABLE)
        return ENOSYS;

    gpio_pin_set(LN298_control.DIR1_PIN, GPIO_PIN_CLEAR);
    gpio_pin_set(LN298_control.DIR2_PIN, GPIO_PIN_CLEAR);

    switch (dirx)
    {
    case STOP:
        gpio_pin_set(LN298_control.DIR1_PIN, GPIO_PIN_SET);
        gpio_pin_set(LN298_control.DIR2_PIN, GPIO_PIN_SET);
        break;
    case RIGHT:
        gpio_pin_set(LN298_control.DIR1_PIN, GPIO_PIN_SET);
        gpio_pin_set(LN298_control.DIR2_PIN, GPIO_PIN_CLEAR);
        break;
    case LEFT:
        gpio_pin_set(LN298_control.DIR1_PIN, GPIO_PIN_CLEAR);
        gpio_pin_set(LN298_control.DIR2_PIN, GPIO_PIN_SET);
        break;
    default:
        return ENOSYS;
        break;
    }

    return VALID;
}

uint8_t ln298_set_velocity_per(uint8_t vel_perx){

    if(LN298_control.state != ENABLE)
        return ENOSYS;

    if(vel_perx > 100)
        vel_perx = 0;

    pwm_set_dt((vel_perx*RANGE)/100);//PWM0, PWM_CHANNEL_1, (vel_perx*RANGE)/100);

    return vel_perx;

}

LN298_DIR_t ln298_get_dir(void){
    if(init != 1)
        return ENOSYS;

    return LN298_control.LN298_status.direction;
}

uint8_t ln298_get_velocity_perc(void){
    if(init != 1)
        return ENOSYS;
    
    return LN298_control.LN298_status.velocity_per;
}