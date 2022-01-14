/**
 * @file LN298.h
 * @author Francisco Marques (fmarques_00@protonmail.com)
 * @brief Contains all the public functions and data available for LN298 module device.
 * @version 0.1
 * @date 2022-01-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef _LN298_H_
#define _LN298_H_

#include <stdint.h>
#include <stddef.h>
#include "gpio.h"
#include "pwm.h"

/**< Enumeration for motor direction.*/
typedef enum LN298_DIR{STOP=0, RIGHT, LEFT}LN298_DIR_t;

/**< Enumeration to enable the motor driver use.*/
typedef enum LN298_STATE{ENABLE=0, DISABLE}LN298_STATE_t;

/**
 * @brief Struct to handle the motor status.
 * 
 * @param direction Store the current direction value;
 * @param velocity_per Store the current motor percentage velocity;
 * 
 */
typedef struct{
    LN298_DIR_t direction;
    uint8_t velocity_per;
}ln298_status_t;

/**
 * @brief Struct to handle the motor variables.
 * 
 * @param enable Variable to enable or disable the motor control;
 * @param LN298_status provide information about the current values of direction and velocity in the motor;
 * @param DIR1_PIN pin associated to motor direction 1;
 * @param DIR2_PIN pin associated to motor direction 2;
 * @param PWM_device Variable to chose the PWM device to be used;
 * @param PWM_channel Variable to chose the PWM device channel to be used;
 * 
 */
typedef struct{
    ln298_status_t LN298_status;

    LN298_STATE_t state;
    GPIO_pin_t DIR1_PIN;
    GPIO_pin_t DIR2_PIN;
    PWM_t PWM_device;
    PWM_channel_t PWM_channel;

}ln298_ctl_t;

/**
 * @brief Function to initialize the LN298 module.
 * 
 * @return VALID on success 
 */
uint8_t ln298_init(void);

/**
 * @brief Function to enable the ln298 module use.
 * 
 * @return uint8_t 
 */
uint8_t ln298_start(void);

/**
 * @brief Stop the motor. 
 * 
 * @return ENOSYS if the module was not initialized yet; VALID on success. 
 */
uint8_t ln298_stop(void);

/**
 * @brief Update the motor direction.
 * 
 * @param dirx new value for motor direction
 * @return ENOSYS if the module was not started or if some unexpected error occur; VALID on success.  
 */
uint8_t ln298_set_dir(LN298_DIR_t dirx);

/**
 * @brief Update the motor velocity
 * 
 * @param vel_perx 
 * @return ENOSYS if the module was not started or if some unexpected error occur; VALID on success.
 */
uint8_t ln298_set_velocity_per(uint8_t vel_perx);

/**
 * @brief Read the current direction value.
 * 
 * @return [STOP, LEFT, RIGHT] 
 */
LN298_DIR_t ln298_get_dir(void);

/**
 * @brief Read the current velocity percentage value.
 * 
 * @return velocity percentage value [0 -100] 
 */
uint8_t ln298_get_velocity_perc(void);



#endif //_LN298_H_