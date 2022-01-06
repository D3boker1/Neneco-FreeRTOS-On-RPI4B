/**
 * @file pwm.h
 * @author Francisco Marques (fmarques_00@protonmail.com)
 * @brief Contains all the public functions and data available for PWM module.
 * @version 0.1
 * @date 2022-01-03
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef _PWM_H_
#define _PWM_H_

#include "gpio.h"
#include <stdint.h>
#include <stddef.h>
#include "errono.h"

#define PWM0_BASE  (0xfe20c000U) /**< PWM0*/

#define PWM_CTL   (*(volatile unsigned int *)(PWM0_BASE + 0x00U)) /**< PWM control*/
#define PWMEN1  0 /**< Channel 1 enable*/
#define MODE1   1 /**< Channel 1 mode*/
#define RPTL1   2 /**< Channel 1 Repeat Last Data*/
#define SBIT1   3 /**< Channel 1 Silence Bit*/
#define POLA1   4 /**< Channel 1 Polarity*/
#define USEF1   5 /**< Channel 1 Use FIFO*/
#define CLRF    6 /**< Clear FIFO*/
#define MSEN1   7 /**< Channel 1 M/S Enable*/

#define PWM_STA   (*(volatile unsigned int *)(PWM0_BASE + 0x04U)) /**< PWM Status*/
#define STA1 9  /**< Channel 1 State*/
#define STA2 10 /**< Channel 2 State*/ 

#define PWM_RNG1   (*(volatile unsigned int *)(PWM0_BASE + 0x10U)) /**< PWM Channel 1 Range*/
#define PWM_DAT1   (*(volatile unsigned int *)(PWM0_BASE + 0x14U)) /**< PWM Channel 1 Data*/
//#define PWM_RNG2   (*(volatile unsigned int *)(PWM0_BASE + 0x20U)) /**< PWM Channel 2 Range*/
//#define PWM_DAT2   (*(volatile unsigned int *)(PWM0_BASE + 0x24U)) /**< PWM Channel 2 Data*/

typedef enum PWM{ PWM0 = 0, PWM1}PWM_t;
typedef enum PWM_channel{ PWM_CHANNEL_1 = 0, PWM_CHANNEL_2}PWM_channel_t;

uint8_t pwm_init(PWM_t pwmx, PWM_channel_t channelx, uint32_t range, uint32_t duty_cycle);

uint8_t pwm_start(PWM_t pwmx, PWM_channel_t channelx);

uint8_t pwm_stop(PWM_t pwmx, PWM_channel_t channelx);

uint8_t pwm_set_range(PWM_t pwmx, PWM_channel_t channelx, uint32_t range);

uint8_t pwm_set_dt(PWM_t pwmx, PWM_channel_t channelx, uint32_t duty_cycle);

uint8_t pwm_status(PWM_t pwmx, PWM_channel_t channelx);


#endif // _PWM_H