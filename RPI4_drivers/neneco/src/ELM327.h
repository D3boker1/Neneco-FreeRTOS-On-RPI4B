/**
 * @file ELM327.h
 * @author Francisco Marques (fmarques_00@protonmail.com)
 * @brief Driver to interact with ELM327 devices
 * @version 0.1
 * @date 2022-01-22
 * 
 * @copyright Copyright (c) 2022
 * 
 * This driver offers a set of function to allow the communication with ELM327 devices. 
 * In version v0.1 only 10 commands are supported, and they are specified in COMMAND enumeration.
 * This module relies on uart driver, and the uart initialization is not made inside it. So
 * to use this module make sure you have the uart initialized and working. 
 */

#ifndef _ELM327_H_
#define _ELM327_H_

#include "errono.h"
#include <stddef.h>
#include <stdint.h>
#include "uart.h"

/**< It is not being used yet (v0.1)*/
#define ELM_BAUD_RATE 38400

#define NUM_COMMANDS 10
#define COMMAND_MAX_LEN 8

typedef enum COMMAND{ATI=0, ATL1, ENGINE_TEMP, ENGINE_SPEED, VEHICLE_SPEED, RUN_TIME, FUEL_LEVEL, BAR_PRESSURE, AIR_TEMP, FUEL_TYPE}COMMAND_t;

typedef struct{
    int coolantTemperature; //0105
    uint32_t engineSpeed;   //010C
    uint8_t vehicleSpeed;   //010D
    uint64_t runTime;       //011F
    uint8_t fuelLevel;      //012F
    uint8_t barPressure;    //0133
    int airTemp;            //0146
    uint8_t fuelType;       //0151
    
}DataELM327_t;

/**< ELM327 data struct*/
extern DataELM327_t elm327_info;

/**
 * @brief Initialize the elm327 device
 * 
 * @param elm327_data data structure with data to be read from elm327 device.
 * @return int16_t -EBADSLT if the data structure passed by argument is a null pointer. VALID in case of success.
 */
int16_t elm327_data_init(DataELM327_t* elm327_data);

/**
 * @brief Executes a specific command
 * 
 * @param command_to_exe Command to be executed
 * @return int16_t 
 */
int16_t elm327_execute_command(COMMAND_t command_to_exe);

/**
 * @brief Execute the next command from the command list.
 * 
 * @param elm327_data data structure of elm327 device.
 * @return int16_t 
 */
int16_t elm327_execute_next(DataELM327_t* elm327_data);

#endif //_ELM327_H_