#include "ELM327.h"

static uint8_t elm327_initialized = 0;
static COMMAND_t next_command =  ENGINE_TEMP;

/**
 * @brief List of commands to read OBDII network
 * 
 * ATI: Model and version of ELM327
 * AT L1: Enable line
 * 0105: Engine coolant temperature
 * 010C: Engine speed
 * 010D: Vehicle speed
 * 011F: Run time since engine start
 * 012F: Fuel Tank Level Input
 * 0133: Absolute Barometric Pressure
 * 0146: Ambient air temperature
 * 0151: Fuel Type
 */
const char commands_list[NUM_COMMANDS][COMMAND_MAX_LEN] = {"ATI", "ATL1", "0105", "010C", "010D", "011F", "012F", "0133", "0146", "0151"};


int16_t elm327_data_init(DataELM327_t* elm327_data){

    int ret = -ENOSYS;

    if(elm327_data == NULL){
        ret = -EBADSLT;
    }
    else{
        elm327_data->airTemp = 0;
        elm327_data->barPressure = 0;
        elm327_data->coolantTemperature = 0;
        elm327_data->engineSpeed = 0;
        elm327_data->fuelLevel = 0;
        elm327_data->fuelType = 0;
        elm327_data->runTime = 0;
        elm327_data->vehicleSpeed = 0;
        ret = VALID;
        elm327_initialized = 1;
    }

    return ret;

}

int16_t elm327_execute_command(COMMAND_t command_to_exe){
    int ret = -ENOSYS;

    if(elm327_initialized == 1){
        uart_puts(commands_list[command_to_exe]);
        ret = VALID;
    }

    return ret;
}

int16_t elm327_execute_next(DataELM327_t* elm327_data){
    /**< to not produce unused warning*/
    (void) elm327_data;

    int ret = -ENOSYS;

    if(elm327_initialized == 1){
        uart_puts(commands_list[next_command++]);
        if(next_command == ATI)
            next_command = ENGINE_TEMP;
        ret = VALID;
    }

    return ret;
}