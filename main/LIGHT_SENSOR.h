/*

    Light sensor library

*/

#ifndef LIGHT_SENSOR_H_   /* Include guard */
#define LIGHT_SENSOR_H_

#ifdef __cplusplus
extern "C" {
#endif

/*==================[INCLUDES]=============================================*/

#include <stdio.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"

/*==================[DEFINES AND MACROS]=====================================*/

typedef gpio_num_t light_sensor_data_pin_t;

/*==================[EXTERNAL DATA DECLARATION]==============================*/

/*==================[EXTERNAL FUNCTIONS DECLARATION]=========================*/

esp_err_t light_sensor_init(light_sensor_data_pin_t light_sens_data_pin);
bool light_trigger(void);

/*==================[END OF FILE]============================================*/

#ifdef __cplusplus
}
#endif

#endif // LIGHT_SENSOR_H_