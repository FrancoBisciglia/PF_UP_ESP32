/*

    CO2 sensor library

*/

#ifndef CO2_SENSOR_H_   /* Include guard */
#define CO2_SENSOR_H_

#ifdef __cplusplus
extern "C" {
#endif

/*==================[INCLUDES]=============================================*/

#include <stdio.h>
#include "esp_err.h"
#include "driver/gpio.h"

/*==================[DEFINES AND MACROS]=====================================*/

/**
 *  Código de error que se le carga al nivel de CO2 si se detecta algun error
 *  en el proceso de sensado.
 */
#define CO2_SENSOR_MEASURE_ERROR -300 

typedef gpio_num_t CO2_sensor_pwm_pin_t;
typedef float CO2_sensor_ppm_t;

/**
 *  @brief  Puntero a función que será utilizado para ejecutar la función que se pase
 *          como callback cuando finalice una nueva conversión del sensor.
 */
typedef void (*CO2SensorCallbackFunction)(void *pvParameters);

/*==================[EXTERNAL DATA DECLARATION]==============================*/

/*==================[EXTERNAL FUNCTIONS DECLARATION]=========================*/

esp_err_t CO2_sensor_init(CO2_sensor_pwm_pin_t CO2_sens_pwm_pin);
esp_err_t CO2_sensor_get_CO2(CO2_sensor_ppm_t *CO2_value_buffer);
bool CO2_sensor_is_warming_up(void);
void CO2_sensor_callback_function_on_new_measurment(CO2SensorCallbackFunction callback_function);

/*==================[END OF FILE]============================================*/

#ifdef __cplusplus
}
#endif

#endif // CO2_SENSOR_H_