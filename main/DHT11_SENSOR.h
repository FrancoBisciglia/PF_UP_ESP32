/*

    DHT11 sensor library

*/

#ifndef DHT11_SENSOR_H_   /* Include guard */
#define DHT11_SENSOR_H_

#ifdef __cplusplus
extern "C" {
#endif

/*==================[INCLUDES]=============================================*/

#include <stdio.h>
#include "esp_err.h"
#include "driver/gpio.h"

/*==================[DEFINES AND MACROS]=====================================*/

#define DHT11_MEASURE_ERROR -200 

typedef gpio_num_t DHT11_sensor_data_pin_t;
typedef float DHT11_sensor_temp_t;
typedef float DHT11_sensor_hum_t;

/**
 *  @brief  Puntero a función que será utilizado para ejecutar la función que se pase
 *          como callback cuando finalice una nueva conversión del sensor.
 */
typedef void (*DHT11SensorCallbackFunction)(void *pvParameters);

/*==================[EXTERNAL DATA DECLARATION]==============================*/

/*==================[EXTERNAL FUNCTIONS DECLARATION]=========================*/

esp_err_t DTH11_sensor_init(DHT11_sensor_data_pin_t DHT11_sens_data_pin);
esp_err_t DHT11_getTemp(DHT11_sensor_temp_t *DHT11_temp_value_buffer);
esp_err_t DHT11_getHum(DHT11_sensor_hum_t *DHT11_hum_value_buffer);
void DHT11_callback_function_on_new_measurment(DHT11SensorCallbackFunction callback_function);

/*==================[END OF FILE]============================================*/

#ifdef __cplusplus
}
#endif

#endif // DHT11_SENSOR_H_