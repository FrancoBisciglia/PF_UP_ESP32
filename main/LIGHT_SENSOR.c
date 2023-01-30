/**
 * @file LIGHT_SENSOR.c
 * @author Franco Bisciglia, David Kündinger
 * @brief   Librería utilizada para obtener el dato de trigger del sensor de luz.
 * @version 0.1
 * @date 2022-12-29
 * 
 * @copyright Copyright (c) 2022
 * 
 */



/**
 * =================================================| EXPLICACIÓN DE LIBRERÍA |=================================================
 *  
 *          
 */



//==================================| INCLUDES |==================================//

#include <stdio.h>
#include <stdbool.h>

#include "LIGHT_SENSOR.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"

//==================================| MACROS AND TYPDEF |==================================//

//==================================| INTERNAL DATA DEFINITION |==================================//

/* Tag para imprimir información en el LOG. */
static const char *TAG = "LIGHT_SENSOR_LIBRARY";

/* Variable que representa el pin GPIO al cual está conectado en sensor DHT11. */
static light_sensor_data_pin_t LIGHT_SENSOR_DATA_PIN;

//==================================| EXTERNAL DATA DEFINITION |==================================//

//==================================| INTERNAL FUNCTIONS DECLARATION |==================================//

//==================================| INTERNAL FUNCTIONS DEFINITION |==================================//

//==================================| EXTERNAL FUNCTIONS DEFINITION |==================================//

/**
 * @brief   Función para inicializar el sensor de luz.
 * 
 * @param light_sens_data_pin    Pin de datos del sensor.
 * @return esp_err_t 
 */
esp_err_t light_sensor_init(light_sensor_data_pin_t light_sens_data_pin)
{
    /**
     *  Se guarda el GPIO al cual está conectado el pin de datos del sensor.
     */
    LIGHT_SENSOR_DATA_PIN = light_sens_data_pin;



    //========================| CONFIGURACIÓN DE GPIO |===========================//

    /* Variable donde se definen las configuraciones para el GPIO */
    gpio_config_t pGPIOConfig;

    /* Se define mediante mascara de bits el GPIO que configuraremos */
    pGPIOConfig.pin_bit_mask = (1ULL << light_sens_data_pin);
    /* Se define si el pin es I u O (input en este caso) */
    pGPIOConfig.mode = GPIO_MODE_DEF_INPUT;
    /* Habilitamos o deshabilitamos la resistencia interna de pull-up (deshabilitada en este caso) */
    pGPIOConfig.pull_up_en = GPIO_PULLUP_DISABLE;
    /* Habilitamos o deshabilitamos la resistencia interna de pull-down (deshabilitada en este caso) */
    pGPIOConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
    /* Definimos si habilitamos la interrupción, y si es asi, de qué tipo (deshabilitada en este caso) */
    pGPIOConfig.intr_type = GPIO_INTR_DISABLE;
    
    /**
     *  Función para configurar un pin GPIO. 
     *  Se le pasa como parametro un puntero a la variable configurada anteriormente
     */
    ESP_RETURN_ON_ERROR(gpio_config(&pGPIOConfig), TAG, "Failed to load gpio config.");

    return ESP_OK;
}



/**
 * @brief   Función para detectar si se superó el nivel de trigger de luz ajustado en el sensor.
 * 
 * @return true     Se superó el nivel de trigger.
 * @return false    No se superó el nivel de trigger.
 */
bool light_trigger(void)
{
    return gpio_get_level(LIGHT_SENSOR_DATA_PIN);
}