/**
 * @file DHT11_SENSOR.c
 * @author Franco Bisciglia, David Kündinger
 * @brief   Librería que actúa como interfaz entre el driver del sensor DHT11 de temperatura y humedad relativa y la aplicación.
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

#include "dht.h"
#include "DHT11_SENSOR.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

//==================================| MACROS AND TYPDEF |==================================//

//==================================| INTERNAL DATA DEFINITION |==================================//

/* Tag para imprimir información en el LOG. */
static const char *TAG = "DHT11_SENSOR_LIBRARY";

/* Handle de la tarea de obtención de datos del sensor DHT11. */
static TaskHandle_t xDHT11TaskHandle = NULL;

/* Puntero a función que apuntará a la función callback pasada como argumento en la función de configuración de callback. */
DHT11SensorCallbackFunction DHT11Callback = NULL;

/* Variable donde se guarda el valor de temperatura medido. */
static DHT11_sensor_temp_t DHT11_temp_value = 0;

/* Variable donde se guarda el valor de humedad relativa medido. */
static DHT11_sensor_hum_t DHT11_hum_value = 0;

/* Variable que representa el pin GPIO al cual está conectado en sensor DHT11. */
static DHT11_sensor_data_pin_t DHT11_SENSOR_DATA_PIN;

//==================================| EXTERNAL DATA DEFINITION |==================================//

//==================================| INTERNAL FUNCTIONS DECLARATION |==================================//

static void vTaskGetTempAndHum(void *pvParameters);

//==================================| INTERNAL FUNCTIONS DEFINITION |==================================//

/**
 * @brief   Tarea encargada de obtener los valores de temperatura y humedad relativa desde el sensor DHT11.
 * 
 * @param pvParameters  Parámetros pasados a la tarea en su creación.
 */
static void vTaskGetTempAndHum(void *pvParameters)
{
    while (1) {

        /**
         *  Se obtiene el valor de temperatura y humedad relativa desde el sensor DHT11.
         */
        if(dht_read_float_data(DHT_TYPE_DHT11, DHT11_SENSOR_DATA_PIN, &DHT11_hum_value, &DHT11_temp_value) != ESP_OK)
        {
            /**
             *  En caso de error de medición del sensor, cargamos a la variable de temperatura
             *  y de humedad el valor definido para detección de error de forma externa a la librería.
             */
            DHT11_hum_value = DHT11_MEASURE_ERROR;
            DHT11_temp_value = DHT11_MEASURE_ERROR;
            ESP_LOGE(TAG, "Failed to get temp and hum.");
        }


        /**
         *  Se ejecuta la función callback configurada.
         * 
         *  NOTA: VER SI SE PUEDE MEJORAR PARA QUE SOLO VUELVA A MANDAR EL NOTIFY
         *  SI LA TAREA A LA QUE HAY QUE NOTIFICAR LEYÓ EL ÚLTIMO DATO. ESTO PODRIA
         *  HACERSE CON UNA SIMPLE BANDERA QUE SE ACTIVA AL LLAMAR A LA FUNCIÓN DE LEER EL DATO.
         */
        if(DHT11Callback != NULL)
        {
            DHT11Callback(NULL);
        }
        
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

//==================================| EXTERNAL FUNCTIONS DEFINITION |==================================//

/**
 * @brief   Función para inicializar el sensor de temperatura y humedad relativa DHT11.
 * 
 * @param DHT11_sens_data_pin    Pin de datos del sensor.
 * @return esp_err_t 
 */
esp_err_t DTH11_sensor_init(DHT11_sensor_data_pin_t DHT11_sens_data_pin)
{
    /**
     *  Se guarda el GPIO al cual está conectado el pin de datos del sensor.
     */
    DHT11_SENSOR_DATA_PIN = DHT11_sens_data_pin;



    //========================| CONFIGURACIÓN DE GPIO |===========================//

    /* Variable donde se definen las configuraciones para el GPIO */
    gpio_config_t pGPIOConfig;

    /* Se define mediante mascara de bits el GPIO que configuraremos */
    pGPIOConfig.pin_bit_mask = (1ULL << DHT11_sens_data_pin);
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



    //========================| CREACIÓN DE TAREA |===========================//

    /**
     *  Se crea la tarea encargada de obtener el valor de temperatura y humedad relativa 
     *  del sensor DHT11.
     * 
     *  Se le da un nivel de prioridad bajo considerando que la dinámica de la evolución
     *  de la temperatura y humedad ambiente es muy lenta, y en la tarea solamente se obtiene 
     *  el valor de temperatura y humedad y se lo guarda en una variable interna.
     */
    if(xDHT11TaskHandle == NULL)
    {
        xTaskCreate(
            vTaskGetTempAndHum,
            "vTaskGetTempAndHum",
            2048,
            NULL,
            2,
            &xDHT11TaskHandle);
        
        /**
         *  En caso de que el handle sea NULL, implica que no se pudo crear la tarea, y se retorna con error.
         */
        if(xDHT11TaskHandle == NULL)
        {
            ESP_LOGE(TAG, "Failed to create vTaskGetTempAndHum task.");
            return ESP_FAIL;
        }
    }

    return ESP_OK;
}



/**
 * @brief   Función para guardar en la variable pasada como argumento el valor de temperatura
 *          obtenido del sensor DHT11.
 * 
 * @param DHT11_temp_value_buffer     Variable donde se guardará el valor de temperatura obtenido.
 * @return esp_err_t 
 */
esp_err_t DHT11_getTemp(DHT11_sensor_temp_t *DHT11_temp_value_buffer)
{
    /**
     *  En caso de que se haya producido un error al sensar, se retorna
     *  ESP_FAIL para indicar la presencia de dicho error.
     */
    if(DHT11_temp_value == DHT11_MEASURE_ERROR)
    {
        return ESP_FAIL;
    }

    *DHT11_temp_value_buffer = DHT11_temp_value;

    return ESP_OK;
}



/**
 * @brief   Función para guardar en la variable pasada como argumento el valor de humedad relativa
 *          obtenido del sensor DHT11.
 * 
 * @param DHT11_hum_value_buffer     Variable donde se guardará el valor de humedad relativa obtenido.
 * @return esp_err_t 
 */
esp_err_t DHT11_getHum(DHT11_sensor_hum_t *DHT11_hum_value_buffer)
{
    /**
     *  En caso de que se haya producido un error al sensar, se retorna
     *  ESP_FAIL para indicar la presencia de dicho error.
     */
    if(DHT11_hum_value == DHT11_MEASURE_ERROR)
    {
        return ESP_FAIL;
    }

    *DHT11_hum_value_buffer = DHT11_hum_value;

    return ESP_OK;
}



/**
 * @brief   Función para configurar que, al finalizarse una nueva medición del sensor,
 *          se ejecute la función que se pasa como argumento.
 * 
 * @param callback_function    Función a ejecutar al finalizar una medición del sensor.
 */
void DHT11_callback_function_on_new_measurment(DHT11SensorCallbackFunction callback_function)
{
    DHT11Callback = callback_function;
}