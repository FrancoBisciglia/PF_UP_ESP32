/**
 * @file MEF_ALGORITMO_CONTROL_LUCES.c
 * @author Franco Bisciglia, David Kündinger
 * @brief   Implementación de las diferentes MEF's del algoritmo de control de las luces de las unidades secundarias.
 * @version 0.1
 * @date 2023-01-16
 * 
 * @copyright Copyright (c) 2023
 * 
 */

//==================================| INCLUDES |==================================//

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "MQTT_PUBL_SUSCR.h"
#include "MCP23008.h"
#include "AUXILIARES_ALGORITMO_CONTROL_LUCES.h"
#include "MEF_ALGORITMO_CONTROL_LUCES.h"

//==================================| MACROS AND TYPDEF |==================================//

//==================================| INTERNAL DATA DEFINITION |==================================//

/* Tag para imprimir información en el LOG. */
static const char *mef_luces_tag = "MEF_CONTROL_LUCES";

/* Task Handle de la tarea del algoritmo de control de las luces. */
static TaskHandle_t xMefLucesAlgoritmoControlTaskHandle = NULL;

/* Handle del cliente MQTT. */
static esp_mqtt_client_handle_t MefLucesClienteMQTT = NULL;

/* Tiempo de apagado de la bomba, en minutos. */
static pump_time_t mef_luces_tiempo_luces_off = MEF_LUCES_TIEMPO_LUCES_OFF;
/* Tiempo de encendido de la bomba, en minutos. */
static pump_time_t mef_luces_tiempo_luces_on = MEF_LUCES_TIEMPO_LUCES_ON;

/* Bandera utilizada para controlar si se está o no en modo manual en el algoritmo de control de las luces. */
static bool mef_luces_manual_mode_flag = 0;
/* Banderas utilizadas para controlar las transiciones con reset de la MEF de control de las luces. */
static bool mef_luces_reset_transition_flag_control_luces = 0;
/**
 *  Bandera utilizada para verificar si se cumplió el timeout del timer utilizado para controlar el encendido
 *  y apagado de las.
 */
static bool mef_luces_timer_finished_flag = 0;

//==================================| EXTERNAL DATA DEFINITION |==================================//

//==================================| INTERNAL FUNCTIONS DECLARATION |==================================//

static void MEFControlLuces(void);
static void vTaskLigthsControl(void *pvParameters);

//==================================| INTERNAL FUNCTIONS DEFINITION |==================================//

/**
 * @brief   Función de la MEF de control de las luces ubicadas en las distintas unidades secundarias.
 *          
 *          Se tiene un periodo compuesto por un tiempo de encendido y un tiempo de apagado de las luces,
 *          en un ciclo completo de 24 hs, es decir, si son 8 hs de luces encendidas, serán 16 hs de luces
 *          apagadas.
 */
static void MEFControlLuces(void)
{
    /**
     * Variable que representa el estado de la MEF de control de las luces.
     */
    static estado_MEF_control_luces_t est_MEF_control_luces = ESPERA_ILUMINACION_CULTIVOS;

    /**
     *  Se controla si se debe hacer una transición con reset, caso en el cual se vuelve al estado
     *  de luces apagadas y se paran los timers correspondientes.
     */
    if(mef_luces_reset_transition_flag_control_luces)
    {
        est_MEF_control_luces = ESPERA_ILUMINACION_CULTIVOS;

        mef_luces_reset_transition_flag_control_luces = 0;

        xTimerStop(aux_control_luces_get_timer_handle(), 0);
        mef_luces_timer_finished_flag = 0;

        set_relay_state(LUCES, OFF);
        /**
         *  Se publica el nuevo estado de las luces en el tópico MQTT correspondiente.
         */
        if(mqtt_check_connection())
        {
            char buffer[10];
            snprintf(buffer, sizeof(buffer), "%s", "OFF");
            esp_mqtt_client_publish(Cliente_MQTT, LIGHTS_STATE_MQTT_TOPIC, buffer, 0, 0, 0);
        }

        ESP_LOGW(mef_luces_tag, "LUCES APAGADAS");
    }


    switch(est_MEF_control_luces)
    {
        
    case ESPERA_ILUMINACION_CULTIVOS:

        /**
         *  Cuando se cumpla el timeout del timer, se cambia al estado con las luces encendidas, y se carga
         *  en el timer el tiempo de encendido de las luces.
         */
        if(mef_luces_timer_finished_flag)
        {
            mef_luces_timer_finished_flag = 0;
            xTimerChangePeriod(aux_control_luces_get_timer_handle(), pdMS_TO_TICKS(HOURS_TO_MS * mef_luces_tiempo_luces_on), 0);
            xTimerReset(aux_control_luces_get_timer_handle(), 0);

            set_relay_state(LUCES, ON);
            /**
             *  Se publica el nuevo estado de las luces en el tópico MQTT correspondiente.
             */
            if(mqtt_check_connection())
            {
                char buffer[10];
                snprintf(buffer, sizeof(buffer), "%s", "ON");
                esp_mqtt_client_publish(Cliente_MQTT, LIGHTS_STATE_MQTT_TOPIC, buffer, 0, 0, 0);
            }

            ESP_LOGW(mef_luces_tag, "LUCES ENCENDIDAS");

            est_MEF_control_luces = ILUMINACION_CULTIVOS;
        }

        break;


    case ILUMINACION_CULTIVOS:

        /**
         *  Cuando se cumpla el timeout del timer, se cambia al estado con las luces apagadas, y se carga
         *  en el timer el tiempo de apagado de las luces.
         */
        if(mef_luces_timer_finished_flag)
        {
            mef_luces_timer_finished_flag = 0;
            xTimerChangePeriod(aux_control_luces_get_timer_handle(), pdMS_TO_TICKS(HOURS_TO_MS * mef_luces_tiempo_luces_off), 0);
            xTimerReset(aux_control_luces_get_timer_handle(), 0);

            set_relay_state(LUCES, OFF);
            /**
             *  Se publica el nuevo estado de las luces en el tópico MQTT correspondiente.
             */
            if(mqtt_check_connection())
            {
                char buffer[10];
                snprintf(buffer, sizeof(buffer), "%s", "OFF");
                esp_mqtt_client_publish(Cliente_MQTT, LIGHTS_STATE_MQTT_TOPIC, buffer, 0, 0, 0);
            }

            ESP_LOGW(mef_luces_tag, "LUCES APAGADAS");

            est_MEF_control_luces = ESPERA_ILUMINACION_CULTIVOS;
        }

        break;
    }
}



/**
 * @brief   Tarea que representa la MEF principal (de mayor jerarquía) del algoritmo de 
 *          control de las luces de las unidades secundarias, alternando entre el modo automatico
 *          o manual de control según se requiera.
 * 
 * @param pvParameters  Parámetro que se le pasa a la tarea en su creación.
 */
static void vTaskLigthsControl(void *pvParameters)
{
    /**
     * Variable que representa el estado de la MEF de jerarquía superior del algoritmo de las luces.
     */
    static estado_MEF_principal_control_luces_t est_MEF_principal = ALGORITMO_CONTROL_LUCES;

    while(1)
    {
        /**
         *  Se realiza un Notify Take a la espera de señales que indiquen:
         *  
         *  -Que se debe pasar a modo MANUAL o modo AUTO.
         *  -Que estando en modo MANUAL, se deba cambiar el estado de las luces.
         *  -Que se cumplió el timeout del timer de control del tiempo de encendido o apagado de las luces.
         * 
         *  Además, se le coloca un timeout para evaluar las transiciones de las MEFs periódicamente, en caso
         *  de que no llegue ninguna de las señales mencionadas.
         */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));


        switch(est_MEF_principal)
        {
        
        case ALGORITMO_CONTROL_LUCES:

            /**
             *  En caso de que se levante la bandera de modo MANUAL, se debe transicionar a dicho estado,
             *  en donde el accionamiento de las luces será manejado por el usuario vía mensajes MQTT.
             */
            if(mef_luces_manual_mode_flag)
            {
                est_MEF_principal = MODO_MANUAL;
                mef_luces_reset_transition_flag_control_luces = 1;
            }

            MEFControlLuces();

            break;


        case MODO_MANUAL:

            /**
             *  En caso de que se baje la bandera de modo MANUAL, se debe transicionar nuevamente al estado
             *  de modo AUTOMATICO, en donde se controla el encendido y apagado de las luces por tiempos, y
             *  se inicia el timer de control de tiempo de encendido y apagado de las luces, para que continue
             *  en el tiempo que se quedó.
             */
            if(!mef_luces_manual_mode_flag)
            {
                est_MEF_principal = ALGORITMO_CONTROL_LUCES;

                xTimerStart(aux_control_luces_get_timer_handle(), 0);

                break;
            }

            /**
             *  Se obtiene el nuevo estado en el que deben estar las luces y se acciona
             *  el relé correspondiente.
             */
            float manual_mode_luces_state = -1;
            mqtt_get_float_data_from_topic(MANUAL_MODE_LIGHTS_STATE_MQTT_TOPIC, &manual_mode_luces_state);

            if(manual_mode_luces_state == 0 || manual_mode_luces_state == 1)
            {
                set_relay_state(LUCES, manual_mode_luces_state);

                /**
                 *  Se publica el nuevo estado de las luces en el tópico MQTT correspondiente.
                 */
                if(mqtt_check_connection())
                {
                    char buffer[10];

                    if(manual_mode_luces_state == 0)
                    {
                        snprintf(buffer, sizeof(buffer), "%s", "OFF");    
                    }

                    else if(manual_mode_luces_state == 1)
                    {
                        snprintf(buffer, sizeof(buffer), "%s", "ON");
                    }
                    
                    esp_mqtt_client_publish(Cliente_MQTT, LIGHTS_STATE_MQTT_TOPIC, buffer, 0, 0, 0);
                }

                ESP_LOGW(mef_luces_tag, "MANUAL MODE LUCES: %.0f", manual_mode_luces_state);
            }

            break;
        }
    }
}

//==================================| EXTERNAL FUNCTIONS DEFINITION |==================================//

/**
 * @brief   Función para inicializar el módulo de MEFs del algoritmo de control de las luces de las unidades
 *          secundarias. 
 * 
 * @param mqtt_client   Handle del cliente MQTT.
 * @return esp_err_t 
 */
esp_err_t mef_luces_init(esp_mqtt_client_handle_t mqtt_client)
{
    /**
     *  Copiamos el handle del cliente MQTT en la variable interna.
     */
    MefLucesClienteMQTT = mqtt_client;


    //=======================| CREACION TAREAS |=======================//
    
    /**
     *  Se crea la tarea mediante la cual se controlará la transicion de las
     *  MEFs del algoritmo de control de luces.
     */
    if(xMefLucesAlgoritmoControlTaskHandle == NULL)
    {
        xTaskCreate(
            vTaskLigthsControl,
            "vTaskLigthsControl",
            4096,
            NULL,
            2,
            &xMefLucesAlgoritmoControlTaskHandle);
        
        /**
         *  En caso de que el handle sea NULL, implica que no se pudo crear la tarea, y se retorna con error.
         */
        if(xMefLucesAlgoritmoControlTaskHandle == NULL)
        {
            ESP_LOGE(mef_luces_tag, "Failed to create vTaskLigthsControl task.");
            return ESP_FAIL;
        }
    }
    
    return ESP_OK;
}



/**
 * @brief   Función que devuelve el Task Handle de la tarea principal del algoritmo de control de las luces.
 * 
 * @return TaskHandle_t Task Handle de la tarea.
 */
TaskHandle_t mef_luces_get_task_handle(void)
{
    return xMefLucesAlgoritmoControlTaskHandle;
}



/**
 * @brief   Función para establecer un nuevo tiempo de encendido de la bomba de solución.
 * 
 * @param nuevo_tiempo_bomba_on   Tiempo de encendido de la bomba.
 */
void mef_luces_set_lights_on_time_hours(light_time_t tiempo_luces_on)
{
    mef_luces_tiempo_luces_on = tiempo_luces_on;

    /**
     *  El tiempo de apagado de las luces se calcula teniendo en cuenta un ciclo completo
     *  de 24 hs.
     */
    mef_luces_tiempo_luces_off = 24 - mef_luces_tiempo_luces_on;
}



/**
 * @brief   Función para cambiar el estado de la bandera de modo MANUAL, utilizada por
 *          la MEF para cambiar entre estado de modo MANUAL y AUTOMATICO.
 * 
 * @param manual_mode_flag_state    Estado de la bandera.
 */
void mef_luces_set_manual_mode_flag_value(bool manual_mode_flag_state)
{
    mef_luces_manual_mode_flag = manual_mode_flag_state;
}



/**
 * @brief   Función para cambiar el estado de la bandera de timeout del timer mediante
 *          el que se controla el tiempo de encendido y apagado de las luces.
 * 
 * @param timer_flag_state    Estado de la bandera.
 */
void mef_luces_set_timer_flag_value(bool timer_flag_state)
{
    mef_luces_timer_finished_flag = timer_flag_state;
}