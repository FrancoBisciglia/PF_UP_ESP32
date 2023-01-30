/**
 * @file AUXILIARES_ALGORITMO_CONTROL_BOMBEO_SOLUCION.c
 * @author Franco Bisciglia, David Kündinger
 * @brief   Funcionalidades del algoritmo de control del bombeo de la solución nutritiva, como funciones callback o de inicialización.
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
#include "FLOW_SENSOR.h"
#include "MEF_ALGORITMO_CONTROL_BOMBEO_SOLUCION.h"
#include "AUXILIARES_ALGORITMO_CONTROL_BOMBEO_SOLUCION.h"

//==================================| MACROS AND TYPDEF |==================================//

//==================================| INTERNAL DATA DEFINITION |==================================//

/* Tag para imprimir información en el LOG. */
static const char *aux_control_bombeo_tag = "AUXILIAR_CONTROL_BOMBEO_SOLUCION";

/* Handle del cliente MQTT. */
static esp_mqtt_client_handle_t Cliente_MQTT = NULL;

/* Handle del timer utilizado para control de encendido y apagado de la bomba de solución. */
static TimerHandle_t xTimerBomba = NULL;

//==================================| EXTERNAL DATA DEFINITION |==================================//

//==================================| INTERNAL FUNCTIONS DECLARATION |==================================//

void vBombaTimerCallback( TimerHandle_t pxTimer );
void CallbackManualMode(void *pvParameters);
void CallbackManualModeNewActuatorState(void *pvParameters);
void CallbackNewPumpOnTime(void *pvParameters);
void CallbackNewPumpOffTime(void *pvParameters);

//==================================| INTERNAL FUNCTIONS DEFINITION |==================================//

/**
 * @brief   Función de callback del timer de control de tiempo de encendido y apagado de la bomba
 *          de solución.
 * 
 * @param pxTimer   Handle del timer para el cual se cumplió el timeout.
 */
static void vBombaTimerCallback( TimerHandle_t pxTimer )
{
    /**
     *  Esta variable sirve para que, en el caso de que un llamado a "xTaskNotifyFromISR()" desbloquee
     *  una tarea de mayor prioridad que la que estaba corriendo justo antes de entrar en la rutina
     *  de interrupción, al retornar se haga un context switch a dicha tarea de mayor prioridad en vez
     *  de a la de menor prioridad (xHigherPriorityTaskWoken = pdTRUE)
     */
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    /**
     *  Se setea la bandera del timer para señalizarle a la MEF "MEFControlBombeoSoluc"
     *  que se cumplió el tiempo de encendido o apagado de la bomba de solución.
     */
    mef_bombeo_set_timer_flag_value(1);

    /**
     *  Se le envía un Task Notify a la tarea de la MEF de control de bombeo de solución.
     */
    vTaskNotifyGiveFromISR(mef_bombeo_get_task_handle(), &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}



/**
 * @brief   Función de callback que se ejecuta cuando llega un mensaje MQTT en el tópico
 *          correspondiente al modo MANUAL, para indicar si se quiere pasar a modo
 *          MANUAL o AUTOMÁTICO.
 * 
 * @param pvParameters 
 */
void CallbackManualMode(void *pvParameters)
{
    /**
     *  Se obtiene el mensaje del tópico de modo MANUAL o AUTO.
     */
    char buffer[10];
    mqtt_get_char_data_from_topic(MANUAL_MODE_MQTT_TOPIC, buffer);

    /**
     *  Dependiendo si el mensaje fue "MANUAL" o "AUTO", se setea o resetea
     *  la bandera correspondiente para señalizarle a la MEF de control de
     *  bombeo de solución que debe pasar al estado de modo MANUAL o AUTOMATICO.
     */
    if(!strcmp("MANUAL", buffer))
    {
        mef_bombeo_set_manual_mode_flag_value(1);
    }

    else if(!strcmp("AUTO", buffer))
    {
        mef_bombeo_set_manual_mode_flag_value(0);
    }

    /**
     * Se le envía un Task Notify a la tarea de la MEF de control de bombeo de solución.
     */
    xTaskNotifyGive(mef_bombeo_get_task_handle());
}



/**
 *  @brief  Función de callback que se ejecuta cuando llega un mensaje MQTT en el tópico
 *          correspondiente al estado de la bomba de solución en el modo MANUAL.
 *          Es decir, cuando estando en modo MANUAL, se quiere accionar la bomba de solución.
 * 
 * @param pvParameters 
 */
void CallbackManualModeNewActuatorState(void *pvParameters)
{
    /**
     * Se le envía un Task Notify a la tarea de la MEF de control de bombeo de solución.
     */
    xTaskNotifyGive(mef_bombeo_get_task_handle());
}



/**
 *  @brief  Función de callback que se ejecuta cuando llega un mensaje al tópico MQTT
 *          correspondiente con un nuevo valor de tiempo de encendido de la bomba
 *          de solución.
 * 
 * @param pvParameters 
 */
void CallbackNewPumpOnTime(void *pvParameters)
{
    /**
     *  Se obtiene el nuevo valor de tiempo de encendido de la bomba.
     */
    pump_time_t tiempo_on_bomba = 0;
    mqtt_get_float_data_from_topic(NEW_PUMP_ON_TIME_MQTT_TOPIC, &tiempo_on_bomba);

    ESP_LOGI(aux_control_bombeo_tag, "NUEVO TIEMPO ENCENDIDO BOMBA: %.0f", tiempo_on_bomba);

    /**
     *  Se actualiza el nuevo tiempo de encendido en la MEF.
     */
    mef_bombeo_set_pump_on_time_min(tiempo_on_bomba);
}



/**
 *  @brief  Función de callback que se ejecuta cuando llega un mensaje al tópico MQTT
 *          correspondiente con un nuevo valor de tiempo de apagado de la bomba
 *          de solución.
 * 
 * @param pvParameters 
 */
void CallbackNewPumpOffTime(void *pvParameters)
{
    /**
     *  Se obtiene el nuevo valor de tiempo de apagado de la bomba.
     */
    pump_time_t tiempo_off_bomba = 0;
    mqtt_get_float_data_from_topic(NEW_PUMP_OFF_TIME_MQTT_TOPIC, &tiempo_off_bomba);

    ESP_LOGI(aux_control_bombeo_tag, "NUEVO TIEMPO APAGADO BOMBA: %.0f", tiempo_off_bomba);

    /**
     *  Se actualiza el nuevo tiempo de apagado en la MEF.
     */
    mef_bombeo_set_pump_off_time_min(tiempo_off_bomba);
}

//==================================| EXTERNAL FUNCTIONS DEFINITION |==================================//

/**
 * @brief   Función para inicializar el módulo de funciones auxiliares del algoritmo de control de bombeo
 *          de solución. 
 * 
 * @param mqtt_client   Handle del cliente MQTT.
 * @return esp_err_t 
 */
esp_err_t aux_control_bombeo_init(esp_mqtt_client_handle_t mqtt_client)
{
    /**
     *  Copiamos el handle del cliente MQTT en la variable interna.
     */
    Cliente_MQTT = mqtt_client;
    

    //=======================| INIT TIMERS |=======================//

    /**
     *  Se inicializa el timer utilizado para el control de tiempo de encendido
     *  y apagado de la bomba de solución nutritiva.
     * 
     *  Se inicializa su período en 1 tick dado que no es relevante en su inicialización, ya que
     *  el tiempo de apertura y cierre de la válvula se asignará en la MEF cuando corresponda, pero
     *  no puede ponerse 0.
     */
    xTimerBomba = xTimerCreate("Timer Bomba Solución",       // Nombre interno que se le da al timer (no es relevante).
                              1,                                // Período del timer en ticks.
                              pdFALSE,                          // pdFALSE -> El timer NO se recarga solo al cumplirse el timeout. pdTRUE -> El timer se recarga solo al cumplirse el timeout.
                              (void *)10,                        // ID de identificación del timer.
                              vBombaTimerCallback                    // Nombre de la función de callback del timer.
    );

    /**
     *  Se verifica que se haya creado el timer correctamente.
     */
    if(xTimerBomba == NULL)
    {
        ESP_LOGE(aux_control_bombeo_tag, "FAILED TO CREATE TIMER.");
        return ESP_FAIL;
    }


    //=======================| TÓPICOS MQTT |=======================//

    /**
     *  Se inicializa el array con los tópicos MQTT a suscribirse, junto
     *  con las funciones callback correspondientes que serán ejecutadas
     *  al llegar un nuevo dato en el tópico.
     */
    mqtt_topic_t list_of_topics[] = {
        [0].topic_name = NEW_PUMP_ON_TIME_MQTT_TOPIC,
        [0].topic_function_cb = CallbackNewPumpOnTime,
        [1].topic_name = NEW_PUMP_OFF_TIME_MQTT_TOPIC,
        [1].topic_function_cb = CallbackNewPumpOffTime,
        [2].topic_name = MANUAL_MODE_MQTT_TOPIC,
        [2].topic_function_cb = CallbackManualMode,
        [3].topic_name = MANUAL_MODE_PUMP_STATE_MQTT_TOPIC,
        [3].topic_function_cb = CallbackManualModeNewActuatorState,
    };

    /**
     *  Se realiza la suscripción a los tópicos MQTT y la asignación de callbacks correspondientes.
     */
    if(mqtt_suscribe_to_topics(list_of_topics, 4, Cliente_MQTT, 0) != ESP_OK)
    {
        ESP_LOGE(aux_control_bombeo_tag, "FAILED TO SUSCRIBE TO MQTT TOPICS.");
        return ESP_FAIL;
    }

    return ESP_OK;
}



/**
 * @brief   Función que retorna el Handle del timer de control de tiempo de encendido
 *          y apagado de la bomba de solución.
 * 
 * @return TimerHandle_t    Handle del timer.
 */
TimerHandle_t aux_control_bombeo_get_timer_handle(void)
{
    return xTimerBomba;
}