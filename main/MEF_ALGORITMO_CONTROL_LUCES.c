/**
 * @file MEF_ALGORITMO_CONTROL_BOMBEO_SOLUCION.c
 * @author Franco Bisciglia, David Kündinger
 * @brief   Implementación de las diferentes MEF's del algoritmo de control del bombeo de la solución.
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
#include "MCP23008.h"
#include "ALARMAS_USUARIO.h"
#include "APP_LEVEL_SENSOR.h"
#include "AUXILIARES_ALGORITMO_CONTROL_BOMBEO_SOLUCION.h"
#include "MEF_ALGORITMO_CONTROL_BOMBEO_SOLUCION.h"

//==================================| MACROS AND TYPDEF |==================================//

//==================================| INTERNAL DATA DEFINITION |==================================//

/* Tag para imprimir información en el LOG. */
static const char *mef_bombeo_tag = "MEF_CONTROL_BOMBEO_SOLUCION";

/* Task Handle de la tarea del algoritmo de control de bombeo de solución. */
static TaskHandle_t xMefBombeoAlgoritmoControlTaskHandle = NULL;

/* Handle del timer utilizado para temporizar el control de flujo de solución en los canales de cultivo. */
static TimerHandle_t xTimerSensorFlujo = NULL;

/* Handle del cliente MQTT. */
static esp_mqtt_client_handle_t MefBombeoClienteMQTT = NULL;

/* Tiempo de apagado de la bomba, en minutos. */
static pump_time_t mef_bombeo_tiempo_bomba_off = MEF_BOMBEO_TIEMPO_BOMBA_OFF;
/* Tiempo de encendido de la bomba, en minutos. */
static pump_time_t mef_bombeo_tiempo_bomba_on = MEF_BOMBEO_TIEMPO_BOMBA_ON;
/* Periodo de tiempo de control de flujo en los canales */
unsigned int mef_bombeo_tiempo_control_sensor_flujo = MEF_BOMBEO_TIEMPO_CONTROL_SENSOR_FLUJO;

/* Bandera utilizada para controlar si se está o no en modo manual en el algoritmo de control de bombeo de solución. */
static bool mef_bombeo_manual_mode_flag = 0;
/* Banderas utilizadas para controlar las transiciones con reset de las MEFs de control de pH y de control de las válvulas. */
static bool mef_bombeo_reset_transition_flag_control_bombeo_solucion = 0;
/**
 *  Bandera utilizada para verificar si se cumplió el timeout del timer utilizado para controlar la apertura y cierre
 *  de las válvulas de control de pH.
 */
static bool mef_bombeo_timer_finished_flag = 0;
/**
 *  Bandera utilizada para verificar si se cumplió el timeout del timer utilizado para controlar si hay presencia
 *  de flujo de solución cuando se está en el estado de la MEF de bombeo de solución.
 */
static bool mef_bombeo_timer_flow_control_flag = 0;

//==================================| EXTERNAL DATA DEFINITION |==================================//

//==================================| INTERNAL FUNCTIONS DECLARATION |==================================//

void MEFControlBombeoSoluc(void);
void vTaskSolutionPumpControl(void *pvParameters);

//==================================| INTERNAL FUNCTIONS DEFINITION |==================================//

/**
 * @brief   Función de callback del timer de control de presencia de flujo en el canal de cultivo.
 * 
 * @param pxTimer   Handle del timer para el cual se cumplió el timeout.
 */
static void vSensorFlujoTimerCallback( TimerHandle_t pxTimer )
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
     *  que se debe volver a controlar si hay flujo de solución en los canales.
     */
    mef_bombeo_timer_flow_control_flag = 1;

    /**
     *  Se le envía un Task Notify a la tarea de la MEF de control de bombeo de solución.
     */
    vTaskNotifyGiveFromISR(xMefBombeoAlgoritmoControlTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}



/**
 * @brief   Función de la MEF de control del bombeo de solución nutritiva desde el tanque principal de
 *          almacenamiento hacia los cultivos.
 *          
 *          Se tiene un periodo compuesto por un tiempo de encendido y un tiempo de apagado de la bomba.
 */
void MEFControlBombeoSoluc(void)
{
    /**
     * Variable que representa el estado de la MEF de control de bombeo de solución.
     */
    static estado_MEF_control_bombeo_soluc_t est_MEF_control_bombeo_soluc = ESPERA_BOMBEO;

    /**
     *  Se controla si se debe hacer una transición con reset, caso en el cual se vuelve al estado
     *  de bomba apagada y se paran los timers correspondiente.
     */
    if(mef_bombeo_reset_transition_flag_control_bombeo_solucion)
    {
        est_MEF_control_bombeo_soluc = ESPERA_BOMBEO;

        mef_bombeo_reset_transition_flag_control_bombeo_solucion = 0;

        xTimerStop(aux_control_bombeo_get_timer_handle(), 0);
        mef_bombeo_timer_finished_flag = 0;

        xTimerStop(xTimerSensorFlujo, 0);
        mef_bombeo_timer_flow_control_flag = 0;

        set_relay_state(BOMBA, OFF);
        /**
         *  Se publica el nuevo estado de la bomba en el tópico MQTT correspondiente.
         */
        if(mqtt_check_connection())
        {
            char buffer[10];
            snprintf(buffer, sizeof(buffer), "%s", "OFF");
            esp_mqtt_client_publish(Cliente_MQTT, PUMP_STATE_MQTT_TOPIC, buffer, 0, 0, 0);
        }

        ESP_LOGW(mef_bombeo_tag, "BOMBA APAGADA");
    }


    switch(est_MEF_control_bombeo_soluc)
    {
        
    case ESPERA_BOMBEO:

        /**
         *  Cuando se levante la bandera que indica que se cumplió el timeout del timer, y si el nivel del tanque
         *  principal esta por encima del límite de reposición de líquido establecido, se cambia al estado donde
         *  se enciende la bomba, y se carga en el timer el tiempo de encendido de la bomba y el periodo con el que
         *  se va a controlar si hay efectivamente flujo de solución en los canales.
         */
        if(mef_bombeo_timer_finished_flag && !app_level_sensor_level_below_limit(TANQUE_PRINCIPAL))
        {
            mef_bombeo_timer_finished_flag = 0;
            xTimerChangePeriod(aux_control_bombeo_get_timer_handle(), pdMS_TO_TICKS(mef_bombeo_tiempo_bomba_on), 0);
            xTimerReset(aux_control_bombeo_get_timer_handle(), 0);

            xTimerChangePeriod(xTimerSensorFlujo, pdMS_TO_TICKS(mef_bombeo_tiempo_control_sensor_flujo), 0);
            xTimerReset(xTimerSensorFlujo, 0);

            set_relay_state(BOMBA, ON);
            /**
             *  Se publica el nuevo estado de la bomba en el tópico MQTT correspondiente.
             */
            if(mqtt_check_connection())
            {
                char buffer[10];
                snprintf(buffer, sizeof(buffer), "%s", "ON");
                esp_mqtt_client_publish(Cliente_MQTT, PUMP_STATE_MQTT_TOPIC, buffer, 0, 0, 0);
            }

            ESP_LOGW(mef_bombeo_tag, "BOMBA ENCENDIDA");

            est_MEF_control_bombeo_soluc = BOMBEO_SOLUCION;
        }

        break;


    case BOMBEO_SOLUCION:

        /**
         *  Cuando se cumple el timeout del timer, se verifica si circula solución por el sensor de flujo ubicado
         *  en la entrada de los canales de cultivo.
         * 
         *  En caso de que no se detecte solución, se procede a publicar en el tópico común de alarmas el 
         *  código de alarma correspondiente a falla en la bomba de solución.
         * 
         *  Además, si se detecta que el nivel del tanque principal está por debajo de un cierto límite, que
         *  implica que debe reponerse el líquido del tanque, se transiciona al estado con la bomba apagada,
         *  a la espera de la reposición.
         */
        if(mef_bombeo_timer_flow_control_flag || app_level_sensor_level_below_limit(TANQUE_PRINCIPAL))
        {
            mef_bombeo_timer_flow_control_flag = 0;
            xTimerChangePeriod(xTimerSensorFlujo, pdMS_TO_TICKS(mef_bombeo_tiempo_control_sensor_flujo), 0);
            xTimerReset(xTimerSensorFlujo, 0);

            if(!flow_sensor_flow_detected())
            {
                if(mqtt_check_connection())
                {
                    char buffer[10];
                    snprintf(buffer, sizeof(buffer), "%i", ALARMA_FALLA_BOMBA);
                    esp_mqtt_client_publish(Cliente_MQTT, ALARMS_MQTT_TOPIC, buffer, 0, 0, 0);
                }

                ESP_LOGE(mef_bombeo_tag, "ALARMA, NO CIRCULA SOLUCIÓN POR LOS CANALES.");
            }
        }

        /**
         *  Cuando se levante la bandera que indica que se cumplió el timeout del timer, se cambia al estado donde
         *  se cierra la válvula, y se carga en el timer el tiempo de cierre de la válvula, además de parar el timer
         *  de control de flujo de solución.
         */
        if(mef_bombeo_timer_finished_flag)
        {
            mef_bombeo_timer_finished_flag = 0;
            xTimerChangePeriod(aux_control_bombeo_get_timer_handle(), pdMS_TO_TICKS(mef_bombeo_tiempo_bomba_off), 0);
            xTimerReset(aux_control_bombeo_get_timer_handle(), 0);

            xTimerStop(xTimerSensorFlujo, 0);
            mef_bombeo_timer_flow_control_flag = 0;

            set_relay_state(BOMBA, OFF);
            /**
             *  Se publica el nuevo estado de la bomba en el tópico MQTT correspondiente.
             */
            if(mqtt_check_connection())
            {
                char buffer[10];
                snprintf(buffer, sizeof(buffer), "%s", "OFF");
                esp_mqtt_client_publish(Cliente_MQTT, PUMP_STATE_MQTT_TOPIC, buffer, 0, 0, 0);
            }

            ESP_LOGW(mef_bombeo_tag, "BOMBA APAGADA");

            est_MEF_control_bombeo_soluc = ESPERA_BOMBEO;
        }

        break;
    }
}



/**
 * @brief   Tarea que representa la MEF principal (de mayor jerarquía) del algoritmo de 
 *          control del bombeo de la solución nutritiva, alternando entre el modo automatico
 *          o manual de control según se requiera.
 * 
 * @param pvParameters  Parámetro que se le pasa a la tarea en su creación.
 */
void vTaskSolutionPumpControl(void *pvParameters)
{
    /**
     * Variable que representa el estado de la MEF de jerarquía superior del algoritmo de control de bombeo de la solución.
     */
    static estado_MEF_principal_control_bombeo_soluc_t est_MEF_principal = ALGORITMO_CONTROL_BOMBEO_SOLUC;

    while(1)
    {
        /**
         *  Se realiza un Notify Take a la espera de señales que indiquen:
         *  
         *  -Que se debe pasar a modo MANUAL o modo AUTO.
         *  -Que estando en modo MANUAL, se deba cambiar el estado de la bomba.
         *  -Que se cumplió el timeout del timer de control del tiempo de encendido o apagado de la bomba.
         * 
         *  Además, se le coloca un timeout para evaluar las transiciones de las MEFs periódicamente, en caso
         *  de que no llegue ninguna de las señales mencionadas.
         */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));


        switch(est_MEF_principal)
        {
        
        case ALGORITMO_CONTROL_BOMBEO_SOLUC:

            /**
             *  En caso de que se levante la bandera de modo MANUAL, se debe transicionar a dicho estado,
             *  en donde el accionamiento de la bomba de solución será manejado por el usuario
             *  vía mensajes MQTT.
             */
            if(mef_bombeo_manual_mode_flag)
            {
                est_MEF_principal = MODO_MANUAL;
                mef_bombeo_reset_transition_flag_control_bombeo_solucion = 1;
            }

            MEFControlBombeoSoluc();

            break;


        case MODO_MANUAL:

            /**
             *  En caso de que se baje la bandera de modo MANUAL, se debe transicionar nuevamente al estado
             *  de modo AUTOMATICO, en donde se controla el encendido y apagado de la bomba por tiempos.
             */
            if(!mef_bombeo_manual_mode_flag)
            {
                est_MEF_principal = ALGORITMO_CONTROL_BOMBEO_SOLUC;
                break;
            }

            /**
             *  Se obtiene el nuevo estado en el que debe estar la bomba de solución y se acciona
             *  el relé correspondiente.
             */
            float manual_mode_bomba_state = -1;
            mqtt_get_float_data_from_topic(MANUAL_MODE_PUMP_STATE_MQTT_TOPIC, &manual_mode_bomba_state);

            if(manual_mode_bomba_state == 0 || manual_mode_bomba_state == 1)
            {
                set_relay_state(BOMBA, manual_mode_bomba_state);
                ESP_LOGW(mef_bombeo_tag, "MANUAL MODE BOMBA: %.0f", manual_mode_bomba_state);
            }

            break;
        }
    }
}

//==================================| EXTERNAL FUNCTIONS DEFINITION |==================================//

/**
 * @brief   Función para inicializar el módulo de MEFs del algoritmo de control de bombeo de solución. 
 * 
 * @param mqtt_client   Handle del cliente MQTT.
 * @return esp_err_t 
 */
esp_err_t mef_bombeo_init(esp_mqtt_client_handle_t mqtt_client)
{
    /**
     *  Copiamos el handle del cliente MQTT en la variable interna.
     */
    MefBombeoClienteMQTT = mqtt_client;

    //=======================| INIT TIMERS |=======================//

    /**
     *  Se inicializa el timer utilizado para la temporización del control de flujo en los
     *  canales de cultivo.
     * 
     *  Se inicializa su período en 1 tick dado que no es relevante en su inicialización, ya que
     *  el tiempo de apertura y cierre de la válvula se asignará en la MEF cuando corresponda, pero
     *  no puede ponerse 0.
     */
    xTimerSensorFlujo = xTimerCreate("Timer Sensor Flujo",       // Nombre interno que se le da al timer (no es relevante).
                              1,                                // Período del timer en ticks.
                              pdFALSE,                          // pdFALSE -> El timer NO se recarga solo al cumplirse el timeout. pdTRUE -> El timer se recarga solo al cumplirse el timeout.
                              (void *)20,                        // ID de identificación del timer.
                              vSensorFlujoTimerCallback                    // Nombre de la función de callback del timer.
    );

    /**
     *  Se verifica que se haya creado el timer correctamente.
     */
    if(xTimerSensorFlujo == NULL)
    {
        ESP_LOGE(aux_control_bombeo_tag, "FAILED TO CREATE TIMER.");
        return ESP_FAIL;
    }

    //=======================| CREACION TAREAS |=======================//
    
    /**
     *  Se crea la tarea mediante la cual se controlará la transicion de las
     *  MEFs del algoritmo de control de bombeo de solución.
     */
    if(xMefBombeoAlgoritmoControlTaskHandle == NULL)
    {
        xTaskCreate(
            vTaskSolutionPumpControl,
            "vTaskSolutionPumpControl",
            4096,
            NULL,
            2,
            &xMefBombeoAlgoritmoControlTaskHandle);
        
        /**
         *  En caso de que el handle sea NULL, implica que no se pudo crear la tarea, y se retorna con error.
         */
        if(xMefBombeoAlgoritmoControlTaskHandle == NULL)
        {
            ESP_LOGE(mef_bombeo_tag, "Failed to create vTaskSolutionPumpControl task.");
            return ESP_FAIL;
        }
    }
    
    return ESP_OK;
}



/**
 * @brief   Función que devuelve el Task Handle de la tarea principal del algoritmo de control de bombeo de solución.
 * 
 * @return TaskHandle_t Task Handle de la tarea.
 */
TaskHandle_t mef_bombeo_get_task_handle(void)
{
    return xMefBombeoAlgoritmoControlTaskHandle;
}



/**
 * @brief   Función para establecer un nuevo tiempo de encendido de la bomba de solución.
 * 
 * @param nuevo_tiempo_bomba_on   Tiempo de encendido de la bomba.
 */
void mef_bombeo_set_pump_on_time_min(pump_time_t nuevo_tiempo_bomba_on)
{
    mef_bombeo_tiempo_bomba_on = nuevo_tiempo_bomba_on;
}



/**
 * @brief   Función para establecer un nuevo tiempo de apagado de la bomba de solución.
 * 
 * @param nuevo_tiempo_bomba_off   Tiempo de apagado de la bomba.
 */
void mef_bombeo_set_pump_off_time_min(pump_time_t nuevo_tiempo_bomba_off)
{
    mef_bombeo_tiempo_bomba_off = nuevo_tiempo_bomba_off;
}



/**
 * @brief   Función para cambiar el estado de la bandera de modo MANUAL, utilizada por
 *          la MEF para cambiar entre estado de modo MANUAL y AUTOMATICO.
 * 
 * @param manual_mode_flag_state    Estado de la bandera.
 */
void mef_bombeo_set_manual_mode_flag_value(bool manual_mode_flag_state)
{
    mef_bombeo_manual_mode_flag = manual_mode_flag_state;
}



/**
 * @brief   Función para cambiar el estado de la bandera de timeout del timer mediante
 *          el que se controla el tiempo de encendido y apagado de la bomba de solución.
 * 
 * @param manual_mode_flag_state    Estado de la bandera.
 */
void mef_bombeo_set_timer_flag_value(bool timer_flag_state)
{
    mef_bombeo_timer_finished_flag = timer_flag_state;
}