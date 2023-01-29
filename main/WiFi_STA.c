/**
 * @file WiFi_STA.c
 * @author Franco Bisciglia, David Kündinger
 * @brief   Librería mediante la cual se configura al ESP32 en modo de conexión WiFi STA (stationary), y se lo conecta a una red
 *          WiFi existente, cuyo SSID y PSWD se pasan como argumento.
 * @version 0.1
 * @date 2022-12-19
 * 
 * @copyright Copyright (c) 2022
 * 
 */

//==================================| INCLUDES |==================================//

#include "WiFi_STA.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_check.h"

#include "nvs_flash.h"

#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"
#include "lwip/sys.h"

//==================================| MACROS AND TYPDEF |==================================//

//==================================| INTERNAL DATA DEFINITION |==================================//

/* Bandera para conocer el estado de la conexión WiFi */
static bool wifi_conn_flag = 0;

/* Tag para imprimir información en el LOG. */
static const char *TAG = "WiFi Library";

/* Event group de WiFi donde se señalizan los distintos eventos WiFi ¨*/
static EventGroupHandle_t wifi_event_group;

/* Handle de la tarea de reconexion de WiFi */
static TaskHandle_t xWiFiReconnTaskHandle = NULL;

//==================================| EXTERNAL DATA DEFINITION |==================================//

//==================================| INTERNAL FUNCTIONS DECLARATION |==================================//

void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
void ip_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
static void vTaskWiFiReconn(void *pvParameters);

//==================================| INTERNAL FUNCTIONS DEFINITION |==================================//

/**
 * @brief   Funcion de handler para eventos del tipo WiFi.
 * 
 * @param arg   Argumento pasado como dato al inicializar el Event Group.
 * @param event_base    Tipo de evento (IP/WIFI).
 * @param event_id      ID del evento.      
 * @param event_data    Datos del evento.
 */
void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    /**
     *  Controlamos que el evento mediante el que se ingresó al handler sea del tipo WiFi, y que a su vez sea porque
     *  se inicializo el ESP32 en modo STA correctamente.
     */
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        
        ESP_LOGI(TAG, "Connecting to AP...");

        //Mediante esta función, se realiza la conexion efectiva entre el ESP32 en modo STA y el AP
        esp_wifi_connect();

    /**
     *  En caso de que haya habido un evento del tipo WiFi, y este evento haya sido que se produjo una desconexión del WiFi,
     *  se ingresa para intentar reconectarse.
     */
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {

        /**
         *  Esta variable sirve para que, en el caso de que un llamado a "xTaskNotifyFromISR()" desbloquee
         *  una tarea de mayor prioridad que la que estaba corriendo justo antes de entrar en la rutina
         *  de interrupción, al retornar se haga un context switch a dicha tarea de mayor prioridad en vez
         *  de a la de menor prioridad (xHigherPriorityTaskWoken = pdTRUE)
         */
        BaseType_t xHigherPriorityTaskWoken;
        xHigherPriorityTaskWoken = pdFALSE;

        /**
         *  Enviamos un Task Notify a la tarea de reconexión de WiFi, para informarle que se intentó la
         *  reconexión, pero falló, por lo que se debe intentar nuevamente. Se resetea además la bandera
         *  representativa del estado de conexión de WiFi.
         */
        vTaskNotifyGiveFromISR(xWiFiReconnTaskHandle, &xHigherPriorityTaskWoken);
        wifi_conn_flag = 0;

        /**
         *  Devolvemos el procesador a la tarea que corresponda. En el caso de que xHigherPriorityTaskWoken = pdTRUE,
         *  implica que se debe realizar un context switch a la tarea de mayor prioridad que fue desbloqueada.
         */
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {

        wifi_conn_flag = 1;

    }

}



/**
 * @brief   Funcion de handler para eventos del tipo IP.
 * 
 * @param arg   Argumento pasado como dato al inicializar el Event Group.
 * @param event_base    Tipo de evento (IP/WIFI).
 * @param event_id      ID del evento.      
 * @param event_data    Datos del evento.
 */
void ip_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    /**
     *  Se controla que se haya recibido un evento del tipo IP, y que el mismo sea que se le asigno una IP
     *  correctamente al ESP32
     */
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        /**
         *  Se crea una variable para obtener el dato del IP asignado y mostrarlo en pantalla
         */
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "STA IP:" IPSTR, IP2STR(&event->ip_info.ip));
    }

}



/**
 * @brief   Tarea que se encarga de realizar una reconexión a la red WiFi externa en
 *          caso de desconexión.
 * 
 * @param pvParameters  Parámetros pasados a la tarea en su creación.
 */
static void vTaskWiFiReconn(void *pvParameters)
{
    while(1)
    {
        /**
         *  Se espera indefinidamente a recibir un Task Notify desde el wifi event handler, solo
         *  en caso de desconexión de la red WiFi.
         */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /**
         *  Se realiza el intento de reconexión a la red WiFi.
         */
        esp_wifi_connect();

        ESP_LOGW(TAG, "ATTEMPTING TO RECONNECT TO WIFI NETWORK...");

        /**
         *  Se espera un cierto tiempo antes de intentar la reconexión (5 seg en este caso).
         */
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

}

//==================================| EXTERNAL FUNCTIONS DEFINITION |==================================//

/**
 * @brief   Función utilizada para conectarse a una red WiFi existente.
 * 
 * @param wifi_network  Estructura que contiene el ID y PASS de la red WiFi.
 * @return esp_err_t 
 */
esp_err_t connect_wifi(wifi_network_t* wifi_network)
{
    //========================| INICIALIZACIÓN |===========================//

    /**
     *  Se inicializa el "non volatile storage" (almacenamiento no volatil) de la flash, donde se guardará la configuración de WiFi
     */
    esp_err_t ret;
    ret = nvs_flash_init();

    /**
     *  En caso de que no haya espacio en la sección de NVS de la flash, se la borra y reinicializa.
     */
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "Failed to erase NVS FLASH.");
        ESP_RETURN_ON_ERROR(nvs_flash_init(), TAG, "Failed to initialize NVS FLASH.");
    }

    /**
     *  Se inicializa la interfaz de red del ESP32.
     */
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "Failed to initialize network interface.");

    /**
     *  Se crea un "evento loop" estandar, que es un ciclo interno que se utiliza para controlar eventos, 
     *  como por ejemplo, conexiones o desconexiones a red WiFi, o llegada de datos.
     */
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "Failed to create event loop.");

    /**
     *  Se le indica a la interfaz de red que vamos a utilizar WiFi y en el modo STA, con la configuración 
     *  estándar o default.
     * 
     */
    esp_netif_create_default_wifi_sta();

    /**
     *  Se crea una variable mediante la cual se realiza la configuración para el inicio de los parametros 
     *  del WiFi, en este caso, se obtiene la configuración estándar.
     */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    /**
     *  Se carga la configuración obtenida, pero todavía no se inicia el WiFi.
     */
    ESP_RETURN_ON_ERROR(esp_wifi_init(&cfg), TAG, "Failed to load WiFi init config.");


    //========================| CONFIGURACIÓN DEL EVENT LOOP |===========================//

    /**
     *  Se crea una variable que representa una instancia del handler de eventos de WiFi, que luego puede 
     *  utilizarse para cerrar dicho handler.
     */
    esp_event_handler_instance_t wifi_handler_event_instance;

    /**
     *  Se instancia el handler de eventos:
     * 
     *  -El primer argumento es para determinar si se espera un evento del tipo WiFi o del tipo IP.
     *  -El segundo argumento es el tipo de evento específico que se atenderá, en este caso cualquiera.
     *  -El tercer parámetro es la función del handler que atenderá el evento ocurrido.
     *  -El cuarto parámetro son los argumentos que se le pasarán al handler.
     *  -El quinto parametro es la instancia del handler declarada anteriormente.
     */
    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(WIFI_EVENT,
                                                            ESP_EVENT_ANY_ID,
                                                            &wifi_event_handler,
                                                            NULL,
                                                            &wifi_handler_event_instance), 
                                                            TAG, "Failed to register WiFi event handler.");

    /**
     *  Se realiza el mismo procedimiento de creacion de instancia de un handler, pero en este caso para eventos del tipo IP.
     * 
     */
    esp_event_handler_instance_t got_ip_event_instance;
    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &ip_event_handler,
                                                        NULL,
                                                        &got_ip_event_instance), 
                                                        TAG, "Failed to register IP event handler.");



    //========================| INICIO DEL DRIVER DE WIFI |===========================//

    /**
     *  Se crea la estructura mediante la cual se configuran las variables del WiFi, como el nombre de la red y contraseña.
     */
    wifi_config_t wifi_config = {
        //En este caso, la conexion es del tipo STA (stationary), que es cuando el ESP32 se conecta a una red existente
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,   //Se establece el modo de conexion WPA2
            .pmf_cfg = {
                .capable = true,                        /**< Deprecated variable. Device will always connect in PMF mode if other device also advertizes PMF capability. */
                .required = false,                      /**< Advertizes that Protected Management Frame is required. Device will not associate to non-PMF capable devices. */
            },
        },
    };

    /**
     *  Se deben copiar de esta forma los datos de la red porque sino lanza un error.
     */
    strcpy((char*)wifi_config.sta.ssid, wifi_network->ssid);
    strcpy((char*)wifi_config.sta.password, wifi_network->pass);

    /**
     *  Establecemos el modo de la conexion WiFi al tipo STA.
     */
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "Failed to set STA mode.");

    /**
     *  Se carga la configuración de WiFi establecida.
     * 
     */
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &wifi_config), TAG, "Failed to load WiFi config.");

    /**
     *  Se inicia el driver de WiFi.
     */
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "Failed to start WiFi driver.");

    ESP_LOGI(TAG, "WiFi STA mode initialization complete.");



    //========================| CREACIÓN DE TAREA |===========================//

    /**
     *  Se crea la tarea encargada de reconectarse al WiFi en caso de desconexión del mismo.
     * 
     *  Se le da una prioridad alta ya que la conexión al WiFi es fundamental para el funcionamiento
     *  del sistema, pero como esta tarea es muy simple y solo actua en caso de desconexión y el
     *  resto del tiempo se encuentra bloqueada, no consumirá prácticamente tiempo de procesador.
     */
    if(xWiFiReconnTaskHandle == NULL)
    {
        xTaskCreate(
            vTaskWiFiReconn,
            "vTaskWiFiReconn",
            2048,
            NULL,
            4,
            &xWiFiReconnTaskHandle);
        
        /**
         *  En caso de que el handle sea NULL, implica que no se pudo crear la tarea, y se retorna con error.
         */
        if(xWiFiReconnTaskHandle == NULL)
        {
            ESP_LOGE(TAG, "Failed to create vTaskWiFiReconn task.");
            return ESP_FAIL;
        }
    }

    return ESP_OK;

}



/**
 * @brief   Función para conocer el estado de conexión WiFi (STA MODE).
 * 
 * @return true     Conectado a red externa.
 * @return false    No conectado a red externa.
 */
bool wifi_check_connection()
{
    return wifi_conn_flag;
}


