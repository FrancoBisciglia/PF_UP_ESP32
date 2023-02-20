/**
 * @file MQTT_PUBL_SUSCR.c
 * @author Franco Bisciglia, David Kündinger
 * @brief   Librería mediante la cual, a través de una conexión WiFi previamente establecida, se establece conexión con el servidor MQTT
 *          correspondientemente definido, pudiendo así publicar en tópicos o suscribirse a los mismos.
 * @version 0.1
 * @date 2022-12-17
 * 
 * @copyright Copyright (c) 2022
 * 
 */



/**
 * =================================================| EXPLICACIÓN DE LIBRERÍA |=================================================
 *      Mediante esta librería, teniendo en cuenta que previamente debe haberse establecido una conexión TCP a alguna red,
 *  se puede establecer conexión a un broker MQTT cuya dirección se conoce. Esto se logra mediante la función
 *  "mqtt_initialize_and_connect()", donde el URI del broker se pasa como argumento, además del handle del cliente MQTT.
 * 
 *      Además, también es posible suscribirse a una lista de tópicos MQTT a través de la función "mqtt_suscribe_to_topics()",
 *  a la cual se le pasa como argumento una lista de los nombres de los tópicos correspondientes y su cantidad, además del
 *  handle del cliente MQTT y el nivel de Quality of Service que tendrán los mensajes MQTT de esos tópicos suscritos.
 * 
 *      Para obtener los datos que se publiquen en dichos tópicos suscritos, se pueden utilizar las funciones
 *  "mqtt_get_float_data_from_topic()", para obtener el último dato publicado en el tópico en formato float, o 
 *  "mqtt_get_char_data_from_topic()", para obtenerlo en formato de string. A ambas se le debe pasar como argumento
 *  el nombre del tópico del cual se desea obtener el dato.
 * 
 *      Con la función "mqtt_check_connection()", se puede conocer si se está o no conectado al broker MQTT.
 * 
 *      Si se desea publicar un dato en un tópico, se debe utilizar la función estándar "esp_mqtt_client_publish()" 
 *  de la librería de ESP-IDF.
 */


//==================================| INCLUDES |==================================//

#include <stdio.h>
#include <stdlib.h>

#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_check.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "mqtt_client.h"

#include "MQTT_PUBL_SUSCR.h"
#include "esp_log.h"

//==================================| MACROS AND TYPDEF |==================================//

//==================================| INTERNAL DATA DEFINITION |==================================//

//Tag para imprimir información en el LOG.
static const char *TAG = "MQTT_LIBRARY";

//Bandera para verificar si se estableció la conexión con el broker MQTT.
static bool MQTT_CONNECTED = 0;

//Cantidad de tópicos a suscribir.
static unsigned int mqtt_topic_num = 0;

/**
 *  Variable en la cual se guardaran los nombres de los tópicos MQTT suscritos, 
 *  junto con los datos que se obtendrán por publicaciones en los mismos, y el task
 *  handle de la tarea a la cual se le quiere informar la llegada de un nuevo
 *  dato al topico correspondiente.
 */
static mqtt_subscribed_topic_data* mqtt_topic_list = NULL;

//==================================| EXTERNAL DATA DEFINITION |==================================//

//==================================| INTERNAL FUNCTIONS DECLARATION |==================================//

//==================================| INTERNAL FUNCTIONS DEFINITION |==================================//

/**
 * @brief Función correspondiente al handler de eventos MQTT.
 *
 *  Esta función es llamada por el client event loop cada vez que se produce un evento que se corresponda
 *  con los configurados mediante la función "esp_mqtt_client_register_event".
 *
 *
 * @param handler_args Datos pasados como argumento por el usuario al utilizar la función "esp_mqtt_client_register_event".
 * @param base Tipo base desde el cual se llamó al handler, MQTT en este caso.
 * @param event_id ID del evento recibido.
 * @param event_data Datos recibidos desde el evento, del tipo "esp_mqtt_event_handle_t".
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;


    switch ((esp_mqtt_event_id_t)event_id) {
    
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        
        //Seteamos la variable global para informar que estamos conectados a un broker MQTT
        MQTT_CONNECTED = 1;

        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        
        //Reseteamos la variable global para indicar que nos deconectamos del broker MQTT
        MQTT_CONNECTED = 0;
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;

    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;

    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;

    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT SUBSCRIBED MESSAGE ARRIVED.");
        //ESP_LOGI(TAG, "MQTT_EVENT_DATA: %.*s", event->data_len, event->data);

        /**
         *  Debido a que el nombre del topico que llega por "event->topic" generalmente contiene
         *  caracteres basura por fuera del tamaño de "event->topic_len", entonces se copia hasta
         *  esa cantidad de caracteres en un array auxiliar que será usado luego para comparación.
         */
        char topic_aux[100] = "";
        strncpy(topic_aux, event->topic, event->topic_len);

        /**
         *  Se compara el nombre del tópico al cual llegó el dato con la lista de tópicos anteriormente definida.
         *  En caso de coincidir con algun nombre de la lista, se copia el dato correspondiente.
         */
        for(int i = 0; i < mqtt_topic_num; i++)
        {
            if(!strcmp(mqtt_topic_list[i].topic, topic_aux))
            {
                /**
                 *  Se borra el contenido anterior del dato del tópico correspondiente y se
                 *  le copia el nuevo, pero solo la cantidad de caracteres "data_len", porque
                 *  si se copia todo "event->data", hay caracteres basura.
                 */
                memset(mqtt_topic_list[i].data, 0, sizeof(mqtt_topic_list[i].data));
                strncpy(mqtt_topic_list[i].data, event->data, event->data_len);

                /**
                 *  En caso de que para este tópico se haya cargado una función callback, se la ejecuta.
                 */
                if(mqtt_topic_list[i].topic_cb != NULL)
                {
                    mqtt_topic_list[i].topic_cb(NULL);
                }

                ESP_LOGI(TAG, "TOPIC DATA ARRIVED: %s", mqtt_topic_list[i].data);

                continue;
            }
        }

        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            // log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            // log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            // log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

        }
        break;

    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}


//==================================| EXTERNAL FUNCTIONS DEFINITION |==================================//

/**
 * @brief   Función mediante la cual, a partir de la dirección del broker MQTT (URI) y del handle del cliente MQTT,
 *          se establece una conexión con dicho broker MQTT.
 * 
 * @param MQTT_BROKER_URI Dirección (URI) del broker MQTT.
 * @param MQTT_CLIENT Handle del cliente MQTT.
 */
esp_err_t mqtt_initialize_and_connect(char* MQTT_BROKER_URI, esp_mqtt_client_handle_t* MQTT_client)
{
    /*====================/CONEXIÓN AL BROKER MQTT/====================*/

    /**
     *  Se crea la variable correspondiente a la dirección del broker MQTT, pasada como argumento.
     */
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = MQTT_BROKER_URI,
        .disable_auto_reconnect = 0,    //Al poner este campo en FALSE, al ocurrir una desconexión inesperada, se intentará una reconexión
    };

    /**
     *  A partir de la configuración establecida, se obtiene la variable handle del cliente MQTT
     */
    *MQTT_client = esp_mqtt_client_init(&mqtt_cfg);

    if(MQTT_client == NULL)
    {
        ESP_LOGE(TAG, "MQTT ERROR: Failed to get MQTT client.");
        
        /**
         * NOTA: REVISAR SI ACA ES NECESARIO HACER FREE
         * 
         */
        free(MQTT_client);
        return ESP_FAIL;
    }

    /**
     *  Se establece qué tipo de eventos MQTT van a ser atendidos por el handler de eventos MQTT.
     */
    ESP_RETURN_ON_ERROR(esp_mqtt_client_register_event(*MQTT_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL), 
                        TAG, "Failed to register MQTT event.");

    /**
     *  Se inicia la conexión con el broker MQTT.
     */
    ESP_RETURN_ON_ERROR(esp_mqtt_client_start(*MQTT_client), 
                        TAG, "Failed to start MQTT connection.");

    return ESP_OK;

}


/**
 * @brief Función para verificar si ya se estableció la conexión con el broker MQTT.
 * 
 * @return bool 
 */
bool mqtt_check_connection()
{
    return MQTT_CONNECTED;
}


/**
 * @brief   Función mediante la cual se registran y se suscribe a los tópicos MQTT que se pasen como argumento.
 * 
 * @param list_of_topics   Listado de nombres de los tópicos MQTT a suscribir.
 * @param number_of_new_topics  Cantidad de tópicos nuevos a suscribir.
 * @param mqtt_client   Handle del cliente MQTT.
 * @param qos   Quality of Service de la comunicación MQTT.
 * 
 * @return esp_err_t 
 */
esp_err_t mqtt_suscribe_to_topics(  const mqtt_topic_t* list_of_topics, const unsigned int number_of_new_topics, 
                                    esp_mqtt_client_handle_t mqtt_client, int qos)
{

    /**
     *  Se verifica que los argumentos recibidos no estén vacíos.
     */
    if(list_of_topics == NULL || number_of_new_topics == 0 || mqtt_client == NULL || qos < 0 || qos > 3)
    {
        ESP_LOGE(TAG, "MQTT ERROR: Failed to suscribe to topics. Enter valid arguments.");
        return ESP_ERR_INVALID_ARG;
    }

    /**
     *  Se guarda la cantidad de topicos a suscribir.
     */
    mqtt_topic_num += number_of_new_topics;

    /**
     *  Se inicializa el array de punteros a la estructura que contendrá los nombres de los tópicos MQTT a suscribir,
     *  junto con el último dato recibido desde dicho tópico.
     */
    if(mqtt_topic_list == NULL)
    {
        mqtt_topic_list = calloc(number_of_new_topics, sizeof(mqtt_subscribed_topic_data));
    }

    /**
     *  En caso de que ya se haya inicializado la primera vez, se procede a concatenar los nuevos topicos en el array,
     *  aumentando a su vez el tamaño del mismo en una cantidad igual a "number_of_new_topics".
     */
    else
    {
        mqtt_topic_list = (mqtt_subscribed_topic_data*) realloc(mqtt_topic_list, mqtt_topic_num * sizeof(mqtt_subscribed_topic_data));
    }

    /**
     *  Se verifica si se pudo reservar correctamente la memoria.
     */
    if(mqtt_topic_list == NULL)
    {
        ESP_LOGE(TAG, "MQTT ERROR: Failed to allocate memory.");
        free(mqtt_topic_list);
        return ESP_ERR_NO_MEM;
    }

    /**
     *  Se copian los nombres y punteros a función callback de los tópicos correspondientes y se suscribe a los mismos.
     */
    for(int i = (mqtt_topic_num - number_of_new_topics); i < mqtt_topic_num; i++)
    {
        mqtt_topic_list[i].topic_cb = list_of_topics[i-(mqtt_topic_num - number_of_new_topics)].topic_function_cb;
        strcpy(mqtt_topic_list[i].topic, list_of_topics[i-(mqtt_topic_num - number_of_new_topics)].topic_name);

        /**
         *  En caso de que la función retorne -1, implica que no se pudo suscribir al
         *  tópico correspondiente, y se retorna con error.
         */
        if(esp_mqtt_client_subscribe(mqtt_client, mqtt_topic_list[i].topic, qos) == ESP_FAIL)
        {
            ESP_LOGE(TAG, "MQTT ERROR: Failed to suscribe to topic: %s", mqtt_topic_list[i].topic);

            return ESP_FAIL;
        }
    }

    return ESP_OK;

}


/**
 * @brief   Función para obtener el último dato de un determinado tópico en formato float.
 * 
 * @param topic Nombre del tópico MQTT del cual se obtendrá el último dato.
 * @param buffer Variable en la cual se guardará el dato.
 * 
 * @return esp_err_t 
 */
esp_err_t mqtt_get_float_data_from_topic(const char* topic, float* buffer)
{
    /**
     *  Se convierte el dato del tópico correspondiente, que es del formato char,
     *  al formato float y se lo carga en el buffer pasado como argumento.
     */
    for(int i = 0; i < mqtt_topic_num; i++)
    {
        if(!strcmp(mqtt_topic_list[i].topic, topic))
        {
            *buffer = atof(mqtt_topic_list[i].data);
        }
    }

    return ESP_OK;
}


/**
 * @brief   Función para obtener el último dato de un determinado tópico en formato de cadena de caracteres.
 * 
 * @param topic Nombre del tópico MQTT del cual se obtendrá el último dato.
 * @param buffer Variable en la cual se guardará el dato.
 * 
 * @return esp_err_t 
 */
esp_err_t mqtt_get_char_data_from_topic(const char* topic, char* buffer)
{
    /**
     *  Se obtiene el dato del tópico correspondiente y se lo carga en el buffer 
     *  pasado como argumento.
     */
    for(int i = 0; i < mqtt_topic_num; i++)
    {
        if(!strcmp(mqtt_topic_list[i].topic, topic))
        {
            strcpy(buffer, mqtt_topic_list[i].data);
            ESP_LOGI(TAG, "BUFFER: %s", buffer);
        }
    }

    return ESP_OK;
}