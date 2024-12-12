
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"

#include "MFP_Protocol.h"
#include "uart.h"
#include "utils.h"
#include "ABNT.h"

#define UART2_TX_PIN GPIO_NUM_18
#define UART2_RX_PIN GPIO_NUM_5

static const char *TAG = "UART: ";

extern QueueHandle_t send_message_mesh_q;
extern QueueHandle_t rcv_message_mesh_q;
extern QueueHandle_t send_message_TCP_q;

uint8_t message[1024];

/**
 * @brief Initialization of UART peripheral, starting the main tasks of receiving commands from PC and communicating with the meter
 */
void start_uart()
{
    send_message_mesh_q = xQueueCreate(QUEUE_SIZE, sizeof(message_t));
    if (send_message_mesh_q == NULL)
        ESP_LOGE(TAG, "Failed to create queue");
    else ESP_LOGE(TAG, "Success in create queue");

    rcv_message_mesh_q = xQueueCreate(QUEUE_SIZE, sizeof(message_t));
    if (rcv_message_mesh_q == NULL)
        ESP_LOGE(TAG, "Failed to create queue");
    else ESP_LOGE(TAG, "Success in create queue");
    
    uart_init();

    xTaskCreate(uart_rcv_task, "uart_rcv_task", 3072, NULL, configMAX_PRIORITIES - 2, NULL);
    xTaskCreate(nock_loose_task, "nock_loose_task", 4096, NULL, configMAX_PRIORITIES - 1, NULL);

}

/**
 * @brief Initialize UART peripherical
 */
void uart_init(void)
{
    // UART0 USB communication
    uart_config_t uart_config_0 = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_UART0, &uart_config_0));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_UART0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_UART0, BUF_SIZE * 2, 0, 0, NULL, 0));

    // UART2 meter communication
    uart_config_t uart_config_2 = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_UART2, &uart_config_2));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_UART2, UART2_TX_PIN, UART2_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_UART2, BUF_SIZE, 0, 0, NULL, 0));
     // Inverts the TX and RX sign
    ESP_ERROR_CHECK(uart_set_line_inverse(UART_NUM_UART2, UART_SIGNAL_TXD_INV  | UART_SIGNAL_RXD_INV));

    vTaskDelay(pdMS_TO_TICKS(1000));
    uart_flush(UART_NUM_UART0);
    uart_flush(UART_NUM_UART2);
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "UARTs inicializadas com sucesso");
}

/**
 * @brief task to receive commands from the PC's UART and add them to the Queue that will be sent to the Mesh network
 **/
void uart_rcv_task(void *pvParameters)
{
    uint8_t *recv_buffer = NULL;

    for(;;)
    {
        // Read data from UART0
        int len = 0;
        ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_UART0, (size_t*)&len));
    
        if (len > 0)//MFP_PROTOCOL_HEADER + 3) //Portocol header + 1 byte payload + 2 CRC16
        {
            recv_buffer = (uint8_t*) malloc(len * sizeof(uint8_t));
             if (recv_buffer == NULL) {
                ESP_LOGE(TAG, "Failed to allocate memory for recv_buffer");
                break;
            }

            len = uart_read_bytes(UART_NUM_UART0, recv_buffer, len, 100 / portTICK_PERIOD_MS);
            data_check_status_e dataStatus = process_data_send_mesh_queue(recv_buffer, len, send_message_mesh_q);

            if (dataStatus == DATA_CHECK_SUCCESS) {
            ESP_LOGI(TAG, "Data successfully processed and sent to queue.");
            } else {
                ESP_LOGE(TAG, "Failed to process data. Error code: %d", dataStatus);
            }

            free(recv_buffer);

        }//if (len > 0)
        
        // UBaseType_t highWaterMark = uxTaskGetStackHighWaterMark(NULL);
        // ESP_LOGI("uart_rcv_task", "Stack high water mark: %d words", highWaterMark);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

    }//for(;;)
}//void uart_rcv_task()

/**
 * @brief task to receive commands from the MESH Network prepare to send and communicate with the meter 
 **/
void nock_loose_task(void *pvParameters)
{
    message_t dyn_msg;
    

    for(;;)
    {
        if(xQueueReceive(rcv_message_mesh_q, &dyn_msg, (TickType_t)portMAX_DELAY))
        {
            
            // ESP_LOGI(TAG, "Recebi da NOCK LOOSE os dados %d:", dyn_msg.length);
            // for (int i = 0; i < dyn_msg.length; i++)
            //     printf("%02X ", dyn_msg.buffer[i]);
            // printf("\n\n");

            if(dyn_msg.buffer[9] == QUESTION)
            {
                communication_type_e type = get_communication_type(dyn_msg.buffer[8]); // 08 protocol type byte
                ESP_LOGI(TAG, "Comunication Type: %02X", dyn_msg.buffer[8]);

                switch(type)
                {
                    case ABNT:
                        uint8_t recv_buffer_rx[DATA_RECEIVED_LENGTH_ABNT] = {0};
                        send_ABNT_14522_command(dyn_msg.buffer, dyn_msg.length, recv_buffer_rx);
                        write_meter_response_mesh_queue(recv_buffer_rx, DATA_RECEIVED_LENGTH_ABNT);

                    break;
                    case DLMS:
                        ESP_LOGI(TAG, "DLMS not defined yet");
                    break;
                    case MESH:
                        ESP_LOGI(TAG, "MESH not defined yet");
                    break;
                    default:
                        ESP_LOGE(TAG, "Protocol not defined yet");
                    break;
                }
            }//  if(dyn_msg.buffer[9] == QUESTION)
            else if (dyn_msg.buffer[9] == ANSWER)
            {
                // ESP_LOGI(TAG, "Recebi da NOCK LOOSE os dados RESPOSTA %d:", dyn_msg.length);
                // for (int i = 13; i < (dyn_msg.length); i++)
                //     printf("%02X ", dyn_msg.buffer[i]);
                // printf("\n\n");
                // Tenta enviar para a fila
                if (xQueueSend(send_message_TCP_q, &dyn_msg, pdMS_TO_TICKS(100)) != pdTRUE) {
                    ESP_LOGE(TAG, "Failed to send data to queue");
                    free(dyn_msg.buffer);
                }

            } else{
                ESP_LOGE(TAG, "Protocol not defined yet");
            }
        }//if (xQueueReceive(rcv_message_mesh_q, &cmd, (TickType_t)portMAX_DELAY)) 

        UBaseType_t highWaterMark = uxTaskGetStackHighWaterMark(NULL);
        ESP_LOGI("nock_loose_task", "Stack high water mark: %d words", highWaterMark);

    }//for(;;)
}//void nock_loose_task()
