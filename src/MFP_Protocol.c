#include "MFP_Protocol.h"
#include "common.h"

extern uint8_t my_mac[6];
extern QueueHandle_t send_message_mesh_q;
extern QueueHandle_t send_message_mesh_q;

static const char *TAG = "MFP Protocol: ";

/**
 * @brief  Identifies the communication protocol based on the provided string.
 *
 * This function compares the input protocol string with known protocol names
 * (ABNT, DLMS, ANSW, ABNTUO, PIMA) and returns a corresponding identifier.
 * If the protocol does not match any known values, the function returns 0.
 *
 * @param  protocol  A constant character pointer to the protocol string.
 *
 * @return  An integer identifier for the protocol:
 *          - 1 for "ABNT"
 *          - 2 for "DLMS"
 *          - 3 for "ANSW"
 *          - 4 for "ABNTUO"
 *          - 5 for "PIMA"
 *          - 0 if the protocol is unknown.
 */
communication_type_e get_communication_type(uint8_t byte) {
    switch (byte) {
        case ABNT_PROTOCOL:
            return ABNT;
        case DLMS_PROTOCOL:
            return DLMS;
        case MESH_PROTOCOL:
            return MESH;
        default:
            return INVALID_TYPE;
    }
}

void write_meter_response_mesh_queue(uint8_t *data, uint16_t length)
{
    message_t message;

    message.length = MFP_PROTOCOL_HEADER + length + 2;
    message.buffer = (uint8_t *)malloc(message.length * sizeof(uint8_t)); // Aloca memória para a cópia dos dados
    if (message.buffer != NULL)
    {
        message.buffer[0] = TAG_1;
        memcpy(&message.buffer[1], meter_attrib.mac, 6);
        message.buffer[7] = TAG_2;
        message.buffer[8] = ABNT_PROTOCOL;
        message.buffer[9] = ANSWER;
        message.buffer[10] = TAG_3;
        message.buffer[11] = (length >> 8) & 0xFF ;         // Add payload size MSB
        message.buffer[12] = length & 0xFF;                 // Add payload size LSB
        memcpy(&message.buffer[13], data, length);          // Add payload
        
        // Calculate CRC and add it at the end
        uint16_t crc = calculate_crc16(message.buffer, message.length - 2);
        message.buffer[message.length - 2] = (crc >> 8) & 0xFF;
        message.buffer[message.length - 1] = crc & 0xFF;

        ESP_LOGI(TAG, "Recebi essa quantidade de dados PARA COLOCAR NA FILA PARA MESH: %d e sao eles: ", message.length);
            for (int i = 0; i < message.length; i++)
            {
                printf("%02X ", message.buffer[i]);
            }
            printf("\n\n");

        // Put data into the queue
        if (xQueueSend(send_message_mesh_q, &message, pdMS_TO_TICKS(100)) != pdTRUE)
        {
            ESP_LOGE(TAG, "Failed to send data to queue");
            free(message.buffer);
        }
        
    } else{
        ESP_LOGE(TAG, "Falha ao alocar memória para mesh_data.data");
        free(message.buffer); // Libera o buffer da mensagem antes de sair
    }
}

data_check_status_e process_data_send_mesh_queue(uint8_t *recv_buffer, int len, QueueHandle_t queue) {
    uint16_t payload_size;
    uint16_t crc_received, crc_calculated;

    // Verifica o tamanho do payload
    payload_size = (recv_buffer[11] << 8) | recv_buffer[12];

    // Calcula e verifica o CRC
    crc_received = (recv_buffer[MFP_PROTOCOL_HEADER + payload_size] << 8) | recv_buffer[MFP_PROTOCOL_HEADER + payload_size + 1];
    crc_calculated = calculate_crc16(recv_buffer, MFP_PROTOCOL_HEADER + payload_size);

    if (crc_received != crc_calculated) {
        ESP_LOGE(TAG, "CRC mismatch: received=0x%04X, calculated=0x%04X", crc_received, crc_calculated);
        return DATA_CHECK_FAIL_CRC;
    }

    if (payload_size == 0 || payload_size > PAYLOAD_MAX_SIZE) {
        ESP_LOGE(TAG, "Payload size out of bounds: %02X", payload_size);
        return DATA_CHECK_FAIL_PAYLOAD_SIZE;
    }

    if (recv_buffer[0] != 0xA1 || recv_buffer[7] != 0xA2 || recv_buffer[10] != 0xA3) {
        ESP_LOGE(TAG, "Invalid packet format for flags A1, A2 or A3");
        return DATA_CHECK_FAIL_FORMAT;
    }

    // Cria a estrutura de mensagem dinâmica
    message_t dyn_msg;
    dyn_msg.buffer = (uint8_t *)malloc(len * sizeof(uint8_t));
    if (dyn_msg.buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for dyn_msg.buffer");
        return DATA_CHECK_FAIL_MEMORY;
    }

    memcpy(dyn_msg.buffer, recv_buffer, len);
    dyn_msg.length = len;

    // Tenta enviar para a fila
    if (xQueueSend(queue, &dyn_msg, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to send data to queue");
        free(dyn_msg.buffer);
        return DATA_CHECK_FAIL_QUEUE;
    }

    ESP_LOGI(TAG, "Data processed and sent to queue successfully");
    return DATA_CHECK_SUCCESS;
}
