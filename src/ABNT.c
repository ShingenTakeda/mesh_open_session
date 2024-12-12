#include "driver/uart.h"
#include "esp_log.h"
#include "ABNT.h"
#include "uart.h"
#include "utils.h"
#include "common.h"

uint8_t key[1] = {0x00};

static const char *TAG = "ABNT command: ";
extern uint8_t my_mac[6];

void set_abnt_data(uint8_t *data, uint8_t command, uint8_t *payload, uint8_t payload_len)
{
    for(int i = 0; i < 66; i++){
        switch(i){
            case 0:
                data[i] = command;
                break;
            case 1:
                data[i] = meter_attrib.device_id[0];
                break;
            case 2:
                data[i] = meter_attrib.device_id[1];
                break;
            case 3:
                data[i] = meter_attrib.device_id[2];
                break;
            default:
                data[i] = 0;
                break;
        }
    }

    if(payload != NULL){
        for(int i = 0; i < payload_len; i++){
            data[4 + i] = payload[i];
        }
    }
    
    uint16_t crc = calculate_crc16(data, 64);
    data[64] = ((crc >> 8) & 0xFF);
    data[65] = ((crc >> 0) & 0xFF);
}

void send_solved_string_with_cmd(uint8_t *ds, uint8_t retries, uint8_t *recv_buffer_rx)
{
    uint8_t open_msg[TOTAL_DATA_SEND_LEGTH_ABNT];
    uint8_t msg_to_solve[32];
    uint16_t crc = 0;
    uint16_t crc_checker = 0;
    set_abnt_data(open_msg, 0x13, NULL, 0);

    int len = uart_write_bytes(UART_NUM_UART2, open_msg, TOTAL_DATA_SEND_LEGTH_ABNT);
    if(len != TOTAL_DATA_SEND_LEGTH_ABNT)
    {
        ESP_LOGE("UART", "Erro ao enviar dados");
    }

    len = uart_read_bytes(UART_NUM_UART2, recv_buffer_rx, DATA_RECEIVED_LENGTH_ABNT,1000 / portTICK_PERIOD_MS);
    if(len > -1)
    {
        crc_checker = calculate_crc16(recv_buffer_rx, TOTAL_DATA_SEND_LEGTH_ABNT);
        crc = replace_byte(crc, recv_buffer_rx[DATA_RECEIVED_LENGTH_ABNT - 2], 1);
        crc = replace_byte(crc, recv_buffer_rx[DATA_RECEIVED_LENGTH_ABNT - 1], 0);

        if(crc != crc_checker)
        {
            ESP_LOGE("UART", "CRC não bate");
            return;
        }
    }
    else
    {
        ESP_LOGE("UART", "Erro recebendo dados");
        return;
    }

    for(int i = 0; i < 32; i++)
    {
        msg_to_solve[i] = recv_buffer_rx[5 + i];
    }

    uint8_t *hmac_str = hmac_abnt(key, 1, msg_to_solve, 32);

    set_abnt_data(open_msg, 0x11, hmac_str, 11);

    len = uart_write_bytes(UART_NUM_UART2, open_msg, TOTAL_DATA_SEND_LEGTH_ABNT);
    if(len != TOTAL_DATA_SEND_LEGTH_ABNT)
    {
        ESP_LOGE("UART", "Erro ao enviar dados");
    }

    len = uart_read_bytes(UART_NUM_UART2, recv_buffer_rx, DATA_RECEIVED_LENGTH_ABNT,1000 / portTICK_PERIOD_MS);
    if(len > -1)
    {
        crc_checker = calculate_crc16(recv_buffer_rx, TOTAL_DATA_SEND_LEGTH_ABNT);
        crc = replace_byte(crc, recv_buffer_rx[DATA_RECEIVED_LENGTH_ABNT - 2], 1);
        crc = replace_byte(crc, recv_buffer_rx[DATA_RECEIVED_LENGTH_ABNT - 1], 0);

        if(crc != crc_checker)
        {
            ESP_LOGE("UART", "CRC não bate");
            return;
        }

        if(crc == crc_checker && recv_buffer_rx[0] == 0x40)
        {
            ESP_LOGE("UART", "Secao travada");
            return;
        }

        if(crc == crc_checker && recv_buffer_rx[0] == 0x11)
        {
            ESP_LOGI("UART", "Mensagem validada, mandando dados...");
            len = uart_write_bytes(UART_NUM_UART2, ds, TOTAL_DATA_SEND_LEGTH_ABNT);
            if(len != TOTAL_DATA_SEND_LEGTH_ABNT)
            {
                ESP_LOGE("UART", "Erro ao enviar dados");
            }

            len = uart_read_bytes(UART_NUM_UART2, recv_buffer_rx, DATA_RECEIVED_LENGTH_ABNT, 1000 / portTICK_PERIOD_MS);
            if(len > -1)
            {
                crc_checker = calculate_crc16(recv_buffer_rx, TOTAL_DATA_SEND_LEGTH_ABNT);
                crc = replace_byte(crc, recv_buffer_rx[DATA_RECEIVED_LENGTH_ABNT - 2], 1);
                crc = replace_byte(crc, recv_buffer_rx[DATA_RECEIVED_LENGTH_ABNT - 1], 0);

                if(crc != crc_checker)
                {
                    ESP_LOGE("UART", "CRC não bate");
                    return;
                }

                for (int i = 0; i < 258; i++) {
                    printf("%02X ", recv_buffer_rx[i]);
                }
            }
            else
            {
                ESP_LOGE("UART", "Erro recebendo dados");
                return;
            }
            
        }
    }
    else
    {
        ESP_LOGE("UART", "Erro recebendo dados");
    }

}

void send_ABNT_14522_command(uint8_t *data,  size_t length, uint8_t *recv_buffer_rx)
{
    uint16_t  payload_size;

    ESP_LOGI(TAG, "CASO ABNT");

    payload_size = (data[11] << 8) | data[12]; // get the payload size
    ESP_LOGI(TAG, "Payload Size %d", payload_size);
    ESP_LOGI(TAG, "Length Size %d", payload_size);

    if(payload_size <= TOTAL_DATA_SEND_LEGTH_ABNT)
    {
        uint8_t  data_send[TOTAL_DATA_SEND_LEGTH_ABNT];
        
        data_send[0] = data[13];

        data_send[1] = meter_attrib.device_id[0];
        data_send[2] = meter_attrib.device_id[1];
        data_send[3] = meter_attrib.device_id[2];

        // Copia os dados do vetor data para o vetor data_send a partir de data_send[5]
        memcpy(&data_send[4], &data[14], payload_size - 1);

        //memcpy(data_send, &data[13], payload_size);
        memset(data_send + payload_size + 3, 0x00, DATA_SEND_LENGTH_ABNT - payload_size);

        uint16_t crc = calculate_crc16(data_send, DATA_SEND_LENGTH_ABNT);
        data_send[DATA_SEND_LENGTH_ABNT] = (uint8_t)(crc & 0xFF);            // Byte menos significativo
        data_send[DATA_SEND_LENGTH_ABNT + 1] = (uint8_t)((crc >> 8) & 0xFF); // Byte mais significativo

        printf("Bytes preparados para serem enviados: ");
        for (int i = 0; i < sizeof(data_send); i++) {
            printf("%02X ", data_send[i]);
        }
        printf("\n\n");

        // Envia os 68 bytes pela UART2
        int bytes_sent = uart_write_bytes(UART_NUM_UART2, (const char *)data_send, TOTAL_DATA_SEND_LEGTH_ABNT);
        if (bytes_sent == TOTAL_DATA_SEND_LEGTH_ABNT)
        {
            ESP_LOGI(TAG, "Dados enviados com sucesso pela UART2");
        }
        else
        {
            ESP_LOGE(TAG, "Erro ao enviar dados pela UART2");
        }
        // Wait for response from UART2
        int len = uart_read_bytes(UART_NUM_UART2, recv_buffer_rx, DATA_RECEIVED_LENGTH_ABNT, 1000 / portTICK_PERIOD_MS);
        
        if (len > 0) {
            printf("UART2 response: ");
            for (int i = 0; i < 258; i++) {
                printf("%02X ", recv_buffer_rx[i]);
            }
            if(recv_buffer_rx[0] == 0x40){
                
            }

        } else {
            printf("No response from UART2\n");
        }
        printf("\n\n");

        // Limpa os buffers para a próxima operação
        memset(data_send, 0, TOTAL_DATA_SEND_LEGTH_ABNT);

    } // if(payload_size <= TOTAL_DATA_SEND_LEGTH_ABNT)
} // send_ABNT_14522_command(uint8_t *data, size_t length)