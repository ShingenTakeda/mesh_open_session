#ifndef MFP_PROTOCOL_
#define MFP_PROTOCOL_

#include "utils.h"
#include "esp_log.h"

#define MFP_PROTOCOL_HEADER 13 // a1 02 03 04 05 06 07 a2 09 10 a3 12 13
#define PAYLOAD_MAX_SIZE 1024

#define QUESTION 0x55
#define ANSWER 0xAA

#define TAG_1 0XA1
#define TAG_2 0XA2
#define TAG_3 0XA3

#define ABNT_PROTOCOL 0x0F
#define DLMS_PROTOCOL 0x1F
#define MESH_PROTOCOL 0x2F

typedef struct {
    uint8_t mac_address[6];
    uint8_t protocol_type[2];
    uint16_t  payload_size;
    uint8_t payload[1024];
} command_t;

typedef struct {
    uint8_t *buffer;
    size_t length;
} message_t;


typedef enum {
    ABNT,
    DLMS,
    MESH,
    INVALID_TYPE
} communication_type_e;

typedef enum {
    DATA_CHECK_SUCCESS = 0,
    DATA_CHECK_FAIL_CRC,
    DATA_CHECK_FAIL_PAYLOAD_SIZE,
    DATA_CHECK_FAIL_FORMAT,
    DATA_CHECK_FAIL_QUEUE,
    DATA_CHECK_FAIL_MEMORY,
    DATA_CHECK_FAIL_UNKNOWN
} data_check_status_e;


communication_type_e get_communication_type(uint8_t byte);
void write_meter_response_mesh_queue(uint8_t *data, uint16_t length);
data_check_status_e process_data_send_mesh_queue(uint8_t *recv_buffer, int len, QueueHandle_t queue);

#endif //#ifndef MFP_PROTOCOL_