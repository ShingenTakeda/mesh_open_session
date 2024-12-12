#ifndef ABNT_H_
#define ABNT_H_

#include <stdint.h>

#define DATA_SEND_LENGTH_ABNT 64
#define TOTAL_DATA_SEND_LEGTH_ABNT 66
#define DATA_RECEIVED_LENGTH_ABNT 258

void send_ABNT_14522_command(uint8_t *data, size_t length, uint8_t *recv_buffer_rx);

#endif /* ABNT_H_ */