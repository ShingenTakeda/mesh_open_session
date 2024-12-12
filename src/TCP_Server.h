#ifndef TCP_SERVER_H
#define TCP_SERVER_H

#include "esp_log.h"
#include "esp_wifi.h"
#include "lwip/sockets.h"
#include "MFP_Protocol.h"

#define TCP_BUFFER 1024 + 13 + 2 // MFP_PROTOCOL_HEADER + PAYLOAD_MAX_SIZE + CRC16
#define WAIT_RESPONSE_TIMEOUT 3000 //3S timeout to send commando to TCP client
// #define SSID "ELETRA350"
// #define PASS "01234567"

#define PORT 9999

int TCP_Server_Init();
void create_TCP_Server_task();
void tcp_server_task(void *pvParameters);

#endif // TCP_SERVER_H