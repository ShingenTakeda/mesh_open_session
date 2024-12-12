#ifndef UTILS_H_
#define UTILD_H_

#include "MFP_Protocol.h"
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_mesh.h"
#include "esp_mac.h"
#include "esp_log.h"

#define QUEUE_SIZE 10

// Function prototypes

uint16_t calculate_crc16(const uint8_t *data, size_t length);
void mac_str_to_bytes(const char *mac_str, uint8_t *mac_bytes);
void mac_bytes_to_string(uint8_t mac[6], char *mac_str);
int hexstr_to_bytes(const char *hexstr, uint8_t *bytes, size_t max_len);
void bytes_to_hexstr(const uint8_t *bytes, char *hexstr, size_t length);
uint8_t nibble_to_decimal_byte(uint8_t nibble, bool MSB);
uint16_t replace_byte(uint16_t value, uint8_t b, uint16_t pos);

//Crypto
uint8_t *hmac_abnt(uint8_t *key, size_t key_length, uint8_t *msg, size_t msg_len);

#endif //#ifndef UTILS_H