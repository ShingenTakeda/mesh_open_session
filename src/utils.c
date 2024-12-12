#include <mbedtls/sha256.h>
#include "utils.h"

static const char *TAG = "Utils: ";

extern uint8_t my_mac[6];
extern QueueHandle_t send_message_mesh_q;

/**
 * @brief  Calculates the CRC16 checksum for a given data buffer.
 * 
 * This function computes the CRC16 checksum using a polynomial of 
 * 0xA001 (CRC-16/IBM or CRC-16/ARC). It processes each byte ofthe input data,
 * performing bitwise operations to calculate the final CRC value. 
 * The initial CRC value is set to 0x0000.
 * 
 * @param  data    Pointer to the input data buffer for which the CRC will be calculated.
 * @param  length  The length of the data buffer in bytes.
 * 
 * @return  The calculated 16-bit CRC value.
 */
uint16_t calculate_crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0x0000; // CRC inicializado com 0
    for (int i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief Tranforma uma String MAC address em array de byte d0:ef:76:44:ec:18 -> D0 EF 76 44 EC 18 
 *
 * @param mac_str Ponteiro para os dados de entrada.
 * @param mac_bytes Vetor aonde será armazenado esses bytes
 *
 */
void mac_str_to_bytes(const char *mac_str, uint8_t *mac_bytes) {
    for (int i = 0; i < 6; ++i) {
        sscanf(mac_str + 3 * i, "%2hhx", &mac_bytes[i]);
    }
}

void mac_bytes_to_string(uint8_t mac[6], char *mac_str) {
    sprintf(mac_str, MACSTR, MAC2STR(mac));
}

int hexstr_to_bytes(const char *hexstr, uint8_t *bytes, size_t max_len) {
    size_t hexstr_len = strlen(hexstr);
    size_t bytes_len = hexstr_len / 2;

    if (bytes_len > max_len) {
        return -1; // Erro: tamanho do buffer insuficiente
    }

    for (size_t i = 0; i < bytes_len; i++) {
        sscanf(&hexstr[i * 2], "%2hhx", &bytes[i]);
    }

    return bytes_len;
}

// Função para converter um buffer de bytes em uma string hexadecimal
void bytes_to_hexstr(const uint8_t *bytes, char *hexstr, size_t length) {
    for (size_t i = 0; i < length; i++) {
        sprintf(hexstr + i * 2, "%02X", bytes[i]);
    }
    hexstr[length * 2] = '\0'; // Adiciona o terminador nulo ao final da string
}

/**
 * @brief Converts a nibble from a byte into its decimal equivalent in BCD format.
 * 
 * This function extracts either the most significant nibble (MSB) or the least significant nibble (LSB)
 * from a byte and converts it to a decimal representation. If the nibble represents a hexadecimal value
 * from 'A' to 'F' (10 to 15), it is adjusted to its Binary-Coded Decimal (BCD) equivalent.
 * 
 * @param byte The input byte containing the nibble to be processed.
 * @param MSB A boolean value indicating which nibble to process:
 *            - `true`: Extracts the most significant nibble (MSB).
 *            - `false`: Extracts the least significant nibble (LSB).
 * 
 * @return The decimal equivalent of the selected nibble in BCD format.
 * 
 * @example
 * For the byte `0xAB`:
 * - `nibble_to_decimal_byte(0xAB, true)` returns `0x10` (decimal representation of MSB).
 * - `nibble_to_decimal_byte(0xAB, false)` returns `0x11` (decimal representation of LSB).
 */
uint8_t nibble_to_decimal_byte(uint8_t byte, bool MSB) {
    byte = MSB ? ((byte & 0xF0) >> 4) : (byte & 0x0F);
    byte = (byte >= 0x0A) ? (0x10 | (byte - 0x0A)) : byte;
    return byte;
}


void update_abnt_pads(uint8_t * ipad, uint8_t *opad, uint8_t *key, int key_len){
    uint8_t IPAD_VALUE = 0x36;
    uint8_t OPAD_VALUE = 0x5c;

    for(int i = 0; i < 64; i++){
        ipad[i] = IPAD_VALUE;
        opad[i] = OPAD_VALUE;
    }

    for(int i = 0; i < key_len; ++i){
        ipad[i] ^= key[i];
        opad[i] ^= key[i];
    }
}

uint8_t *hmac_abnt(uint8_t *key, size_t key_length, uint8_t *msg, size_t msg_len)
{
    uint8_t *aux  = (uint8_t *)malloc((msg_len + 64) * sizeof(uint8_t));
    uint8_t *hash = (uint8_t *)malloc(sizeof(uint8_t) * 32);

    uint8_t t_ipad[64];
    uint8_t t_opad[64];

    update_abnt_pads(t_ipad, t_opad, key, key_length);

    memcpy(aux, t_ipad, 64);

    for(int i = 0; i < msg_len; i++)
    {
        aux[64 + i] = msg[i];
    }

    mbedtls_sha256(aux, msg_len + 64, hash, 0);

    free(aux);
    aux = (uint8_t *)malloc((msg_len + 64) * sizeof(uint8_t));

    memcpy(aux, t_opad, 64);
    
    for(int i = 0; i < 32; i++)
    {
        aux[64 + i] = hash[i];
    }
    
    mbedtls_sha256(aux, msg_len + 64, hash, 0);

    free(aux);
    return hash;
}

uint16_t replace_byte(uint16_t value, uint8_t b, uint16_t pos)
{
    return (value & ~(0xFF << (pos * 8))) | ((b & 0xFF) << (pos * 8));
}