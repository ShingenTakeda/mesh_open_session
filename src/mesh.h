#ifndef MESH_H_
#define MESH_H_

#include "esp_mesh.h"

#pragma once

/*******************************************************
 *                Macros
 *******************************************************/
#define MAC_ADDR_LEN (6u)
#define MAC_ADDR_EQUAL(a, b) (0 == memcmp(a, b, MAC_ADDR_LEN))


/*******************************************************
 *                Type Definitions
 *******************************************************/
typedef void (mesh_raw_recv_cb_t)(mesh_addr_t *from, mesh_data_t *data);

/*******************************************************
 *                Function Declarations
 *******************************************************/
void create_mesh_p2p_tasks();
void delete_send_mesh_p2p_task();
void delete_rcv_mesh_p2p_task();
void check_Main_Tasks(void *pvParameters);
esp_err_t esp_mesh_comm_p2p_start(void);
void mesh_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
void start_mesh(char* router_ssid, char* router_password, uint8_t mesh_id[6], char* mesh_password);
void ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);




#endif // MESH_H_