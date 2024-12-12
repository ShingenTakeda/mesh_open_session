#include <string.h>
#include "esp_wifi.h"
#include "esp_mesh.h"
#include "esp_log.h"
#include "nvs_flash.h"


#include "MFP_Protocol.h"
#include "mesh.h"
#include "TCP_Server.h"
#include "uart.h"
#include "utils.h"
#include "error_handling.h"
#include "common.h"

#define MESH_RCV_SIZE          (1024 + MFP_PROTOCOL_HEADER + 2) // 2 -> CRC16
#define MESH_TRM_SIZE          (1024 + MFP_PROTOCOL_HEADER + 2) 

/*******************************************************
 *                Macros
 *******************************************************/
#define MAX_MESH_AP_CONNECTIONS_ON_EACH_NODE 2 //CONFIG_MESH_AP_CONNECTIONS

/*******************************************************
 *                Variable Definitions
 *******************************************************/
static const char *TAG = "MESH_NETWORK";
//static uint8_t tx_buf[TX_SIZE] = { 0 };
//static uint8_t rx_buf[RX_SIZE] = { 0 };
static bool is_mesh_connected = false;
static int mesh_layer = -1;
static esp_netif_t *netif_sta = NULL;

extern uint8_t my_mac[6];

QueueHandle_t send_message_mesh_q;
QueueHandle_t rcv_message_mesh_q;

//node_addr é o endereço relacionado ao WiFi no modo estação 'STA_MAC_address'
//parent_addr do ponto de vista de um node, é o WiFi do pai como ponto de acesso a malha 'AP_MESH_address'
//onde o endereço mac do modo AP é 1 unidade maior do que o mac no modo STA

static mesh_addr_t PARENT_AP_MESH_address;
static mesh_addr_t PARENT_STA_MESH_address;

// Declare a global TaskHandle_t to store the task's handle
TaskHandle_t send_mesh_p2p_task_handle = NULL;
TaskHandle_t rcv_mesh_p2p_task_handle = NULL;


void send_mesh_p2p_task(void *pvParameters)
{
    //message_t cmd;
    message_t dyn_msg;
    //uint8_t *data = NULL;;
    mesh_data_t mesh_data;
    mesh_addr_t dest_addr;
    mesh_data.proto = MESH_PROTO_BIN;
    mesh_data.tos = MESH_TOS_P2P;
    // mesh_data.data = data;
    // mesh_data.size = sizeof(data);

    while (1) {
        if (xQueueReceive(send_message_mesh_q, &dyn_msg, (TickType_t)portMAX_DELAY)) 
        {

            // ESP_LOGI(TAG, "Recebi essa quantidade de dados PARA ENVIAR NA MESH: %d e sao eles: ", dyn_msg.length);
            // for (int i = 0; i < dyn_msg.length; i++)
            // {
            //     printf("%02X ", dyn_msg.buffer[i]);
            // }
            // printf("\n\n");

            memcpy(dest_addr.addr, &dyn_msg.buffer[1], 6); // Get the MAC Address
            // ESP_LOGI(TAG, "O Endereco MAC de destino eh esse: %02X:%02X:%02X:%02X:%02X:%02X", 
            //              dest_addr.addr[0], dest_addr.addr[1], dest_addr.addr[2], 
            //              dest_addr.addr[3], dest_addr.addr[4], dest_addr.addr[5]);
            

            mesh_data.data = (uint8_t *)malloc(dyn_msg.length * sizeof(uint8_t)); // Aloca memória para a cópia dos dados
            if (mesh_data.data == NULL)
            {
                ESP_LOGE(TAG, "Falha ao alocar memória para mesh_data.data");
                free(dyn_msg.buffer); // Libera o buffer da mensagem antes de sair
                break;
            }

            mesh_data.size = dyn_msg.length;
            memcpy(mesh_data.data, dyn_msg.buffer,  mesh_data.size);
            //memcpy(&mesh_data.data, dyn_msg.buffer, mesh_data.size);

           
            // ESP_LOGI(TAG, "Mesh data size: %d", mesh_data.size);
            // ESP_LOGI(TAG, "Mesh data: ");
            // for (int i = 0; i < mesh_data.size; i++)
            // {
            //     printf("%02X ", mesh_data.data[i]);
            // }
            // printf("\n\n");

            // Send data via mesh network
            esp_err_t err;
            if(dyn_msg.buffer[9] == QUESTION)
            {
                ESP_LOGD(TAG, "Send a Question");
                err = esp_mesh_send(&dest_addr, &mesh_data, MESH_DATA_P2P, NULL, 0);/* @attention check the errors to return proper messages  */
            }//  if(dyn_msg.buffer[9] == QUESTION)
            else if (dyn_msg.buffer[9] == ANSWER)
            {
                ESP_LOGD(TAG, "Send a Answer");
                err = esp_mesh_send(NULL, &mesh_data, MESH_DATA_P2P, NULL, 0);
            } else{
                ESP_LOGD(TAG, "Protocol not defined yet");
            }
            if(err != ESP_OK){
                mesh_error_handling(err);
            }

            free(dyn_msg.buffer);
            free(mesh_data.data); // Libera a memória do mesh_data.data

            UBaseType_t highWaterMark = uxTaskGetStackHighWaterMark(NULL);
            ESP_LOGI("send_mesh_p2p_task", "Stack high water mark: %d words", highWaterMark);

        }// if (xQueueReceive(send_message_mesh_q, &cmd, (TickType_t)portMAX_DELAY))
    }// while (1)

    delete_send_mesh_p2p_task();
}

void rcv_mesh_p2p_task(void *pvParameters)
{
    message_t dyn_msg;
    uint8_t mesh_buff[MESH_RCV_SIZE];

    mesh_addr_t from_addr;
    mesh_data_t mesh_data;
    int flag = 0;

    uint8_t target_mac[6];
    uint16_t  payload_size, crc_received, crc_calculated;

    while(1)
    {
        mesh_data.data = mesh_buff;
        mesh_data.size = MESH_RCV_SIZE;
         // Receive mesh data
        esp_err_t err = esp_mesh_recv(&from_addr, &mesh_data, (TickType_t)portMAX_DELAY, &flag, NULL, 0);
        if (err == ESP_OK) 
        {
            // Extract MAC address from received data (next part after tag A1)
            memcpy(target_mac, &mesh_data.data[1], 6); //Extract mac address

            // Check if the destination MAC address matches the device's MAC
            if ((memcmp(target_mac, meter_attrib.mac, 6) == 0) || mesh_data.data[9] == ANSWER)
            {
                crc_received = (mesh_data.data[mesh_data.size - 2] << 8) | mesh_data.data[mesh_data.size - 1]; //get the received CRC
                crc_calculated = calculate_crc16(mesh_data.data, mesh_data.size - 2); // Calculates the CRC
                if (crc_received == crc_calculated)
                {
                    payload_size = (mesh_data.data[11] << 8) | mesh_data.data[12]; // get the payload size
                    if (payload_size > 0 && payload_size <= 1024) // Checks if the payload size is valid
                    {
                        if (mesh_data.data[0] == 0xA1 && mesh_data.data[7] == 0xA2 && mesh_data.data[10] == 0xA3)
                        {

                            ESP_LOGI(TAG, "Received data from MESH (size=%d):", mesh_data.size);
                            for (int i = 0; i < mesh_data.size; i++) {
                                printf("%02X ", mesh_data.data[i]);
                            }//for (int i = 0; i < data.size; i++)
                            printf("\n\n");

                            dyn_msg.buffer = (uint8_t *)malloc(mesh_data.size * sizeof(uint8_t)); // Aloca memória para a cópia dos dados
                            if (dyn_msg.buffer == NULL)
                            {
                                ESP_LOGE(TAG, "Falha ao alocar memória para dyn_msg.buffer");
                                break;
                            }

                            memcpy(dyn_msg.buffer, mesh_data.data, mesh_data.size); // Copia os dados para o buffer dinâmico
                            dyn_msg.length = mesh_data.size;

                            // Put data into the queue
                            if (xQueueSend(rcv_message_mesh_q, &dyn_msg, pdMS_TO_TICKS(100)) != pdTRUE)
                            {
                                ESP_LOGE(TAG, "Failed to send data to queue");
                                free(dyn_msg.buffer);
                            }                            

                        } else {
                            ESP_LOGE(TAG, "Invalid packet format for flags A1, A2 or A3");
                        }//if (recv_buffer[0] == 0xA1 && recv_buffer[7] == 0xA2 && recv_buffer[10] == 0xA3)
                    } else {
                        ESP_LOGE(TAG, "Payload with size outside of expected, please send something with size between 0 and 1024. Size sent: %02X", payload_size);
                    }//if (payload_size > 0 && payload_size <= 1024) 
                } else {
                    ESP_LOGE(TAG, "CRC mismatch: received=0x%04X, calculated=0x%04X", crc_received, crc_calculated);   
                }// if (crc_received == crc_calculated)
            } else {
                ESP_LOGE(TAG, "Im not the device you want!");
                ESP_LOGI(TAG, "My MAC: "MACSTR"", MAC2STR(meter_attrib.mac));
                ESP_LOGI(TAG, "Target MAC: "MACSTR"", MAC2STR(target_mac));
            }// if (memcmp(target_mac, my_mac, 6) == 0)
        } else {
           mesh_error_handling(err);
        }

        UBaseType_t highWaterMark = uxTaskGetStackHighWaterMark(NULL);
        ESP_LOGI("rcv_mesh_p2p_task", "Stack high water mark: %d words", highWaterMark);

        //free(dyn_msg.buffer);
    }//while(1)
    delete_rcv_mesh_p2p_task();
    
}
/**
 * @brief Function to create the task if it doesn't exist
 *
 */
void create_mesh_p2p_tasks()
{
    if(send_mesh_p2p_task_handle == NULL)
    {
        xTaskCreate(send_mesh_p2p_task, "mesh_p2p_tx_task", 3072, NULL, configMAX_PRIORITIES - 5, &send_mesh_p2p_task_handle);
        printf("Task send_mesh_p2p_task created.\n");
    }
    else{
        //printf("Task send_mesh_p2p_task already exists.\n");
    }
    if(rcv_mesh_p2p_task_handle == NULL)
    {
        xTaskCreate(rcv_mesh_p2p_task, "mesh_p2p_rx_task", 4096, NULL, configMAX_PRIORITIES - 4, &rcv_mesh_p2p_task_handle);
        printf("Task rcv_mesh_p2p_task created.\n");
    }
    else{
       // printf("Task rcv_mesh_p2p_task already exists.\n");
    }
}
/**
 * @brief Function to delete the task if it's running
 *
 */
void delete_send_mesh_p2p_task()
{
    // Check if the task handle is valid
    if (send_mesh_p2p_task_handle != NULL) {
        // Delete the task and set the handle to NULL
        vTaskDelete(send_mesh_p2p_task_handle);
        send_mesh_p2p_task_handle = NULL;
        printf("Task send_mesh_p2p_task_handle deleted.\n");
    } else {
        printf("Task send_mesh_p2p_task_handle does not exist.\n");
    }

}
/**
 * @brief Function to delete the task if it's running
 */
void delete_rcv_mesh_p2p_task()
{
    // Check if the task handle is valid
    if (rcv_mesh_p2p_task_handle != NULL) {
        // Delete the task and set the handle to NULL
        vTaskDelete(rcv_mesh_p2p_task_handle);
        rcv_mesh_p2p_task_handle = NULL;
        printf("Task rcv_mesh_p2p_task_handle deleted.\n");
    } else {
        printf("Task rcv_mesh_p2p_task_handle does not exist.\n");
    }

}
/**
 * @brief check_Main_Tasks
 */
void check_Main_Tasks(void *pvParameters)
{
    for(;;)
    {
        //create_send_mesh_p2p_task();
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

esp_err_t esp_mesh_comm_p2p_start(void)
{
    static bool is_comm_p2p_started = false;
    if (!is_comm_p2p_started) {
        is_comm_p2p_started = true;
        create_mesh_p2p_tasks();
       
    }
    return ESP_OK;
}

void mesh_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    mesh_addr_t id = {0,};
    static uint16_t last_layer = 0;

    switch (event_id) {
    case MESH_EVENT_STARTED: {
        esp_mesh_get_id(&id);
        ESP_LOGI(TAG, "<MESH_EVENT_MESH_STARTED>ID:"MACSTR"", MAC2STR(id.addr));
        is_mesh_connected = false;
        mesh_layer = esp_mesh_get_layer();
    }
    break;
    case MESH_EVENT_STOPPED: {
        ESP_LOGI(TAG, "<MESH_EVENT_STOPPED>");
        is_mesh_connected = false;
        mesh_layer = esp_mesh_get_layer();
    }
    break;
    case MESH_EVENT_CHILD_CONNECTED: {
        mesh_event_child_connected_t *child_connected = (mesh_event_child_connected_t *)event_data;
        ESP_LOGI(TAG, "<MESH_EVENT_CHILD_CONNECTED>aid:%d, "MACSTR"",
                 child_connected->aid,
                 MAC2STR(child_connected->mac));
    }
    break;
    case MESH_EVENT_CHILD_DISCONNECTED: {
        mesh_event_child_disconnected_t *child_disconnected = (mesh_event_child_disconnected_t *)event_data;
        ESP_LOGI(TAG, "<MESH_EVENT_CHILD_DISCONNECTED>aid:%d, "MACSTR"",
                 child_disconnected->aid,
                 MAC2STR(child_disconnected->mac));
    }
    break;
    case MESH_EVENT_ROUTING_TABLE_ADD: {
        mesh_event_routing_table_change_t *routing_table = (mesh_event_routing_table_change_t *)event_data;
        ESP_LOGW(TAG, "<MESH_EVENT_ROUTING_TABLE_ADD>add %d, new:%d, layer:%d",
                 routing_table->rt_size_change,
                 routing_table->rt_size_new, mesh_layer);
    }
    break;
    case MESH_EVENT_ROUTING_TABLE_REMOVE: {
        mesh_event_routing_table_change_t *routing_table = (mesh_event_routing_table_change_t *)event_data;
        ESP_LOGW(TAG, "<MESH_EVENT_ROUTING_TABLE_REMOVE>remove %d, new:%d, layer:%d",
                 routing_table->rt_size_change,
                 routing_table->rt_size_new, mesh_layer);
    }
    break;
    case MESH_EVENT_NO_PARENT_FOUND: {
        mesh_event_no_parent_found_t *no_parent = (mesh_event_no_parent_found_t *)event_data;
        ESP_LOGI(TAG, "<MESH_EVENT_NO_PARENT_FOUND>scan times:%d",
                 no_parent->scan_times);
    }
    /* TODO handler for the failure */
    break;
    case MESH_EVENT_PARENT_CONNECTED: {
        mesh_event_connected_t *connected = (mesh_event_connected_t *)event_data;
        esp_mesh_get_id(&id);
        mesh_layer = connected->self_layer;
        memcpy(PARENT_AP_MESH_address.addr, connected->connected.bssid, 6);
        memcpy(PARENT_STA_MESH_address.addr, PARENT_AP_MESH_address.addr, 6);
        //O endereço mac no modo AP é 1 unidade maior do que o mac no modo STA
        PARENT_STA_MESH_address.addr[5] = PARENT_AP_MESH_address.addr[5] - 1;
        ESP_LOGI(TAG,
                 "<MESH_EVENT_PARENT_CONNECTED>layer:%d-->%d, parent: sta"MACSTR"/ ap"MACSTR" %s, ID:"MACSTR", duty:%d",
                 last_layer, mesh_layer, MAC2STR(PARENT_STA_MESH_address.addr), MAC2STR(PARENT_AP_MESH_address.addr),
                 esp_mesh_is_root() ? "<ROOT>" :
                 (mesh_layer == 2) ? "<layer2>" : "", MAC2STR(id.addr), connected->duty);
        last_layer = mesh_layer;
        
        // Get the device's MAC address
        esp_wifi_get_mac(ESP_IF_WIFI_STA, meter_attrib.mac);

        meter_attrib.device_id[0] = nibble_to_decimal_byte(meter_attrib.mac[4], false);
        meter_attrib.device_id[1] = nibble_to_decimal_byte(meter_attrib.mac[5], true);
        meter_attrib.device_id[2] = nibble_to_decimal_byte(meter_attrib.mac[5], false);

        is_mesh_connected = true;
        if (esp_mesh_is_root()) {
            esp_netif_dhcpc_stop(netif_sta);
            esp_netif_dhcpc_start(netif_sta);
            create_TCP_Server_task();
        }
        esp_mesh_comm_p2p_start();
        //xTaskCreate(check_Main_Tasks, "HEALTH_TASK", configMINIMAL_STACK_SIZE+1024, NULL, configMAX_PRIORITIES-3, NULL);
    }
    break;
    case MESH_EVENT_PARENT_DISCONNECTED: {
        mesh_event_disconnected_t *disconnected = (mesh_event_disconnected_t *)event_data;
        ESP_LOGI(TAG,
                 "<MESH_EVENT_PARENT_DISCONNECTED>reason:%d",
                 disconnected->reason);
        is_mesh_connected = false;
         //mesh_disconnected_indicator(); //LED RGB, setava uma cor de alerta pra dizer que nao esta conectado ao pai, logo nao esta em nenhuma camada.
        mesh_layer = esp_mesh_get_layer();
    }
    break;
    case MESH_EVENT_LAYER_CHANGE: {
        mesh_event_layer_change_t *layer_change = (mesh_event_layer_change_t *)event_data;
        mesh_layer = layer_change->new_layer;
        ESP_LOGI(TAG, "<MESH_EVENT_LAYER_CHANGE>layer:%d-->%d%s",
                 last_layer, mesh_layer,
                 esp_mesh_is_root() ? "<ROOT>" :
                 (mesh_layer == 2) ? "<layer2>" : "");
        last_layer = mesh_layer;
        //mesh_connected_indicator(mesh_layer); //LED RGB, setava a cor do LED de acordo com a camada do nó (Ex: camada 1: red, 2: verde, 3: azul, e assim por diante)
    }
    break;
    case MESH_EVENT_ROOT_ADDRESS: {
        mesh_event_root_address_t *root_addr = (mesh_event_root_address_t *)event_data;
        ESP_LOGI(TAG, "<MESH_EVENT_ROOT_ADDRESS>root address:"MACSTR"",
                 MAC2STR(root_addr->addr));
    }
    break;
    case MESH_EVENT_VOTE_STARTED: {
        mesh_event_vote_started_t *vote_started = (mesh_event_vote_started_t *)event_data;
        ESP_LOGI(TAG,
                 "<MESH_EVENT_VOTE_STARTED>attempts:%d, reason:%d, rc_addr:"MACSTR"",
                 vote_started->attempts,
                 vote_started->reason,
                 MAC2STR(vote_started->rc_addr.addr));
    }
    break;
    case MESH_EVENT_VOTE_STOPPED: {
        ESP_LOGI(TAG, "<MESH_EVENT_VOTE_STOPPED>");
        break;
    }
    case MESH_EVENT_ROOT_SWITCH_REQ: {
        mesh_event_root_switch_req_t *switch_req = (mesh_event_root_switch_req_t *)event_data;
        ESP_LOGI(TAG,
                 "<MESH_EVENT_ROOT_SWITCH_REQ>reason:%d, rc_addr:"MACSTR"",
                 switch_req->reason,
                 MAC2STR( switch_req->rc_addr.addr));
    }
    break;
    case MESH_EVENT_ROOT_SWITCH_ACK: {
        /* new root */
        mesh_layer = esp_mesh_get_layer();
        esp_mesh_get_parent_bssid(&PARENT_AP_MESH_address);
        ESP_LOGI(TAG, "<MESH_EVENT_ROOT_SWITCH_ACK>layer:%d, parent:"MACSTR"", mesh_layer, MAC2STR(PARENT_AP_MESH_address.addr));
    }
    break;
    case MESH_EVENT_TODS_STATE: {
        mesh_event_toDS_state_t *toDs_state = (mesh_event_toDS_state_t *)event_data;
        ESP_LOGI(TAG, "<MESH_EVENT_TODS_REACHABLE>state:%d", *toDs_state);
    }
    break;
    case MESH_EVENT_ROOT_FIXED: {
        mesh_event_root_fixed_t *root_fixed = (mesh_event_root_fixed_t *)event_data;
        ESP_LOGI(TAG, "<MESH_EVENT_ROOT_FIXED>%s",
                 root_fixed->is_fixed ? "fixed" : "not fixed");
    }
    break;
    case MESH_EVENT_ROOT_ASKED_YIELD: {
        mesh_event_root_conflict_t *root_conflict = (mesh_event_root_conflict_t *)event_data;
        ESP_LOGI(TAG,
                 "<MESH_EVENT_ROOT_ASKED_YIELD>"MACSTR", rssi:%d, capacity:%d",
                 MAC2STR(root_conflict->addr),
                 root_conflict->rssi,
                 root_conflict->capacity);
    }
    break;
    case MESH_EVENT_CHANNEL_SWITCH: {
        mesh_event_channel_switch_t *channel_switch = (mesh_event_channel_switch_t *)event_data;
        ESP_LOGI(TAG, "<MESH_EVENT_CHANNEL_SWITCH>new channel:%d", channel_switch->channel);
    }
    break;
    case MESH_EVENT_SCAN_DONE: {
        mesh_event_scan_done_t *scan_done = (mesh_event_scan_done_t *)event_data;
        ESP_LOGI(TAG, "<MESH_EVENT_SCAN_DONE>number:%d",
                 scan_done->number);
    }
    break;
    case MESH_EVENT_NETWORK_STATE: {
        mesh_event_network_state_t *network_state = (mesh_event_network_state_t *)event_data;
        ESP_LOGI(TAG, "<MESH_EVENT_NETWORK_STATE>is_rootless:%d",
                 network_state->is_rootless);
    }
    break;
    case MESH_EVENT_STOP_RECONNECTION: {
        ESP_LOGI(TAG, "<MESH_EVENT_STOP_RECONNECTION>");
    }
    break;
    case MESH_EVENT_FIND_NETWORK: {
        mesh_event_find_network_t *find_network = (mesh_event_find_network_t *)event_data;
        ESP_LOGI(TAG, "<MESH_EVENT_FIND_NETWORK>new channel:%d, router BSSID:"MACSTR"",
                 find_network->channel, MAC2STR(find_network->router_bssid));
    }
    break;
    case MESH_EVENT_ROUTER_SWITCH: {
        mesh_event_router_switch_t *router_switch = (mesh_event_router_switch_t *)event_data;
        ESP_LOGI(TAG, "<MESH_EVENT_ROUTER_SWITCH>new router:%s, channel:%d, "MACSTR"",
                 router_switch->ssid, router_switch->channel, MAC2STR(router_switch->bssid));
    }
    break;
    case MESH_EVENT_PS_PARENT_DUTY: {
        mesh_event_ps_duty_t *ps_duty = (mesh_event_ps_duty_t *)event_data;
        ESP_LOGI(TAG, "<MESH_EVENT_PS_PARENT_DUTY>duty:%d", ps_duty->duty);
    }
    break;
    case MESH_EVENT_PS_CHILD_DUTY: {
        mesh_event_ps_duty_t *ps_duty = (mesh_event_ps_duty_t *)event_data;
        ESP_LOGI(TAG, "<MESH_EVENT_PS_CHILD_DUTY>cidx:%d, "MACSTR", duty:%d", ps_duty->child_connected.aid-1,
                MAC2STR(ps_duty->child_connected.mac), ps_duty->duty);
    }
    break;
    default:
        ESP_LOGI(TAG, "unknown id:%" PRId32 "", event_id);
        break;
    }
}

void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    switch(event_id)
    {
        case WIFI_EVENT_STA_CONNECTED:
            ESP_LOGI(TAG, "<WIFI_EVENT_STA_CONNECTED>");
        break;

        case WIFI_EVENT_STA_DISCONNECTED:
            ESP_LOGI(TAG, "<WIFI_EVENT_STA_DISCONNECTED>");
        break;
    }

}

void ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{

    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    ESP_LOGI(TAG, "<IP_EVENT_STA_GOT_IP>IP:" IPSTR, IP2STR(&event->ip_info.ip));

    // ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;

    // switch(event_id)
    // {
    //     case IP_EVENT_STA_GOT_IP:
    //         ESP_LOGI(TAG, "<IP_EVENT_STA_GOT_IP>IP:" IPSTR, IP2STR(&event->ip_info.ip));
    //         if(esp_mesh_is_root()) {
    //             start_webserver(false);
    //         }
    //     break;

    //     case IP_EVENT_STA_LOST_IP:
    //         ESP_LOGW(TAG, "<IP_EVENT_STA_LOST_IP>");
    //     break;
    // }

}
/**
 * @brief  Initializes and starts the ESP-MESH network.
 * 
 * This function sets up the required network configurations and initializes
 * the ESP-MESH network. It configures the TCP/IP stack, event handling,
 * Wi-Fi, and mesh network parameters such as the router credentials, mesh ID,
 * and mesh softAP settings. If power-saving mode is enabled for mesh, it also
 * configures the appropriate duty cycle. The function ensures that all necessary
 * system components are initialized before starting the mesh network.
 * 
 * In the event handler function `mesh_event_handler`, tasks responsible for 
 * executing the data transmission and reception loops are created when mesh 
 * events occur. These tasks handle the sending and receiving of data over the 
 * mesh network, ensuring proper communication between nodes.
 * 
 * @param  router_ssid      The SSID of the router to connect the mesh root.
 * @param  router_password  The password for the router.
 * @param  mesh_id          A 6-byte identifier for the mesh network.
 * @param  mesh_password    The password for mesh network softAP authentication.
 * 
 * @return  This function does not return any value, but it logs the mesh startup
 *          status and will abort the program if any critical error occurs during
 *          initialization.
 */
void start_mesh(char* router_ssid, char* router_password, uint8_t mesh_id[6], char* mesh_password)
{

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    // Inicializa o armazenamento não volátil (NVS)
    // ESP_ERROR_CHECK(nvs_flash_init());

    /*  tcpip initialization */
    ESP_ERROR_CHECK(esp_netif_init());
    /*  event initialization */
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    /*  create network interfaces for mesh (only station instance saved for further manipulation, soft AP instance ignored */
    ESP_ERROR_CHECK(esp_netif_create_default_wifi_mesh_netifs(&netif_sta, NULL));
    /*  wifi initialization */
    wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&config));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
    ESP_ERROR_CHECK(esp_wifi_start());

    /*  mesh initialization */
    ESP_ERROR_CHECK(esp_mesh_init());
    ESP_ERROR_CHECK(esp_event_handler_register(MESH_EVENT, ESP_EVENT_ANY_ID, &mesh_event_handler, NULL));
    /*  set mesh topology */
    ESP_ERROR_CHECK(esp_mesh_set_topology(0)); //CONFIG_MESH_TOPOLOGY
    /*  set mesh max layer according to the topology */
    ESP_ERROR_CHECK(esp_mesh_set_max_layer(6)); //CONFIG_MESH_MAX_LAYER
    ESP_ERROR_CHECK(esp_mesh_set_vote_percentage(1));
    ESP_ERROR_CHECK(esp_mesh_set_xon_qsize(128));
 #ifdef CONFIG_MESH_ENABLE_PS
    /* Enable mesh PS function */
    ESP_ERROR_CHECK(esp_mesh_enable_ps());
    /* better to increase the associate expired time, if a small duty cycle is set. */
    ESP_ERROR_CHECK(esp_mesh_set_ap_assoc_expire(60));
    /* better to increase the announce interval to avoid too much management traffic, if a small duty cycle is set. */
    ESP_ERROR_CHECK(esp_mesh_set_announce_interval(600, 3300));
 #else
    /* Disable mesh PS function */
    ESP_ERROR_CHECK(esp_mesh_disable_ps());
    ESP_ERROR_CHECK(esp_mesh_set_ap_assoc_expire(10));
 #endif
    mesh_cfg_t cfg = MESH_INIT_CONFIG_DEFAULT();
    /* mesh ID */
    memcpy((uint8_t *) &cfg.mesh_id, mesh_id, 6);
    
    /* router */
    cfg.channel = 0; //CONFIG_MESH_CHANNEL
    cfg.router.ssid_len = strlen(router_ssid);
    memcpy((uint8_t *) &cfg.router.ssid, router_ssid, cfg.router.ssid_len);
    memcpy((uint8_t *) &cfg.router.password, router_password, strlen(router_password));

    /* mesh softAP */
    ESP_ERROR_CHECK(esp_mesh_set_ap_authmode(3)); //CONFIG_MESH_AP_AUTHMODE
    cfg.mesh_ap.max_connection = MAX_MESH_AP_CONNECTIONS_ON_EACH_NODE;
    cfg.mesh_ap.nonmesh_max_connection = 1; //CONFIG_MESH_NON_MESH_AP_CONNECTIONS
    if((cfg.mesh_ap.max_connection + cfg.mesh_ap.nonmesh_max_connection) > 10) {
        ESP_LOGE(TAG, "Config ultrapassa o maximo de conexoes permitidas");
        return;
    }
    memcpy((uint8_t *) &cfg.mesh_ap.password, mesh_password, strlen(mesh_password));
    ESP_ERROR_CHECK(esp_mesh_set_config(&cfg));
    /* mesh start */
    ESP_ERROR_CHECK(esp_mesh_start());
 #ifdef CONFIG_MESH_ENABLE_PS
    /* set the device active duty cycle. (default:10, MESH_PS_DEVICE_DUTY_REQUEST) */
    ESP_ERROR_CHECK(esp_mesh_set_active_duty_cycle(CONFIG_MESH_PS_DEV_DUTY, CONFIG_MESH_PS_DEV_DUTY_TYPE));
    /* set the network active duty cycle. (default:10, -1, MESH_PS_NETWORK_DUTY_APPLIED_ENTIRE) */
    ESP_ERROR_CHECK(esp_mesh_set_network_duty_cycle(CONFIG_MESH_PS_NWK_DUTY, CONFIG_MESH_PS_NWK_DUTY_DURATION, CONFIG_MESH_PS_NWK_DUTY_RULE));
 #endif
    ESP_LOGI(TAG, "mesh starts successfully, heap:%" PRId32 ", %s<%d>%s, ps:%d",  esp_get_minimum_free_heap_size(),
             esp_mesh_is_root_fixed() ? "root fixed" : "root not fixed",
             esp_mesh_get_topology(), esp_mesh_get_topology() ? "(chain)":"(tree)", esp_mesh_is_ps_enabled());
}