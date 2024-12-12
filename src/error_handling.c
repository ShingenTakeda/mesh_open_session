#include "error_handling.h"

static const char *TAG = "MESH Error: ";

void mesh_error_handling (esp_err_t err)
{
    switch (err) {
        case ESP_ERR_MESH_WIFI_NOT_START:
            ESP_LOGE(TAG, "Wi-Fi isn't started");
            break;
        case ESP_ERR_MESH_NOT_INIT:
            ESP_LOGE(TAG, "Mesh isn't initialized");
            break;
        case ESP_ERR_MESH_NOT_CONFIG:
            ESP_LOGE(TAG, "Mesh isn't configured");
            break;
        case ESP_ERR_MESH_NOT_START:
            ESP_LOGE(TAG, "Mesh isn't started");
            break;
        case ESP_ERR_MESH_NOT_SUPPORT:
            ESP_LOGE(TAG, "Not supported yet");
            break;
        case ESP_ERR_MESH_NOT_ALLOWED:
            ESP_LOGE(TAG, "Operation is not allowed");
            break;
        case ESP_ERR_MESH_NO_MEMORY:
            ESP_LOGE(TAG, "Out of memory");
            break;
        case ESP_ERR_MESH_ARGUMENT:
            ESP_LOGE(TAG, "Illegal argument");
            break;
        case ESP_ERR_MESH_EXCEED_MTU:
            ESP_LOGE(TAG, "Packet size exceeds MTU");
            break;
        case ESP_ERR_MESH_TIMEOUT:
            ESP_LOGE(TAG, "Timeout");
            break;
        case ESP_ERR_MESH_DISCONNECTED:
            ESP_LOGE(TAG, "Disconnected with parent on station interface");
            break;
        case ESP_ERR_MESH_QUEUE_FAIL:
            ESP_LOGE(TAG, "Queue fail");
            break;
        case ESP_ERR_MESH_QUEUE_FULL:
            ESP_LOGE(TAG, "Queue full");
            break;
        case ESP_ERR_MESH_NO_PARENT_FOUND:
            ESP_LOGE(TAG, "No parent found to join the mesh network");
            break;
        case ESP_ERR_MESH_NO_ROUTE_FOUND:
            ESP_LOGE(TAG, "No route found to forward the packet");
            break;
        case ESP_ERR_MESH_OPTION_NULL:
            ESP_LOGE(TAG, "No option found");
            break;
        case ESP_ERR_MESH_OPTION_UNKNOWN:
            ESP_LOGE(TAG, "Unknown option");
            break;
        case ESP_ERR_MESH_XON_NO_WINDOW:
            ESP_LOGE(TAG, "No window for software flow control on upstream");
            break;
        case ESP_ERR_MESH_INTERFACE:
            ESP_LOGE(TAG, "Low-level Wi-Fi interface error");
            break;
        case ESP_ERR_MESH_DISCARD_DUPLICATE:
            ESP_LOGE(TAG, "Discard the packet due to duplicate sequence number");
            break;
        case ESP_ERR_MESH_DISCARD:
            ESP_LOGE(TAG, "Discard the packet");
            break;
        case ESP_ERR_MESH_VOTING:
            ESP_LOGE(TAG, "Vote in progress");
            break;
        case ESP_ERR_MESH_XMIT:
            ESP_LOGE(TAG, "XMIT");
            break;
        case ESP_ERR_MESH_QUEUE_READ:
            ESP_LOGE(TAG, "Error in reading queue");
            break;
        case ESP_ERR_MESH_PS:
            ESP_LOGE(TAG, "Mesh PS is not specified as enable or disable");
            break;
        case ESP_ERR_MESH_RECV_RELEASE:
            ESP_LOGE(TAG, "Release esp_mesh_recv_toDS");
            break;
        default:
            ESP_LOGE(TAG, "Unknown mesh error: 0x%x", err);
            break;
    }

}