
#include "TCP_Server.h"

// Declare a global TaskHandle_t to store the task's handle
TaskHandle_t TCP_Server_task_handle = NULL;
QueueHandle_t send_message_TCP_q;
extern QueueHandle_t send_message_mesh_q;


static const char *TAG = "TCP_SOCKET";

/**
 * @brief Initializes a TCP server socket.
 * 
 * This function configures a TCP server socket by binding it to a specified port
 * and setting it to listen for incoming client connections. The socket is created
 * using IPv4 and TCP protocol. It also sets socket options for reusability.
 * 
 * @return 
 *     - The file descriptor of the listening socket on success.
 *     - `-1` if an error occurs during socket creation, binding, or listening.
 */
int TCP_Server_Init()
{
    struct sockaddr_in server_addr;

    struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&server_addr;
    dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr_ip4->sin_family = AF_INET;
    dest_addr_ip4->sin_port = htons(PORT);

    // Open socket
    int listen_sock = socket(AF_INET, SOCK_STREAM, 0); // 0 for TCP Protocol
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Error creating socket: errno %d", errno);
        return -1;
    }

    // Set socket options
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // Bind socket
    if (bind(listen_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) != 0) {
        ESP_LOGE(TAG, "Error binding socket: errno %d", errno);
        close(listen_sock);
        return -1;
    }
    ESP_LOGI(TAG, "Socket bound to port %d", PORT);

    // Listen to the socket
    if (listen(listen_sock, 1) != 0) {
        ESP_LOGE(TAG, "Error listening on socket: errno %d", errno);
        close(listen_sock);
        return -1;
    }

    send_message_TCP_q = xQueueCreate(QUEUE_SIZE, sizeof(message_t));
    if (send_message_TCP_q == NULL){
        ESP_LOGE(TAG, "Failed to create queue");
        return -1;
    }

    ESP_LOGI(TAG, "Socket is now listening");
    return listen_sock;
}

/**
 * @brief Function to create the task if it doesn't exist
 */
void create_TCP_Server_task()
{
    if(TCP_Server_task_handle == NULL)
    {
        xTaskCreate(tcp_server_task, "tcp_server_task", 4096, NULL, configMAX_PRIORITIES - 3, &TCP_Server_task_handle);
        printf("Task tcp_server_task created.\n");
    }
    else{
        //printf("Task tcp_server_task already exists.\n");
    }
}

void tcp_server_task(void *pvParameters)
{
    uint8_t rx_buffer[TCP_BUFFER];
    struct sockaddr_storage client_addr;
    socklen_t addr_len = sizeof(client_addr);

    int keepAlive = 1;
    int keepIdle = 5;
    int keepInterval = 5;
    int keepCount = 3;

    // Initialize the TCP server
    int listen_sock = TCP_Server_Init();
    if (listen_sock < 0) {
        vTaskDelete(NULL);
    }

    for(;;)
    {
        int client_sock = accept(listen_sock, (struct sockaddr *)&client_addr, &addr_len);
        if ( client_sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: erro %d", errno);
            continue;
        }

        ESP_LOGI(TAG, "Client connected");

        // Set tcp keepalive option
        setsockopt(client_sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
        setsockopt(client_sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
        setsockopt(client_sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
        setsockopt(client_sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));

        int len = recv(client_sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
        if (len < 0) 
        {
            ESP_LOGE(TAG, "Error receiving data: errno %d", errno);
            close(client_sock);
            continue;
        }

        ESP_LOGI(TAG, "Received message: ");
        for (int i = 0; i < len; i++) {
            printf("%02X ", rx_buffer[i]);
        }
        printf("\n");

        data_check_status_e dataStatus = process_data_send_mesh_queue(rx_buffer, len, send_message_mesh_q);

        if (dataStatus == DATA_CHECK_SUCCESS) {
            ESP_LOGI(TAG, "Data successfully processed and sent to queue.");
        } else {
            ESP_LOGE(TAG, "Failed to process data. Error code: %d", dataStatus);
        }
        message_t messageTCP;
        if(xQueueReceive(send_message_TCP_q, &messageTCP, WAIT_RESPONSE_TIMEOUT / portTICK_PERIOD_MS))
        {
            send(client_sock, messageTCP.buffer, messageTCP.length, 0);
            ESP_LOGI(TAG, "Data successfully sent to TCP Client.");
        }
        else{
            ESP_LOGI(TAG, "Failed to send the message to TCP Client.");
        }

        shutdown(client_sock, 0);
        close(client_sock);
        UBaseType_t highWaterMark = uxTaskGetStackHighWaterMark(NULL);
        ESP_LOGI("tcp_server_task", "Stack high water mark: %d words", highWaterMark);
    }//for(;;) 
}//static void tcp_server_task(void *pvParameters)

