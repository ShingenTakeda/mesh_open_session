#ifndef UART_H_
#define UART_H_

#pragma once

#define UART_NUM_UART0      UART_NUM_0   // Default UART for USB communication
#define UART_NUM_UART2      UART_NUM_2   // UART for meter communication

#define BUF_SIZE            1024         // Buffer sizer USB UART

// Function prototypes
void start_uart(void);
void uart_init(void);
void uart_rcv_task(void *pvParameters);
void nock_loose_task(void *pvParameters);


#endif /* UART_H_ */