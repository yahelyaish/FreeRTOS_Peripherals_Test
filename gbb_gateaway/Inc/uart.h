#ifndef UART_H
#define UART_H

#include <stdint.h>
#include "packet.h"

int uart_open(const char *device, int baudrate);
int uart_write(int fd, const packet_t *data, int len);
int uart_read(int fd, void *buf, int len);
int uart_wait_for_response(int fd, packet_t *rx_buffer, int len);
void uart_close(int fd);

#endif //UART_H