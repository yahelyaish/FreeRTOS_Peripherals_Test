#ifndef PACKET_H
#define PACKET_H
#include <sys/types.h>
#include <stdint.h>
#include "stdio.h"
#include <netinet/in.h> 

#define PAYLOAD_MAX 256

typedef struct __attribute__((packed)){
    uint32_t test_id;        // מזהה בדיקה
    uint8_t  peripheral;    // UART / SPI / I2C
    uint8_t  iterations;    // כמה פעמים
    uint8_t  payload_len;   // אורך payload
    uint8_t  payload[PAYLOAD_MAX];
} packet_t;

void build_packet(packet_t *pkt, uint8_t peripheral, uint32_t test_id);
ssize_t send_packet_udp(int sock,const struct sockaddr_in *dest_addr,const packet_t *pkt);
void printPacket(const packet_t *pkt);
#endif
