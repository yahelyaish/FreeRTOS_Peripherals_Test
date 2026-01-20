#include "packet.h"
#include "stdlib.h"
#include <stddef.h>
#include <arpa/inet.h>
#include "stdio.h"

void build_packet(packet_t *pkt, uint8_t peripheral, uint32_t test_id)
{
    pkt->test_id     = test_id;
    pkt->peripheral  = peripheral;
    pkt->iterations  = 20;

    //pkt->payload_len = (rand() % PAYLOAD_MAX) + 1;
    pkt->payload_len=20;
    /* random payload content */
    for (uint8_t i = 0; i < pkt->payload_len; i++) {
        pkt->payload[i] = (uint8_t)(rand() % 256);
    }
}

ssize_t send_packet_udp(int sock,const struct sockaddr_in *dest_addr,const packet_t *pkt)
{
    size_t header_len = offsetof(packet_t, payload); //calculate the size until payload.
    size_t len  = header_len + pkt->payload_len;

    return sendto(sock,
                  pkt,
                  len  ,
                  0,
                  (const struct sockaddr *)dest_addr,
                  sizeof(*dest_addr));
}



void printPacket(const packet_t *pkt)
{
    printf("Packet:\n");
    printf("  test_id     = %u\n", pkt->test_id);
    printf("  peripheral  = %u\n", pkt->peripheral);
    printf("  iterations  = %u\n", pkt->iterations);
    printf("  payload_len = %u\n", pkt->payload_len);

    printf("  payload     : ");
    for (uint8_t i = 0; i < pkt->payload_len; i++)
        printf("%02X ", pkt->payload[i]);
    printf("\n");
}