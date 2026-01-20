#include "packet.h"
#include "stdlib.h"
#include <stddef.h>
#include <arpa/inet.h>
#include "stdio.h"
#include "string.h"



void build_ack(packet_t *ack, const packet_t *cmd)
{
    memset(ack, 0, sizeof(*ack));
    ack->test_id=cmd->test_id;
    ack->peripheral=cmd->peripheral;
}




ssize_t send_packet_udp(int sock,const struct sockaddr_in *dest_addr,const packet_t *pkt)
{
    size_t header_len = offsetof(packet_t, payload); //calculate the size until payload.
    size_t len  = header_len + pkt->payload_len;

    return sendto(sock,
                  pkt,
                  len,
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