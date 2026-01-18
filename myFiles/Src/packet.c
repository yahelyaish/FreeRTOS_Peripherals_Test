/*
 * packet.c
 *
 *  Created on: Jan 18, 2026
 *      Author: yahel
 */
#include "packet.h"
#include "stdlib.h"
const char* periphStr[]={"UART","I2C","SPI"};

packet_t* initPacket(const uint8_t *buff, uint8_t len)
{
    packet_t *pkt = pvPortMalloc(sizeof(packet_t));
    if (pkt == NULL)
    {
        return NULL;
    }

    pkt->payLoad = pvPortMalloc(len + 1);
    if (pkt->payLoad == NULL)
    {
        vPortFree(pkt);
        return NULL;
    }

    memcpy(pkt->payLoad, buff, len);
    pkt->payLoad[len] = '\0';
    pkt->payloadSize = len;

    return pkt;
}


void fillPacketMeta(packet_t* pkt,uint32_t* testID){
	pkt->TEST_ID=++(*testID);
	pkt->numOfIter=NUM_OF_ITER;
	pkt->PERIPHERAL=UART;
}

void printPacket(packet_t* pkt){
	if(pkt){
	printf("\r\ntestid = %d\r\nperipheral = %s\r\nnum of iter = %d\r\npayload size = %d\r\npayload[] = %s\r\n",
	pkt->TEST_ID,
	periphStr[pkt->PERIPHERAL],
	pkt->numOfIter,
	pkt->payloadSize,
	pkt->payLoad);
}else{
	printf("pkt is NULL\r\n");
}
}
