/*
 * packet.h
 *
 *  Created on: Jan 18, 2026
 *      Author: yahel
 */

#ifndef INC_PACKET_H_
#define INC_PACKET_H_

#include <stdint.h>

#define NUM_OF_ITER 100
#define MAX_PAYLOAD 255


#define UART 0x0
#define I2C 0x1
#define SPI 0x2


extern const char* periphStr[];


typedef struct packet_t{
	uint32_t TEST_ID;
	uint8_t PERIPHERAL;
	uint8_t numOfIter;
	uint8_t payloadSize;
	uint8_t payLoad[MAX_PAYLOAD];
}packet_t;

typedef struct resProtocol{
    uint8_t peripheral;
	uint8_t status;
}resProtocol;




#endif /* INC_PACKET_H_ */
