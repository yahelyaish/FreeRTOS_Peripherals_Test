/*
 * loopbackCRC.c
 *
 *  Created on: Jan 18, 2026
 *      Author: yahel
 */

#include "loopbackCRC.h"
#include "spi.h"
#include "usart.h"
#include "i2c.h"
#include "crc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "string.h"

#define SLAVE_ADDR 0x34<<1

const loopback_table_t loopback = {
    .uart = uart_loopback_crc,
    .spi  = spi_loopback_crc,
    .i2c  = i2c_loopback_crc
};


uint8_t uart_loopback_crc(uint8_t *tx, uint8_t *rx,uint8_t len, uint8_t iters)
{
	uint8_t UARTsuccessCnt=0;
	for(size_t i=0;i<iters;i++)
	{
		HAL_UART_Receive_IT(&huart2, rx, len);
		HAL_UART_Transmit_IT(&huart2, tx, len);
	    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

	    if(crc_check(tx,rx,len)==CRC_OK){
				++UARTsuccessCnt;
	}
}
	return UARTsuccessCnt;
}



uint8_t i2c_loopback_crc(uint8_t *tx, uint8_t *rx,uint8_t len, uint8_t iters)
{
	uint8_t I2CsuccessCnt=0;
	for(size_t i=0;i<iters;i++)
	{
		HAL_I2C_Slave_Receive_IT(&hi2c2, rx, len);
		HAL_I2C_Master_Transmit_IT(&hi2c1, SLAVE_ADDR, tx, len);

	    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

			if(crc_check(tx,rx,len)==CRC_OK){
				++I2CsuccessCnt;
	}
}
	return I2CsuccessCnt;
}




uint8_t spi_loopback_crc(uint8_t *tx, uint8_t *rx,uint8_t len, uint8_t iters)
{

	uint8_t dummy_tx[len],dummy_rx[len];
	memset(dummy_tx, 0xEE, len);
	memset(dummy_rx, 0xFF, len);
	uint8_t SPIsuccessCnt=0;

	for(size_t i=0;i<iters;i++)
	{
		HAL_SPI_TransmitReceive_IT(&hspi4, dummy_tx,rx, len);
		HAL_SPI_TransmitReceive_IT(&hspi1, tx, dummy_rx,len);

	    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

			if(crc_check(tx,rx,len)==CRC_OK){
				++SPIsuccessCnt;
	}
}

	return SPIsuccessCnt;
}


crc_status_t crc_check(uint8_t *tx,uint8_t *rx,uint8_t len){

	uint32_t crc_tx;
	uint32_t crc_rx;

	crc_tx=HAL_CRC_Calculate(&hcrc,(uint32_t*)tx,len);
	crc_rx=HAL_CRC_Calculate(&hcrc,(uint32_t*)rx, len);
	if(crc_tx==crc_rx){
		return CRC_OK;
	}
	return CRC_FAIL;
}

