/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
//#include "crc.h"
#include <semphr.h>

#include "i2c.h"
#include "usart.h"
#include "spi.h"

#include "loopbackCRC.h"
#include "packet.h"


int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart3, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */



/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */



uint8_t ch;
uint8_t loopback_rx_buff[MAX_PAYLOAD];


QueueHandle_t packetQueue;

QueueHandle_t UARTQueue;
QueueHandle_t I2CQueue;
QueueHandle_t SPIQueue;

QueueHandle_t resultQueue;


TaskHandle_t uartRxTask_Handle;
void uartRxTask(void *argument){
    static uint8_t buff[MAX_PAYLOAD];
    static uint8_t idx;

    for(;;){
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		if(ch!='\r'&&ch!='\n'&&idx<MAX_PAYLOAD-1)
		{
			HAL_UART_Transmit_IT(&huart3, &ch, 1);
			buff[idx++]=ch;
		}else
		{
		    packet_t rxPacket;
		    rxPacket.payloadSize=idx;
			strcpy((char*)rxPacket.payLoad,(char*)buff);
			rxPacket.payLoad[rxPacket.payloadSize]='\0';

			if (xQueueSend(packetQueue, &rxPacket, portMAX_DELAY) != pdTRUE){
			   printf("Send msg on QUEUE failed!\r\n");
			}
		}
		HAL_UART_Receive_IT(&huart3,&ch,1);
	}
}


TaskHandle_t dispatcherTask_Handle;
void dispatcherTask(void *arg){
	packet_t pktCHECK;
	static uint32_t g_testIdCounter;
	for(;;)
	{
		if(xQueueReceive(packetQueue, &pktCHECK, portMAX_DELAY)==pdTRUE){
			pktCHECK.TEST_ID=(++g_testIdCounter);
			pktCHECK.numOfIter=NUM_OF_ITER;
			pktCHECK.PERIPHERAL=SPI;
			switch (pktCHECK.PERIPHERAL)
			{
			case UART:
			    if (xQueueSend(UARTQueue, &pktCHECK, portMAX_DELAY) != pdTRUE)
			    {
			       printf("Send msg on UARTQUEUE failed!\r\n");
			    }
			    break;

			case I2C:
			    if (xQueueSend(I2CQueue, &pktCHECK, portMAX_DELAY) != pdTRUE)
			    {
			        printf("Send msg on I2CQUEUE failed!\r\n");
			    }
			    break;

			 case SPI:
			     if (xQueueSend(SPIQueue, &pktCHECK, portMAX_DELAY) != pdTRUE)
			     {
			         printf("Send msg on SPIQUEUE failed!\r\n");
			     }
			     break;

			default:
			    printf("Unknown PERIPHERAL = 0x%02X\r\n", pktCHECK.PERIPHERAL);
			    break;
			}
			}
		}
	}


TaskHandle_t UARTPeriphCheckTASK_Handle;
void UARTPeriphCheckTASK(void *arg){
	packet_t UART_PKT;
	resProtocol result;
	for(;;){
		if(xQueueReceive(UARTQueue, &UART_PKT, portMAX_DELAY)==pdTRUE){
			result.peripheral=UART_PKT.PERIPHERAL;
			result.status = loopback.uart(UART_PKT.payLoad,loopback_rx_buff,UART_PKT.payloadSize,UART_PKT.numOfIter);
			if(xQueueSend(resultQueue,&result,portMAX_DELAY)!=pdTRUE){
				printf("sending FAILED\n");
			}

		}
	}
}

TaskHandle_t I2CPeriphCheckTASK_Handle;
void I2CPeriphCheckTASK(void *arg){
	packet_t I2C_PKT;
	resProtocol result;
	for(;;){
		if(xQueueReceive(I2CQueue, &I2C_PKT, portMAX_DELAY)!=pdFALSE){
			result.peripheral=I2C_PKT.PERIPHERAL;
			result.status = loopback.i2c(I2C_PKT.payLoad,loopback_rx_buff,I2C_PKT.payloadSize,I2C_PKT.numOfIter);
			if(xQueueSend(resultQueue,&result,portMAX_DELAY)!=pdTRUE){
							printf("sending FAILED\n");
			}
		}
	}
}

TaskHandle_t SPIPeriphCheckTASK_Handle;
void SPIPeriphCheckTASK(void *arg){
	packet_t SPI_PKT;
	resProtocol result;
	for(;;){
		if(xQueueReceive(SPIQueue, &SPI_PKT, portMAX_DELAY)!=pdFALSE){
			result.peripheral=SPI_PKT.PERIPHERAL;
			result.status = loopback.spi(SPI_PKT.payLoad,loopback_rx_buff,SPI_PKT.payloadSize,SPI_PKT.numOfIter);
			if(xQueueSend(resultQueue,&result,portMAX_DELAY)!=pdTRUE){
							printf("sending FAILED\n");
			}
		}
	}
}


TaskHandle_t uartTx_Handle;
void transmitTask(void *arg){
    resProtocol resTX;
	for(;;){
		if(xQueueReceive(resultQueue, &resTX, portMAX_DELAY)==pdTRUE){
			printf("\r\n%s PASSED THE TEST %d TIMES OUT OF %d\r\n",periphStr[resTX.peripheral],resTX.status,NUM_OF_ITER);
		}
	}
}

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
HAL_UART_Receive_IT(&huart3, &ch, 1);
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	//doneRxSem = xSemaphoreCreateCounting(10, 0);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	packetQueue= xQueueCreate(4,sizeof(packet_t));

	UARTQueue= xQueueCreate(4,sizeof(packet_t));
	I2CQueue= xQueueCreate(4,sizeof(packet_t));
	SPIQueue= xQueueCreate(4,sizeof(packet_t));


	resultQueue=xQueueCreate(4,sizeof(resProtocol));


  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */

  /* USER CODE BEGIN RTOS_THREADS */
	xTaskCreate(transmitTask,"transmitTask", 256, NULL, 1, &uartTx_Handle);
	xTaskCreate(dispatcherTask, "dispatcherTask", 256, NULL, 1, &dispatcherTask_Handle);

	xTaskCreate(UARTPeriphCheckTASK, "UARTPeriphCheckTASK", 256, NULL, 1, &UARTPeriphCheckTASK_Handle);
	xTaskCreate(I2CPeriphCheckTASK, "I2CPeriphCheckTASK", 256, NULL, 1, &I2CPeriphCheckTASK_Handle);
	xTaskCreate(SPIPeriphCheckTASK, "SPIPeriphCheckTASK", 256, NULL, 1, &SPIPeriphCheckTASK_Handle);

	xTaskCreate(uartRxTask, "uartRxTask", 256, NULL, 1, &uartRxTask_Handle);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */



/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (huart == &huart3)
    {
    	 xTaskNotifyFromISR(
    			 uartRxTask_Handle,
    	            0,
    	            eNoAction,
    	            &xHigherPriorityTaskWoken
    	        );
    }

    if(huart==&huart2){

    	 xTaskNotifyFromISR(
    			 	 	 	UARTPeriphCheckTASK_Handle,
    	    	            0,
    	    	            eNoAction,
    	    	            &xHigherPriorityTaskWoken
    	    	        );
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

//uint8_t uart_loopback_crc(uint8_t *tx, uint8_t *rx,
//                             uint16_t len, uint8_t iters)
//{
//	uint8_t UARTsuccessCnt=0;
//	for(size_t i=0;i<iters;i++)
//	{
//		HAL_UART_Receive_IT(&huart2, rx, len);
//		HAL_UART_Transmit_IT(&huart2, tx, len);
//
//	    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
//
//	    if(crc_check(tx,rx,len)==CRC_OK){
//				++UARTsuccessCnt;
//	}
//}
//	return UARTsuccessCnt;
//}


void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c){
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if(hi2c==&hi2c2)
	{
		 xTaskNotifyFromISR(
				 	 	 	 I2CPeriphCheckTASK_Handle,
		    	    	            0,
		    	    	            eNoAction,
		    	    	            &xHigherPriorityTaskWoken
		    	    	        );
	}
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

}

//uint8_t i2c_loopback_crc(uint8_t *tx, uint8_t *rx,
//                             uint16_t len, uint8_t iters)
//{
//	uint8_t I2CsuccessCnt=0;
//	for(size_t i=0;i<iters;i++)
//	{
//		HAL_I2C_Slave_Receive_IT(&hi2c2, rx, len);
//		HAL_I2C_Master_Transmit_IT(&hi2c1, SLAVE_ADDR, tx, len);
//
//	    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
//
//			if(crc_check(tx,rx,len)==CRC_OK){
//				++I2CsuccessCnt;
//	}
//}
//	return I2CsuccessCnt;
//}
//



//
//void print_hex(const char *label, uint8_t *buf, uint16_t len)
//{
//    printf("%s: ", label);
//    for (uint16_t i = 0; i < len; i++) {
//        printf("%02X ", buf[i]);
//    }
//    printf("\r\n");
//}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
	BaseType_t xHigherPriorityTaskWoken;
	if(hspi==&hspi1){
		 xTaskNotifyFromISR(
		    			 	 	 	SPIPeriphCheckTASK_Handle,
		    	    	            0,
		    	    	            eNoAction,
		    	    	            &xHigherPriorityTaskWoken
		    	    	        );
		    }

		    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


//
//uint8_t spi_loopback_crc(uint8_t *tx, uint8_t *rx,
//                             uint16_t len, uint8_t iters)
//{
//
//	uint8_t dummy_tx[len],dummy_rx[len];
//	memset(dummy_tx, 0xEE, len);
//	memset(dummy_rx, 0xFF, len);
//	uint8_t SPIsuccessCnt=0;
//
////	print_hex("BEFORE rx", rx, len);
////			print_hex("BEFORE tx", tx, len);
////			print_hex("BEFORE dummy_rx", dummy_rx, len);
////			print_hex("BEFORE dummy_tx", dummy_tx, len);
//	for(size_t i=0;i<iters;i++)
//	{
//		HAL_SPI_TransmitReceive_IT(&hspi4, dummy_tx,rx, len);
//		HAL_SPI_TransmitReceive_IT(&hspi1, tx, dummy_rx,len);
//
//	    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
//
//			if(crc_check(tx,rx,len)==CRC_OK){
//				++SPIsuccessCnt;
//	}
//}
////	print_hex("AFTER rx", rx, len);
////	print_hex("AFTER tx", tx, len);
////	print_hex("AFTER dummy_rx", dummy_rx, len);
////	print_hex("AFTER dummy_tx", dummy_tx, len);
//	return SPIsuccessCnt;
//}


//
//crc_status_t crc_check(uint8_t *tx,uint8_t *rx,uint8_t len){
//
//	uint32_t crc_tx;
//	uint32_t crc_rx;
//
//	crc_tx=HAL_CRC_Calculate(&hcrc, tx,len);
//	crc_rx=HAL_CRC_Calculate(&hcrc, rx, len);
//	if(crc_tx==crc_rx){
//		return CRC_OK;
//	}
//	return CRC_FAIL;
//}
//


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* USER CODE END Variables */


/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */






/* USER CODE END Application */

