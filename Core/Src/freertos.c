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
#include <stdlib.h>
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
#define MAX_PAYLOAD 255

uint8_t ch;
uint8_t loopback_rx_buff[MAX_PAYLOAD];

QueueHandle_t packetQueue;
QueueHandle_t routeMsgQueue;
static QueueHandle_t periphQueueTable[PERIPH_COUNT];
QueueHandle_t resultQueue;


TaskHandle_t uartRxTask_Handle;
TaskHandle_t dispatcherTask_Handle;
TaskHandle_t routerTask_Handle;
TaskHandle_t UARTPeriphCheckTASK_Handle;
TaskHandle_t I2CPeriphCheckTASK_Handle;
TaskHandle_t SPIPeriphCheckTASK_Handle;
TaskHandle_t uartTx_Handle;



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
			packet_t* rxPacket = initPacket(buff, idx);
		    idx = 0;
			if(rxPacket){
			if (xQueueSend(packetQueue, &rxPacket, portMAX_DELAY) != pdTRUE){
				vPortFree(rxPacket->payLoad);
				vPortFree(rxPacket);
				printf("Send msg on QUEUE failed!\r\n");
			}
			}
		}
		HAL_UART_Receive_IT(&huart3,&ch,1);
	}
}

void dispatcherTask(void *arg){
	packet_t *pktCHECK;
	static uint32_t g_testIdCounter;
	for(;;)
	{
		if(xQueueReceive(packetQueue, &pktCHECK, portMAX_DELAY)==pdTRUE)
		{
			fillPacketMeta(pktCHECK,&g_testIdCounter);
			if(xQueueSend(routeMsgQueue,&pktCHECK,portMAX_DELAY)!=pdTRUE)
			{
				vPortFree(pktCHECK->payLoad);
				vPortFree(pktCHECK);
				printf("sending to routeMsgQueue Failed\r\n");
			}
		}
	}
}


void routerTask(void *arg){
	packet_t * routerPKT;
	for(;;){
		if(xQueueReceive(routeMsgQueue, &routerPKT,portMAX_DELAY)==pdTRUE){
			if(xQueueSend(periphQueueTable[routerPKT->PERIPHERAL],&routerPKT,portMAX_DELAY)!=pdTRUE){
				printf("error sending to peripheral\r\n");
			}
		}
	}
}

void UARTPeriphCheckTASK(void *arg){
	packet_t* UART_PKT;
	resProtocol result;
	for(;;){
		if(xQueueReceive(periphQueueTable[UART], &UART_PKT, portMAX_DELAY)==pdTRUE){
			result.peripheral=UART_PKT->PERIPHERAL;
			result.status = loopback.uart(UART_PKT->payLoad,loopback_rx_buff,UART_PKT->payloadSize,UART_PKT->numOfIter);
			if(xQueueSend(resultQueue,&result,portMAX_DELAY)!=pdTRUE){
				printf("sending FAILED\n");
			}
		}
		vPortFree(UART_PKT->payLoad);
		vPortFree(UART_PKT);
	}
}

void I2CPeriphCheckTASK(void *arg){
	packet_t* I2C_PKT;
	resProtocol result;
	for(;;){
		if(xQueueReceive(periphQueueTable[I2C], &I2C_PKT, portMAX_DELAY)!=pdFALSE){
			result.peripheral=I2C_PKT->PERIPHERAL;
			result.status = loopback.i2c(I2C_PKT->payLoad,loopback_rx_buff,I2C_PKT->payloadSize,I2C_PKT->numOfIter);
			if(xQueueSend(resultQueue,&result,portMAX_DELAY)!=pdTRUE){
							printf("sending FAILED\n");
			}
		}
		vPortFree(I2C_PKT->payLoad);
		vPortFree(I2C_PKT);
	}
}
void SPIPeriphCheckTASK(void *arg){
	packet_t* SPI_PKT;
	resProtocol result;
	for(;;){
		if(xQueueReceive(periphQueueTable[SPI], &SPI_PKT, portMAX_DELAY)!=pdFALSE){
			result.peripheral=SPI_PKT->PERIPHERAL;
			result.status = loopback.spi(SPI_PKT->payLoad,loopback_rx_buff,SPI_PKT->payloadSize,SPI_PKT->numOfIter);
			if(xQueueSend(resultQueue,&result,portMAX_DELAY)!=pdTRUE){
							printf("sending FAILED\n");
			}
		}
		vPortFree(SPI_PKT->payLoad);
		vPortFree(SPI_PKT);
	}
}


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
	packetQueue= xQueueCreate(4,sizeof(packet_t*));

	//array of QueueTables
	periphQueueTable[UART] = xQueueCreate(4, sizeof(packet_t*));
	periphQueueTable[I2C]  = xQueueCreate(4, sizeof(packet_t*));
	periphQueueTable[SPI]  = xQueueCreate(4, sizeof(packet_t*));

	routeMsgQueue =xQueueCreate(4,sizeof(packet_t*));

	resultQueue=xQueueCreate(4,sizeof(resProtocol));








  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */

  /* USER CODE BEGIN RTOS_THREADS */
	xTaskCreate(transmitTask,"transmitTask", 256, NULL, 1, &uartTx_Handle);
	xTaskCreate(dispatcherTask, "dispatcherTask", 256, NULL, 1, &dispatcherTask_Handle);

	xTaskCreate(routerTask, "routerTask", 256, NULL, 1, &routerTask_Handle);

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

/*--------------CALLBACKS-----------*/
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

