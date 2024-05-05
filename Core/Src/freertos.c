/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "uart_receive_dma.h"
#include "usart.h"
#include "rc522.h"
#include "scan_motor.h"
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
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern queue8_t uart_queue;

extern TIM_HandleTypeDef htim1;
#define transmitSignal 0x0001
#define transmitSignal2 0x0002

uint8_t canTxData[8];
uint32_t	TxMailBox;
int motor_status = 0;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId rfidExecuteTaskHandle;
osThreadId tagNumTransmitTHandle;
osThreadId rc522_readHandle;
osThreadId motorTaskHandle;
osThreadId transmitTask2Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartRfidExecuteTask(void const * argument);
void StartTagNumTransmitTask(void const * argument);
void rc522_readTask(void const * argument);
void controlmotor(void const * argument);
void StarttransmitTask2(void const * argument);

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

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of rfidExecuteTask */
  osThreadDef(rfidExecuteTask, StartRfidExecuteTask, osPriorityNormal, 0, 128);
  rfidExecuteTaskHandle = osThreadCreate(osThread(rfidExecuteTask), NULL);

  /* definition and creation of tagNumTransmitT */
  osThreadDef(tagNumTransmitT, StartTagNumTransmitTask, osPriorityNormal, 0, 128);
  tagNumTransmitTHandle = osThreadCreate(osThread(tagNumTransmitT), NULL);

  /* definition and creation of rc522_read */
  osThreadDef(rc522_read, rc522_readTask, osPriorityNormal, 0, 128);
  rc522_readHandle = osThreadCreate(osThread(rc522_read), NULL);

  /* definition and creation of motorTask */
  osThreadDef(motorTask, controlmotor, osPriorityNormal, 0, 128);
  motorTaskHandle = osThreadCreate(osThread(motorTask), NULL);

  /* definition and creation of transmitTask2 */
  osThreadDef(transmitTask2, StarttransmitTask2, osPriorityNormal, 0, 128);
  transmitTask2Handle = osThreadCreate(osThread(transmitTask2), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
	read_rfid_number();
	read_command();
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartRfidExecuteTask */
/**
* @brief Function implementing the rfidExecuteTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRfidExecuteTask */
void StartRfidExecuteTask(void const * argument)
{
  /* USER CODE BEGIN StartRfidExecuteTask */
	  uint8_t read_tag_single_time[] = {0xBB, 0x00, 0x22, 0x00, 0x00, 0x22, 0x7E};
	  vTaskSuspend(rfidExecuteTaskHandle);
	  /* Infinite loop */
	  for(;;)
	  {
		HAL_UART_Transmit(&huart1, read_tag_single_time, sizeof(read_tag_single_time), HAL_MAX_DELAY);
		//vTaskResume(defaultTaskHandle);
	    osDelay(300);
	  }
  /* USER CODE END StartRfidExecuteTask */
}

/* USER CODE BEGIN Header_StartTagNumTransmitTask */
/**
* @brief Function implementing the tagNumTransmitT thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTagNumTransmitTask */
void StartTagNumTransmitTask(void const * argument)
{
  /* USER CODE BEGIN StartTagNumTransmitTask */
  /* Infinite loop */
  osEvent event;
  for(;;)
  {
	event = osSignalWait(transmitSignal, 100);
	if(event.value.signals == transmitSignal){
		transmitData();
		}
  osDelay(1);
  }


  /* USER CODE END StartTagNumTransmitTask */
}

/* USER CODE BEGIN Header_rc522_readTask */
/**
* @brief Function implementing the rc522_read thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_rc522_readTask */
void rc522_readTask(void const * argument)
{
  /* USER CODE BEGIN rc522_readTask */
  uint8_t status,cardstr[16] = {0,};

  /* Infinite loop */
  for(;;)
  {

	if(MFRC522_Request(PICC_REQIDL, cardstr) == MI_OK){
		status = MFRC522_Anticoll(cardstr);
		if(status == MI_OK){
			HAL_UART_Transmit(&huart2, cardstr, 5, 500);
		}
	}
	osDelay(300);
  }
  /* USER CODE END rc522_readTask */
}

/* USER CODE BEGIN Header_controlmotor */
/**
* @brief Function implementing the motorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_controlmotor */
void controlmotor(void const * argument)
{
  /* USER CODE BEGIN controlmotor */
  /* Infinite loop */
	vTaskSuspend(motorTaskHandle);
  /* Infinite loop */
  for(;;)
  {
	  if(motor_status == 0){
		  scan_motor_up();
	  }else if(motor_status == 1){
		  scan_motor_down();
	  }
  }
  /* USER CODE END controlmotor */
}

/* USER CODE BEGIN Header_StarttransmitTask2 */
/**
* @brief Function implementing the transmitTask2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StarttransmitTask2 */
void StarttransmitTask2(void const * argument)
{
  /* USER CODE BEGIN StarttransmitTask2 */
	osEvent event;
  /* Infinite loop */
	  for(;;)
	  {
		  event = osSignalWait(transmitSignal2, 100);
		  if(event.value.signals == transmitSignal2){
			  transmitCommand();
		  }
		  osDelay(1);
	  }
  /* USER CODE END StarttransmitTask2 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
