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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "chispa.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

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
/* ---- Log por UART: cualquier tarea escribe en la cola, solo tLog usa la UART ---- */
typedef struct { char txt[96]; } logmsg_t;
osMessageQueueId_t qLogHandle;      /* mensajes de texto -> tarea Log          */
osMessageQueueId_t qPantallaHandle; /* vista_t (longitud 1) Logica -> Pantalla  */

void log_printf(const char *fmt, ...)
{
  logmsg_t m;
  int n = snprintf(m.txt, sizeof(m.txt), "[%lu] ", (unsigned long)osKernelGetTickCount());
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(m.txt + n, sizeof(m.txt) - n, fmt, ap);
  va_end(ap);
  osMessageQueuePut(qLogHandle, &m, 0, 0);   /* no bloquea; si la cola esta llena se descarta */
}

/* USER CODE END Variables */
/* Definitions for tSensores */
osThreadId_t tSensoresHandle;
const osThreadAttr_t tSensores_attributes = {
  .name = "tSensores",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for tSonido */
osThreadId_t tSonidoHandle;
const osThreadAttr_t tSonido_attributes = {
  .name = "tSonido",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for tLogica */
osThreadId_t tLogicaHandle;
const osThreadAttr_t tLogica_attributes = {
  .name = "tLogica",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for tPantalla */
osThreadId_t tPantallaHandle;
const osThreadAttr_t tPantalla_attributes = {
  .name = "tPantalla",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for tLog */
osThreadId_t tLogHandle;
const osThreadAttr_t tLog_attributes = {
  .name = "tLog",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for qEventos */
osMessageQueueId_t qEventosHandle;
const osMessageQueueAttr_t qEventos_attributes = {
  .name = "qEventos"
};
/* Definitions for mutexI2C */
osMutexId_t mutexI2CHandle;
const osMutexAttr_t mutexI2C_attributes = {
  .name = "mutexI2C"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartSensores(void *argument);
void StartSonido(void *argument);
void StartLogica(void *argument);
void StartPantalla(void *argument);
void StartLog(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
  /* Error grave: se reporta directo por la UART (sin cola) y se detiene el sistema */
  char msg[48];
  int n = snprintf(msg, sizeof(msg), "\r\n!! STACK OVERFLOW en %s\r\n", (char *)pcTaskName);
  HAL_UART_Transmit(&huart1, (uint8_t *)msg, n, 100);
  __disable_irq();
  while (1) { }
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
  const char msg[] = "\r\n!! MALLOC FAILED: falta heap de FreeRTOS\r\n";
  HAL_UART_Transmit(&huart1, (uint8_t *)msg, sizeof(msg) - 1, 100);
  __disable_irq();
  while (1) { }
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
}
/* USER CODE END 5 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of mutexI2C */
  mutexI2CHandle = osMutexNew(&mutexI2C_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of qEventos */
  qEventosHandle = osMessageQueueNew (10, sizeof(uint8_t), &qEventos_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  qLogHandle = osMessageQueueNew(16, sizeof(logmsg_t), NULL);
  qPantallaHandle = osMessageQueueNew(1, sizeof(vista_t), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of tSensores */
  tSensoresHandle = osThreadNew(StartSensores, NULL, &tSensores_attributes);

  /* creation of tSonido */
  tSonidoHandle = osThreadNew(StartSonido, NULL, &tSonido_attributes);

  /* creation of tLogica */
  tLogicaHandle = osThreadNew(StartLogica, NULL, &tLogica_attributes);

  /* creation of tPantalla */
  tPantallaHandle = osThreadNew(StartPantalla, NULL, &tPantalla_attributes);

  /* creation of tLog */
  tLogHandle = osThreadNew(StartLog, NULL, &tLog_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartSensores */
/**
  * @brief  Function implementing the tSensores thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartSensores */
void StartSensores(void *argument)
{
  /* USER CODE BEGIN StartSensores */
  Sensores_Task();   /* implementada en chispa_sensores.c */
  /* USER CODE END StartSensores */
}

/* USER CODE BEGIN Header_StartSonido */
/**
* @brief Function implementing the tSonido thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSonido */
void StartSonido(void *argument)
{
  /* USER CODE BEGIN StartSonido */
  Sonido_Task();   /* implementada en chispa_sonido.c */
  /* USER CODE END StartSonido */
}

/* USER CODE BEGIN Header_StartLogica */
/**
* @brief Function implementing the tLogica thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLogica */
void StartLogica(void *argument)
{
  /* USER CODE BEGIN StartLogica */
  Logica_Task();   /* implementada en chispa_logica.c */
  /* USER CODE END StartLogica */
}

/* USER CODE BEGIN Header_StartPantalla */
/**
* @brief Function implementing the tPantalla thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPantalla */
void StartPantalla(void *argument)
{
  /* USER CODE BEGIN StartPantalla */
  Pantalla_Task();   /* implementada en chispa_pantalla.c */
  /* USER CODE END StartPantalla */
}

/* USER CODE BEGIN Header_StartLog */
/**
* @brief Function implementing the tLog thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLog */
void StartLog(void *argument)
{
  /* USER CODE BEGIN StartLog */
  logmsg_t m;
  log_printf("=== Chispa arrancando ===\r\n");
  for(;;)
  {
    /* Bloqueada (sin consumir CPU) hasta que llegue un mensaje */
    if (osMessageQueueGet(qLogHandle, &m, NULL, osWaitForever) == osOK)
    {
      HAL_UART_Transmit(&huart1, (uint8_t *)m.txt, strlen(m.txt), 100);
    }
  }
  /* USER CODE END StartLog */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

