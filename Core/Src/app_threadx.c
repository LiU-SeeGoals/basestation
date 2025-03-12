/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_threadx.c
  * @author  MCD Application Team
  * @brief   ThreadX applicative file
  ******************************************************************************
    * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "app_threadx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "com.h"
#include <stdio.h>
#include "log.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TX_APP_STACK_SIZE 1024
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
TX_THREAD app_thread;
TX_THREAD dummy_data_thread;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
static VOID tx_app_thread_entry (ULONG thread_input);
static VOID dummy_data_thread_entry(ULONG thread_input);
/* USER CODE END PFP */

/**
  * @brief  Application ThreadX Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT App_ThreadX_Init(VOID *memory_ptr)
{
  UINT ret = TX_SUCCESS;

  /* USER CODE BEGIN App_ThreadX_MEM_POOL */

  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;
  CHAR *pointer, *pointer2;
  if (tx_byte_allocate(byte_pool, (VOID **) &pointer, TX_APP_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }

#if SEND_DUMMY_DATA==1
  if (tx_byte_allocate(byte_pool, (VOID **) &pointer2, TX_APP_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }
#endif
  /* USER CODE END App_ThreadX_MEM_POOL */

  /* USER CODE BEGIN App_ThreadX_Init */
  ret = tx_thread_create(&app_thread, "Tx App thread", tx_app_thread_entry, 0, pointer, TX_APP_STACK_SIZE, 11, 11, TX_NO_TIME_SLICE, TX_AUTO_START);
#if SEND_DUMMY_DATA==1
  ret |= tx_thread_create(&dummy_data_thread, "Dummy Data Thread", dummy_data_thread_entry, 0, pointer2, TX_APP_STACK_SIZE, 11, 11, TX_NO_TIME_SLICE, TX_AUTO_START);
#endif
  /* USER CODE END App_ThreadX_Init */

  return ret;
}

  /**
  * @brief  MX_ThreadX_Init
  * @param  None
  * @retval None
  */
void MX_ThreadX_Init(void)
{
  /* USER CODE BEGIN  Before_Kernel_Start */

  /* USER CODE END  Before_Kernel_Start */

  tx_kernel_enter();

  /* USER CODE BEGIN  Kernel_Start_Error */

  /* USER CODE END  Kernel_Start_Error */
}

/* USER CODE BEGIN 1 */
static VOID tx_app_thread_entry(ULONG thread_input)
{
  COM_RF_Init(&hspi1);
  //COM_RF_PingRobots(true);
}

static VOID dummy_data_thread_entry(ULONG thread_input)
{
  for (;;) {
    for (uint8_t i = 0; i < MAX_ROBOT_COUNT; ++i) {
      uint8_t robot_id = i;
      uint8_t len;
      uint8_t* buffer = COM_CreateDummyPacket(robot_id, &len);
      COM_RF_Transmit(robot_id, buffer, len);
      free(buffer);
    }
    tx_thread_sleep(0);
  }
}
/* USER CODE END 1 */
