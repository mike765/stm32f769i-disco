/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f7xx.h"
#include "stm32f769i_discovery.h"
#include "FreeRTOS.h"
#include "task.h"
#include "gpio.h"

void task1(void)
{
	logger_send("Task1 is started!");

	while (1)
	{
		HAL_GPIO_WritePin(LD_USER2_GPIO_Port, LD_USER2_Pin, GPIO_PIN_SET);
		vTaskDelay(1000);
		HAL_GPIO_WritePin(LD_USER2_GPIO_Port, LD_USER2_Pin, GPIO_PIN_RESET);
		vTaskDelay(1000);
	}
}

void task2(void)
{
	while (1)
	{
		vTaskDelay(500);
	}
}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  if(uartHandle->Instance==USART1)
  {
    /* Peripheral clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    /**USART1 GPIO Configuration
    PA10     ------> USART1_RX
    PA9     ------> USART1_TX
    */
    GPIO_InitStruct.Pin = VCP_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(VCP_RX_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = VCP_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(VCP_TX_GPIO_Port, &GPIO_InitStruct);
  }

  if(uartHandle->Instance==USART6)
    {
      /* Peripheral clock enable */
      __HAL_RCC_USART6_CLK_ENABLE();

      /* USART6 GPIO Configuration */
	 GPIO_InitStruct.Pin = ARD_D0_RX_Pin;
	 GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	 GPIO_InitStruct.Pull = GPIO_PULLUP;
	 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	 GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
	 HAL_GPIO_Init(ARD_D0_RX_GPIO_Port, &GPIO_InitStruct);

	 GPIO_InitStruct.Pin = ARDUINO_TX_D1_Pin;
	 GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	 GPIO_InitStruct.Pull = GPIO_NOPULL;
	 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	 GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
	 HAL_GPIO_Init(ARDUINO_TX_D1_GPIO_Port, &GPIO_InitStruct);
    }
}

static void SystemClock_Config(void);

int main(void)
{
	HAL_Init();

	/* Configure the System clock to 216 MHz */
	SystemClock_Config();

	MX_GPIO_Init();

	/* The logger use VCP UART */
	logger_init();

	xTaskCreate( task1,				/* The function that implements the task. */
							"task1", 		/* The text name assigned to the task - for debug only as it is not used by the kernel. */
							2048, 			/* The size of the stack to allocate to the task. */
							NULL, 			/* The parameter passed to the task - not used in this case. */
							5, 	/* The priority assigned to the task. */
							NULL );
	xTaskCreate( task2,				/* The function that implements the task. */
							"task2", 		/* The text name assigned to the task - for debug only as it is not used by the kernel. */
							2048, 			/* The size of the stack to allocate to the task. */
							NULL, 	/* The parameter passed to the task - not used in this case. */
							5, 	/* The priority assigned to the task. */
							NULL );
	vTaskStartScheduler();

	for(;;);
}

void vApplicationIdleHook(void)
{
	__WFI;
}

void vApplicationMallocFailedHook(void)
{
	while(1);
}

void vApplicationStackOverflowHook(void)
{
	while(1);
}

void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
  while(1);
}

void vAssertCalled( unsigned long ulLine, const char * const pcFileName )
{
static portBASE_TYPE xPrinted = pdFALSE;
volatile uint32_t ulSetToNonZeroInDebuggerToContinue = 0;

    /* Parameters are not used. */
    ( void ) ulLine;
    ( void ) pcFileName;

    taskENTER_CRITICAL();
    {
        /* You can step out of this function to debug the assertion by using
        the debugger to set ulSetToNonZeroInDebuggerToContinue to a non-zero
        value. */
        while( ulSetToNonZeroInDebuggerToContinue == 0 )
        {
        }
    }
    taskEXIT_CRITICAL();
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 216000000
  *            HCLK(Hz)                       = 216000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 25
  *            PLL_N                          = 432
  *            PLL_P                          = 2
  *            PLL_Q                          = 9
  *            PLL_R                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 7
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef  ret = HAL_OK;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  RCC_OscInitStruct.PLL.PLLR = 7;

  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }

  /* Activate the OverDrive to reach the 216 MHz Frequency */
  ret = HAL_PWREx_EnableOverDrive();
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
}
