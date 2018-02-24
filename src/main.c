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
			
void task1(void)
{

}

int main(void)
{
	xTaskCreate( task1,				/* The function that implements the task. */
							"task1", 								/* The text name assigned to the task - for debug only as it is not used by the kernel. */
							2048, 			/* The size of the stack to allocate to the task. */
							NULL, 								/* The parameter passed to the task - not used in this case. */
							10, 	/* The priority assigned to the task. */
							NULL );
	vTaskStartScheduler();

	for(;;);
}
