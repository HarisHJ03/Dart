#include "pc_task.h"
#include "STM32_TIM_BASE.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

//#include "comm_task.h"
//#include "detect_task.h"
#include "data_packet.h"
#include "pc_rx_data.h"
#include "pc_tx_data.h"
//#include "kalman_filter.h"

//外部调试
//#include "bsp_vofa.h"
//#include "stdio.h"

extern TaskHandle_t pc_rx_Task_Handle;
float nowfps;
UBaseType_t pc_tx_stack_surplus;
UBaseType_t pc_rx_stack_surplus;

float my_time;
void pc_tx_task(void *parm)
{
	//while (1)
	{
		pc_send_data_packet_pack();
		send_packed_fifo_data(&pc_txdata_fifo, UP_REG_ID);
		pc_tx_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
	}
}

TaskHandle_t myxHandle;
TaskStatus_t myxTaskDetails;

//用RTOS的API获取任务的信息
//void vAFunction(void)
//{

//	/* Obtain the handle of a task from its name. */
//	myxHandle = xTaskGetHandle("judge_rx_task");

//	/* Check the handle is not NULL. */
//	configASSERT(myxHandle);

//	/* Use the handle to obtain further information about the task. */
//	vTaskGetInfo(/* The handle of the task being queried. */
//				 myxHandle,
//				 /* The TaskStatus_t structure to complete with information
//				 on xTask. */
//				 &myxTaskDetails,
//				 /* Include the stack high water mark value in the
//				 TaskStatus_t structure. */
//				 pdTRUE,
//				 /* Include the task state in the TaskStatus_t structure. */
//				 eInvalid);
//}

//WorldTime_RxTypedef PC_WorldTime;
//WorldTime_RxTypedef PC_KF_Time;
uint32_t PC_FPS;
uint32_t pc_last_times, pc_times;
#ifdef USE_USB_OTG_FS
extern fifo_s_t pc_rxdata_fifo;
extern uint8_t USB_USART_RX_BUF[64];
extern uint32_t USB_USART_RX_STA;
#endif
void pc_rx_task(void *parm)
{

	//while (1)
	{
#ifdef USE_USB_OTG_FS
		pc_times = HAL_GetTick() - pc_last_times;
		pc_last_times = HAL_GetTick();
//		Get_FPS(&PC_WorldTime, &PC_FPS);//计算帧率
//		nowfps = (float)PC_FPS;
//		PC_KF_Time.WorldTime = xTaskGetTickCount();//记录任务执行的时间
		fifo_s_puts(&pc_rxdata_fifo, USB_USART_RX_BUF, USB_USART_RX_STA);//将数据放入缓存区
		unpack_fifo_data(&pc_unpack_obj, UP_REG_ID);//解包
#else
		
				pc_times = HAL_GetTick() - pc_last_times;
				pc_last_times = HAL_GetTick();
				Get_FPS(&PC_WorldTime, &PC_FPS);
				nowfps = (float)PC_FPS;
				PC_KF_Time.WorldTime = xTaskGetTickCount();
				dma_buffer_to_unpack_buffer(&pc_rx_obj, UART_IDLE_IT);
				unpack_fifo_data(&pc_unpack_obj, UP_REG_ID);
		
#endif
		pc_rx_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
		
	}
}

/*USART1 中断函数*/
void USART1_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (USART_GetFlagStatus(USART1, USART_FLAG_IDLE) != RESET && USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
	{
		USART_ReceiveData(USART1);
		USART_ClearFlag(USART1, USART_FLAG_IDLE); // 清除空闲中断标志位

		err_detector_hook(PC_SYS_OFFLINE);

		if (pc_rx_Task_Handle != NULL) // 避免任务没来得及创建就发送信号量，导致卡在断言机制中
		{
			xTaskNotifyFromISR((TaskHandle_t)pc_rx_Task_Handle,
							   (uint32_t)PC_UART_IDLE_SIGNAL,
							   (eNotifyAction)eSetBits,
							   (BaseType_t *)&xHigherPriorityTaskWoken);

			/*进行上下文切换*/
			if (xHigherPriorityTaskWoken != pdFALSE)
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
	}
}
