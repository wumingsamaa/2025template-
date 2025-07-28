#include "hwt101_bsp.h"

HWT101_t hwt101;  // HWT101������ʵ��

void hwt101_init(void)
{
	// ����HWT101ʵ��
	HWT101_Create(&hwt101, &huart3, 1000);  // ʹ��USART2����ʱ1000ms
	// ��ʼ����������DMA����
	HAL_UART_Receive_DMA(&huart3, uart3_rx_buffer, sizeof(uart3_rx_buffer));
}

void hwt101_proc(void)
{
    // ��ȡ����������
    float yaw = HWT101_GetYaw(&hwt101);           // ��ȡƫ����
    float gyro_z = HWT101_GetGyroZ(&hwt101);      // ��ȡ���ٶ�Z
    
    // ��ȡ��������
    HWT101_Data_t* data = HWT101_GetData(&hwt101);
    if (data != NULL)
    {
        my_printf(&huart1,"Yaw: %.2f��, GyroZ: %.2f��/s\r\n", data->yaw, data->gyro_z);
    }
}
