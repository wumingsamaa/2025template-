#include "encoder_bsp.h"

// ���ұ��������
Encoder left_encoder;
Encoder right_encoder;

/**
 * @brief ��ʼ��������Ӧ��
 */
void encoder_init(void)
{
  Encoder_Driver_Init(&left_encoder, &htim3, 0);
  Encoder_Driver_Init(&right_encoder, &htim4, 1);
}

/**
 * @brief ������Ӧ���������� (Ӧ�ɵ����������Ե���)
 */
void encoder_proc(void)
{
  Encoder_Driver_Update(&left_encoder);
  Encoder_Driver_Update(&right_encoder);
	
}
