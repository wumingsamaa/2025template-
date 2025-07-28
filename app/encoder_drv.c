#include "encoder_drv.h"

/**
 * @brief ��ʼ������������
 */
void Encoder_Driver_Init(Encoder* encoder, TIM_HandleTypeDef *htim, unsigned char reverse)
{
  encoder->htim = htim;			//��ʱ��
  encoder->reverse = reverse;	//����
  
  // ������ʱ���ı�����ģʽ
  HAL_TIM_Encoder_Start(encoder->htim, TIM_CHANNEL_ALL);

  // ���������
  __HAL_TIM_SetCounter(encoder->htim, 0);

  // ��ʼ�����ݽṹ
  encoder->count = 0;
  encoder->total_count = 0;
  encoder->speed_cm_s = 0.0f;
}

/**
 * @brief ���±��������� (Ӧ�����Ե���, ����10msһ��)
 */
void Encoder_Driver_Update(Encoder* encoder)
{
  // 1. ��ȡԭʼ����ֵ
  encoder->count = (int16_t)__HAL_TIM_GetCounter(encoder->htim);
  
  // 2. �������������
  encoder->count = encoder->reverse == 0 ? encoder->count : -encoder->count;

  // 3. ����Ӳ����������Ϊ�¸�������׼��
  __HAL_TIM_SetCounter(encoder->htim, 0);

  // 4. �ۼ�����
  encoder->total_count += encoder->count;

  // 5. �����ٶ� (cm/s)
  // �ٶ� = (����ֵ / PPR) * �ܳ� / ����ʱ��
  encoder->speed_cm_s = (float)encoder->count* WHEEL_CIRCUMFERENCE_CM / ENCODER_PPR  / SAMPLING_TIME_S;
	
  //6������·��
  encoder ->distence_cm += encoder->speed_cm_s * SAMPLING_TIME_S;  //�ҵ���ÿ����ٶȣ����Ի���Ҫ����ʱ�����
}

