#ifndef __ENCODER_DRV_H
#define __ENCODER_DRV_H

#include "bsp_system.h"

// ������ÿתһȦ�������� (PPR)
#define ENCODER_PPR (13 * 30 * 4) // 13��/��, 20�����ٱ�, 4��Ƶ
// ����ֱ�� (��λ: ����)
#define WHEEL_DIAMETER_CM 6.5f

// �Զ������ܳ��Ͳ���ʱ��
#define PI 3.14159265f
#define WHEEL_CIRCUMFERENCE_CM (WHEEL_DIAMETER_CM * PI)
#define SAMPLING_TIME_S 0.02f // ����ʱ��, �� Scheduler �е���������һ�� (10ms)

/**
 * @brief ���������ݽṹ��
 */
typedef struct
{
  TIM_HandleTypeDef *htim; // ��ʱ��
  unsigned char reverse; // �������ķ����Ƿ�ת��0-������1-��ת
  int16_t count;          // ��ǰ���������ڵ�ԭʼ����ֵ
  int32_t total_count;    // �ۼ��ܼ���ֵ
  float speed_cm_s;     // ��������ٶ� (cm/s)
  float distence_cm;    //����·��
} Encoder;

void Encoder_Driver_Init(Encoder* encoder, TIM_HandleTypeDef *htim, unsigned char reverse);
void Encoder_Driver_Update(Encoder* encoder);

extern Encoder left_encoder;
extern Encoder right_encoder;

#endif
