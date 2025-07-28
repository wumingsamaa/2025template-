#ifndef __STEP_MOTOR_BSP_H__
#define __STEP_MOTOR_BSP_H__

#include "bsp_system.h"

/* ������ƺ궨�� */
#define MOTOR_X_ADDR        0x01          // X������ַ
#define MOTOR_Y_ADDR        0x01          // Y������ַ
#define MOTOR_X_UART        huart2        // X�������� xia
#define MOTOR_Y_UART        huart4        // Y�������� shang
#define MOTOR_MAX_SPEED     3            // ������ת��(RPM)
#define MOTOR_ACCEL         0             // ������ٶ�(0��ʾֱ������)
#define MOTOR_SYNC_FLAG     false         // ���ͬ����־
#define MOTOR_MAX_ANGLE     50            // ������Ƕ�����(��50��)

/* �������� */
void Step_Motor_Init(void);                    // �����ʼ��
void Step_Motor_Set_Speed(int8_t x_percent, int8_t y_percent);  // ����XY����ٶ�(�ٷֱ�)
void Step_Motor_Set_Speed_my(float x_rpm, float y_rpm);
void Step_Motor_Stop(void);                    // ֹͣ���е��
void step_motor_proc(void);
void Step_Motor_Set_Pwm(int32_t x_distance, int32_t y_distance);

#endif
