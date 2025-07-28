

/**
 ******************************************************************************
 * @file    motor_driver.h
 * @brief   TB6612 ���������ͷ�ļ� (ͨ��ӳ���)
 * @author  (�������)
 * @date    2025��07��08��
 ******************************************************************************
 * @attention
 *
 * 1. ����רΪTB6612FNG�������оƬ��ơ�
 * 2. ���á��ٶ�ӳ�䡱˼�룬����ӿ�ʹ�ðٷֱ��ٶ�
 * �����Զ�ӳ��ΪӲ��PWMֵ�����и�ͨ���ԡ�
 * 3. ֧��˫����������ơ�
 *
 ******************************************************************************
 */

#ifndef __MOTOR_DRIVER_TB6612_H__
#define __MOTOR_DRIVER_TB6612_H__

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
// ������Ĺ��̰�����Ҫ��HAL��ͷ�ļ�
#include "bsp_system.h"

/* Exported constants --------------------------------------------------------*/

//=============================================================================
//=====                  �û������������� (User Core Configuration)         =====
//=============================================================================

/**
 * @brief ����PWM������ֵ (��Ӳ����ʱ������ǿ���)
 * @note  ��ֵӦΪ��Ķ�ʱ��ARR(Auto-Reload Register)ֵ + 1��
 * ���磬������ `htim.Init.Period` ����Ϊ 999, �˴�Ӧ����Ϊ 1000��
 */
#define MOTOR_PWM_MAX_PULSE 100

/**
 * @brief ��С����PWM������ֵ (��λ��PWM����ֵ)
 * @note  ���ڽ�������PWMռ�ձȹ���ʱ�޷����������⡣
 * ����������PWMֵ����0��С�ڴ���ֵ����ᱻǿ������������ֵ��
 * �������Ҫ�˹��ܣ����Խ�������Ϊ 0��
 */
#define MOTOR_MIN_PHYSICAL_PWM 0

/**
 * @brief TB6612 ����(Standby)���Ŷ���
 */
#define TB6612_STBY_PORT GPIOB
#define TB6612_STBY_PIN GPIO_PIN_10

//=============================================================================
//=====                  ���ڲ��궨�� (Library Internal Macros)             =====
//=============================================================================

/**
 * @brief �߼��ٶȷ�Χ (�ṩ���û��Ľӿ�)���̶�Ϊ�ٷֱ�
 * @note
 */
#define MOTOR_LOGIC_SPEED_MAX 100  // �߼������ת�ٶ�
#define MOTOR_LOGIC_SPEED_MIN -100 // �߼����ת�ٶ�

    /* Exported types ------------------------------------------------------------*/

    /**
     * @brief ���״̬ö��
     */
    typedef enum
    {
        MOTOR_STATE_STOP = 0, // ֹͣ/ɲ��
        MOTOR_STATE_FORWARD,  // ��ת
        MOTOR_STATE_BACKWARD, // ��ת
        MOTOR_STATE_ERROR     // ����
    } MotorState_t;

    /**
     * @brief ���Ӳ�����ýṹ��
     */
    typedef struct
    {
        TIM_HandleTypeDef *htim; // PWM��ʱ�����
        uint32_t channel;        // PWMͨ�� (TIM_CHANNEL_x)

        GPIO_TypeDef *in1_port; // �����������1 �˿�
        uint16_t in1_pin;       // �����������1 �ܽ�

        GPIO_TypeDef *in2_port; // �����������2 �˿�
        uint16_t in2_pin;       // �����������2 �ܽ�
    } MotorHW_t;

    /**
     * @brief �������ʵ��ṹ��
     */
    typedef struct
    {
        MotorHW_t hw;       // Ӳ������
        int16_t speed;      // ��ǰ�ٶ� (-100% ~ +100%)
        MotorState_t state; // ��ǰ״̬
        uint8_t enable;     // ���ʹ�ܱ�־ (1:ʹ��, 0:ʧ��)
        uint8_t reverse;    // �����װ���� (0:��װ, 1:��װ)
    } Motor_t;

    /* Exported functions prototypes ---------------------------------------------*/

    /**
     * @brief ��ʼ��TB6612оƬ��������STBY���š�
     * @param enable: 1-ʹ��оƬ(�˳�����), 0-��оƬ�������ģʽ��
     */
    void TB6612_Init(uint8_t enable);

    /**
     * @brief ��������ʼ��һ�����ʵ����
     * @param motor:    ָ��Ҫ��ʼ���ĵ��ʵ��ṹ���ָ�롣
     * @param htim:     ���Ƹõ����PWM��ʱ�������
     * @param channel:  PWMͨ�� (���� TIM_CHANNEL_1)��
     * @param in1_port: ��������1���ڵ�GPIO�˿� (���� GPIOE)��
     * @param in1_pin:  ��������1�Ĺܽź� (���� GPIO_PIN_9)��
     * @param in2_port: ��������2���ڵ�GPIO�˿ڡ�
     * @param in2_pin:  ��������2�Ĺܽźš�
     * @param reverse:  �����װ���� (0 ��ʾ��װ, 1 ��ʾ��װ����װʱ�ٶȻ��Զ�ȡ��)��
     * @retval 0: �ɹ�, -1: �����������
     */
    int8_t Motor_Create(Motor_t *motor,
                        TIM_HandleTypeDef *htim,
                        uint32_t channel,
                        GPIO_TypeDef *in1_port, uint16_t in1_pin,
                        GPIO_TypeDef *in2_port, uint16_t in2_pin,
                        uint8_t reverse);

    /**
     * @brief ���õ���ٶȡ�
     * @param motor: ָ����ʵ���ָ�롣
     * @param speed: �ٶ�ֵ
     * - ����������ת��
     * - ��������ת��
     * - 0 ����ɲ����
     * @retval 0: �ɹ�, -1: �����������
     */
    int8_t Motor_SetSpeed(Motor_t *motor, int16_t speed);

    /**
     * @brief ����ֹͣ��� (ɲ��ģʽ)��
     * @param motor: ָ����ʵ���ָ�롣
     * @retval 0: �ɹ�, -1: �����������
     */
    int8_t Motor_Stop(Motor_t *motor);

    /**
     * @brief ��ȡ�����ǰ״̬��
     * @param motor: ָ����ʵ���ָ�롣
     * @retval ���״̬ (MOTOR_STATE_FORWARD, MOTOR_STATE_BACKWARD, MOTOR_STATE_STOP)��
     */
    MotorState_t Motor_GetState(Motor_t *motor);

    /**
     * @brief (�������)ʹ�ܻ�ʧ�ܵ��������
     * @note  ʧ�ܺ󣬵��������ֹͣ�����޷�ͨ��SetSpeed�ٴ���������������ʹ�ܡ�
     * @param motor:  ָ����ʵ���ָ�롣
     * @param enable: 1-ʹ�ܵ��, 0-ʧ�ܵ����
     * @retval 0: �ɹ�, -1: �����������
     */
    int8_t Motor_SetEnable(Motor_t *motor, uint8_t enable);
	
	
	extern Motor_t motor_A;

#ifdef __cplusplus
}
#endif

#endif

