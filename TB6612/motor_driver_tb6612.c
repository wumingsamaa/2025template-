
/**
 ******************************************************************************
 * @file   	motor_driver.c
 * @brief   TB6612 ���������Դ�ļ�
 * @author  (�������)
 * @date    2025��07��08��
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "motor_driver_tb6612.h"

/* Private function prototypes -----------------------------------------------*/
// �ڲ�˽�к��������𽫰ٷֱ��ٶ�ӳ��ΪPWMֵ
static uint32_t Speed_To_PWM(int16_t speed);
// �ڲ�˽�к�����������֤����ṹ��ָ���Ƿ���Ч
static int8_t Motor_ValidateParams(Motor_t *motor);

/* Exported functions --------------------------------------------------------*/

/**
 * @brief ��ʼ��TB6612оƬ��������STBY���š�
 */
void TB6612_Init(uint8_t enable)
{
    // ���ô˺���ǰ����ȷ��STBY���Ŷ�Ӧ��GPIOʱ������MX_GPIO_Init()��ʹ��
    if (enable)
    {
        HAL_GPIO_WritePin(TB6612_STBY_PORT, TB6612_STBY_PIN, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(TB6612_STBY_PORT, TB6612_STBY_PIN, GPIO_PIN_RESET);
    }
}

/**
 * @brief ��������ʼ��һ�����ʵ����
 */
int8_t Motor_Create(Motor_t *motor,
                    TIM_HandleTypeDef *htim,
                    uint32_t channel,
                    GPIO_TypeDef *in1_port, uint16_t in1_pin,
                    GPIO_TypeDef *in2_port, uint16_t in2_pin,
                    uint8_t reverse)
{
    // 1. ������Ч�Լ��
    if (motor == NULL || htim == NULL || in1_port == NULL || in2_port == NULL)
    {
        return -1;
    }
    // ���PWMͨ���Ƿ�Ϸ�
    if (channel != TIM_CHANNEL_1 && channel != TIM_CHANNEL_2 &&
        channel != TIM_CHANNEL_3 && channel != TIM_CHANNEL_4)
    {
        return -1;
    }

    // 2. ���Ӳ��������Ϣ���ṹ��
    motor->hw.htim = htim;
    motor->hw.channel = channel;
    motor->hw.in1_port = in1_port;
    motor->hw.in1_pin = in1_pin;
    motor->hw.in2_port = in2_port;
    motor->hw.in2_pin = in2_pin;

    // 3. ��ʼ������߼�״̬
    motor->speed = 0;
    motor->state = MOTOR_STATE_STOP;
    motor->enable = 1; // Ĭ�����ʹ��
    motor->reverse = reverse;

    // 4. ������Ӧ��PWMͨ��
    HAL_TIM_PWM_Start(motor->hw.htim, motor->hw.channel);

    // 5. ���ó�ʼ����״̬��ɲ�� (IN1=LOW, IN2=LOW, PWM=0)
    HAL_GPIO_WritePin(motor->hw.in1_port, motor->hw.in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor->hw.in2_port, motor->hw.in2_pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(motor->hw.htim, motor->hw.channel, 0);

    return 0; // ��ʼ���ɹ�
}

/**
 * @brief ���õ���ٶȡ�
 */
int8_t Motor_SetSpeed(Motor_t *motor, int16_t speed)
{
    // 1. �������
    if (Motor_ValidateParams(motor) != 0)
        return -1;
    if (speed < MOTOR_LOGIC_SPEED_MIN || speed > MOTOR_LOGIC_SPEED_MAX)
        return -1;

    // 2. ������Ƿ����ʹ��
    if (!motor->enable)
        return -1;

    // 3. �����߼��ٶ�ֵ
    motor->speed = speed;

    // 4. �������Ƿ���װ�ģ����ٶ��߼�ȡ��
    if (motor->reverse)
    {
        speed = -speed;
    }

    // 5. �����ٶȵ�����������TB6612��IN1/IN2���Ų�����PWMֵ
    if (speed == 0)
    {
        // --- ɲ��ģʽ ---
        // TB6612�߼���IN1��IN2ͬΪ�ߵ�ƽ��͵�ƽ��Ϊɲ�����ߵ�ƽɲ��Ч����ǿ��
        HAL_GPIO_WritePin(motor->hw.in1_port, motor->hw.in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor->hw.in2_port, motor->hw.in2_pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(motor->hw.htim, motor->hw.channel, 0);
        motor->state = MOTOR_STATE_STOP;
    }
    else if (speed > 0)
    {
        // --- ��תģʽ ---
        // TB6612�߼���IN1=HIGH, IN2=LOW
        HAL_GPIO_WritePin(motor->hw.in1_port, motor->hw.in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor->hw.in2_port, motor->hw.in2_pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(motor->hw.htim, motor->hw.channel, Speed_To_PWM(speed));
        motor->state = MOTOR_STATE_FORWARD;
    }
    else
    { // speed < 0
        // --- ��תģʽ ---
        // TB6612�߼���IN1=LOW, IN2=HIGH
        HAL_GPIO_WritePin(motor->hw.in1_port, motor->hw.in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->hw.in2_port, motor->hw.in2_pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(motor->hw.htim, motor->hw.channel, Speed_To_PWM(speed));
        motor->state = MOTOR_STATE_BACKWARD;
    }

    return 0;
}

/**
 * @brief ����ֹͣ��� (ɲ��ģʽ)��
 */
int8_t Motor_Stop(Motor_t *motor)
{
    // ֱ�ӵ���SetSpeed(0)��ʵ��ɲ��
    return Motor_SetSpeed(motor, 0);
}

/**
 * @brief ��ȡ�����ǰ״̬��
 */
MotorState_t Motor_GetState(Motor_t *motor)
{
    if (Motor_ValidateParams(motor) != 0)
    {
        return MOTOR_STATE_ERROR;
    }
    return motor->state;
}

/**
 * @brief (�������)ʹ�ܻ�ʧ�ܵ��������
 */
int8_t Motor_SetEnable(Motor_t *motor, uint8_t enable)
{
    if (Motor_ValidateParams(motor) != 0)
    {
        return -1;
    }

    motor->enable = enable;

    // �����ʧ�ܲ�����������ֹͣ���
    if (!enable)
    {
        Motor_Stop(motor);
    }

    return 0;
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  ���߼��ٶ�ӳ��Ϊ����PWMֵ(0~ARR)��
 * @param  speed: �߼��ٶ�ֵ��
 * @retval ����PWM����ֵ��
 */
static uint32_t Speed_To_PWM(int16_t speed)
{
    // 1. ȡ�ٶȵľ���ֵ
    int16_t abs_speed = (speed < 0) ? -speed : speed;

    // 2. ����ӳ�乫ʽ��
    //    ����PWMֵ = (�߼��ٶ� / �߼�����ٶ�) * �������PWMֵ(ARR)
    //    ���� (MOTOR_PWM_MAX_PULSE - 1) ���Ƕ�ʱ����ARRֵ
    uint32_t pwm_value = (uint32_t)abs_speed * (MOTOR_PWM_MAX_PULSE - 1) / MOTOR_LOGIC_SPEED_MAX;

    // 3. Ӧ����С������ֵ (��ѡ����)
    //    ��ֹ���ٶȺܵ�ʱ�������ΪŤ���������ת
#if MOTOR_MIN_PHYSICAL_PWM > 0
    if (pwm_value > 0 && pwm_value < MOTOR_MIN_PHYSICAL_PWM)
    {
        pwm_value = MOTOR_MIN_PHYSICAL_PWM;
    }
#endif

    // 4. �������ռ������PWMֵ
    return pwm_value;
}

/**
 * @brief ��֤����ṹ��ָ�뼰���ڲ��ؼ�ָ���Ƿ���Ч��
 */
static int8_t Motor_ValidateParams(Motor_t *motor)
{
    if (motor == NULL || motor->hw.htim == NULL)
    {
        return -1; // ��Ч
    }
    return 0; // ��Ч
}


