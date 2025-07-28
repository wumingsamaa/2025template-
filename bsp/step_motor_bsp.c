#include "step_motor_bsp.h"

/**
 * @brief �����ʼ������
 */
void Step_Motor_Init(void)
{
    /* ʹ��X���� */
    Emm_V5_En_Control(&MOTOR_X_UART, MOTOR_X_ADDR, true, MOTOR_SYNC_FLAG);

    /* ʹ��Y���� */
    Emm_V5_En_Control(&MOTOR_Y_UART, MOTOR_Y_ADDR, true, MOTOR_SYNC_FLAG);

    /* ��ʼֹͣ */
    Step_Motor_Stop();
}

/**
 * @brief ����XY�����ٶ�
 * @param x_percent X���ٶȰٷֱȣ���Χ-100��100
 * @param y_percent Y���ٶȰٷֱȣ���Χ-100��100
 */
void Step_Motor_Set_Speed(int8_t x_percent, int8_t y_percent)
{
    uint8_t x_dir, y_dir;
    uint16_t x_speed, y_speed;

    /* ���ưٷֱȷ�Χ */
    if (x_percent > 100)
        x_percent = 100;
    if (x_percent < -100)
        x_percent = -100;
    if (y_percent > 100)
        y_percent = 100;
    if (y_percent < -100)
        y_percent = -100;

    /* ����X�᷽�� */
    if (x_percent >= 0)
    {
        x_dir = 0; /* CW���� */
    }
    else
    {
        x_dir = 1;              /* CCW���� */
        x_percent = -x_percent; /* ȡ����ֵ */
    }

    /* ����Y�᷽�� */
    if (y_percent >= 0)
    {
        y_dir = 0; /* CW���� */
    }
    else
    {
        y_dir = 1;              /* CCW���� */
        y_percent = -y_percent; /* ȡ����ֵ */
    }

    /* ����ʵ���ٶ�ֵ(�ٷֱ�ת��ΪRPM) */
    x_speed = (uint16_t)((x_percent * MOTOR_MAX_SPEED) / 100);
    y_speed = (uint16_t)((y_percent * MOTOR_MAX_SPEED) / 100);

	/* ��ӵ��Դ�ӡȷ�ϼ��� */
     my_printf(&huart1, "X: dir=%d, speed=%u; Y: dir=%d, speed=%u\r\n", x_dir, x_speed, y_dir, y_speed);
	
    /* ����X���� */
    Emm_V5_Vel_Control(&MOTOR_X_UART, MOTOR_X_ADDR, x_dir, x_speed, MOTOR_ACCEL, MOTOR_SYNC_FLAG);

    /* ����Y���� */
    Emm_V5_Vel_Control(&MOTOR_Y_UART, MOTOR_Y_ADDR, y_dir, y_speed, MOTOR_ACCEL, MOTOR_SYNC_FLAG);
}

/**
 * @brief ����XY�����ٶ�
 * @param x_rpm X��Ŀ���ٶȣ���λRPM (revolutions per minute)��
 *              ֧�ָ�ֵ��ʾ����
 *              �ٶȾ���Ϊ0.1RPM��С��0.05 RPM�ľ���ֵ��������Ϊ0��
 * @param y_rpm Y��Ŀ���ٶȣ���λRPM (revolutions per minute)��
 *              ֧�ָ�ֵ��ʾ����
 *              �ٶȾ���Ϊ0.1RPM��С��0.05 RPM�ľ���ֵ��������Ϊ0��
 */
void Step_Motor_Set_Speed_my(float x_rpm, float y_rpm)
{
    uint8_t x_dir, y_dir;
    uint16_t x_speed_scaled, y_speed_scaled; // �ٶ�ֵ����λΪ 0.1 RPM
    float abs_x_rpm, abs_y_rpm;

    /* 1. ��������RPM��Χ��ȷ�������������������ٶ� */
    // �������RPMֵǯλ�� [-MOTOR_MAX_SPEED, MOTOR_MAX_SPEED] ֮��
    if (x_rpm > MOTOR_MAX_SPEED)
    {
        x_rpm = MOTOR_MAX_SPEED;
    }
    else if (x_rpm < -MOTOR_MAX_SPEED)
    {
        x_rpm = -MOTOR_MAX_SPEED;
    }

    if (y_rpm > MOTOR_MAX_SPEED)
    {
        y_rpm = MOTOR_MAX_SPEED;
    }
    else if (y_rpm < -MOTOR_MAX_SPEED)
    {
        y_rpm = -MOTOR_MAX_SPEED;
    }

    /* 2. ����X�᷽��ͻ�ȡ�����ٶ� */
    if (x_rpm >= 0.0f)
    {
        x_dir = 0; /* CW���� (��ת) */
        abs_x_rpm = x_rpm;
    }
    else
    {
        x_dir = 1; /* CCW���� (��ת) */
        abs_x_rpm = -x_rpm; /* ȡ����ֵ */
    }

    /* 3. ����Y�᷽��ͻ�ȡ�����ٶ� */
    if (y_rpm >= 0.0f)
    {
        y_dir = 0; /* CW���� (��ת) */
        abs_y_rpm = y_rpm;
    }
    else
    {
        y_dir = 1; /* CCW���� (��ת) */
        abs_y_rpm = -y_rpm; /* ȡ����ֵ */
    }

    /* 4. ����ʵ�ʷ��͸�������������ٶ�ֵ (��λΪ 0.1 RPM) */
    // ��RPMֵ����10���õ���0.1RPMΪ��λ������ֵ��
    // ����0.5f��Ϊ�˽����������롣
    // ���������� 0.04 RPM (0.4 scaled) + 0.5f = 0.9f -> 0 (uint16_t)
    // 0.05 RPM (0.5 scaled) + 0.5f = 1.0f -> 1 (uint16_t)
    x_speed_scaled = (uint16_t)(abs_x_rpm * 10 + 0.5f);
    y_speed_scaled = (uint16_t)(abs_y_rpm * 10 + 0.5f);
    
    // �ٴμ�������� scaled speed �Ƿ񳬳� uint16_t �����ֵ��
    // ������������ǯλ�� (MOTOR_MAX_SPEED * RPM_TO_SCALED_FACTOR) ���ᳬ�� uint16_t��
    // ����Ϊ³���Լ�飬������Ӵ��С�
//    uint16_t max_scaled_speed = (uint16_t)(MOTOR_MAX_SPEED * 10 + 0.5f);
//    if (x_speed_scaled > max_scaled_speed) {
//        x_speed_scaled = max_scaled_speed;
//    }
//    if (y_speed_scaled > max_scaled_speed) {
//        y_speed_scaled = max_scaled_speed;
//    }


//    /* ��ӵ��Դ�ӡȷ�ϼ��� */
//    my_printf(&huart1, "X: input_rpm=%.1f, dir=%d, abs_rpm=%.1f, scaled_speed=%u; Y: input_rpm=%.1f, dir=%d, abs_rpm=%.1f, scaled_speed=%u\r\n",
//              x_rpm, x_dir, abs_x_rpm, x_speed_scaled,
//              y_rpm, y_dir, abs_y_rpm, y_speed_scaled);
    
    /* ����X���� */
    Emm_V5_Vel_Control(&MOTOR_X_UART, MOTOR_X_ADDR, x_dir, x_speed_scaled, MOTOR_ACCEL, MOTOR_SYNC_FLAG);

    /* ����Y���� */
    Emm_V5_Vel_Control(&MOTOR_Y_UART, MOTOR_Y_ADDR, y_dir, y_speed_scaled, MOTOR_ACCEL, MOTOR_SYNC_FLAG);
}

/**
 * @brief ����XY�����ƶ�һ�ξ��루ʹ��λ��ģʽ��
 * @param x_distance X���ƶ����루������������ֵΪCW���򣬸�ֵΪCCW����
 * @param y_distance Y���ƶ����루������������ֵΪCW���򣬸�ֵΪCCW����
 */
void Step_Motor_Set_Pwm(int32_t x_distance, int32_t y_distance)
{
    uint8_t x_dir, y_dir;
    uint32_t x_clk, y_clk;
    uint16_t speed = MOTOR_MAX_SPEED;  /* ʹ������ٶȣ��������Ҫ���� */
    uint8_t acc = MOTOR_ACCEL;        /* ʹ��Ԥ������ٶ� */

    /* ����X�᷽��������� */
    if (x_distance >= 0)
    {
        x_dir = 0; /* CW���� */
        x_clk = (uint32_t)x_distance;
    }
    else
    {
        x_dir = 1; /* CCW���� */
        x_clk = (uint32_t)(-x_distance); /* ȡ����ֵ */
    }

    /* ����Y�᷽��������� */
    if (y_distance >= 0)
    {
        y_dir = 0; /* CW���� */
        y_clk = (uint32_t)y_distance;
    }
    else
    {
        y_dir = 1; /* CCW���� */
        y_clk = (uint32_t)(-y_distance); /* ȡ����ֵ */
    }

    /* ����X����������˶��������þ���ģʽ�� */
    Emm_V5_Pos_Control(&MOTOR_X_UART, MOTOR_X_ADDR, x_dir, speed, acc, x_clk, false, MOTOR_SYNC_FLAG);

    /* ����Y����������˶��������þ���ģʽ�� */
    Emm_V5_Pos_Control(&MOTOR_Y_UART, MOTOR_Y_ADDR, y_dir, speed, acc, y_clk, false, MOTOR_SYNC_FLAG);
}


/**
 * @brief ֹͣ���е��
 */
void Step_Motor_Stop(void)
{
    /* ֹͣX���� */
    Emm_V5_Stop_Now(&MOTOR_X_UART, MOTOR_X_ADDR, MOTOR_SYNC_FLAG);

    /* ֹͣY���� */
    Emm_V5_Stop_Now(&MOTOR_Y_UART, MOTOR_Y_ADDR, MOTOR_SYNC_FLAG);
}

void step_motor_proc(void)
{
	
}

