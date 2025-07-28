#include "schedule.h"

typedef struct {
	void (*task_func)(void);
	uint32_t rate_ms;
	uint32_t last_run;
}schedule_task_t;

uint8_t task_num;

static schedule_task_t schedule_task[] = {
//	{oled_proc,100,0},
	{uart_proc,1,0},
	{pi_proc,20,0}

//	{motor_proc,20,0},
//	{encoder_proc,20,0},
//	{key_proc,10,0},
//	{gray_proc,20,0}
	
};

/**
 * @brief ��������ʼ������
 * �������������Ԫ�ظ�������������洢�� task_num ��
 */
void schedule_init(void)
{
	task_num = sizeof(schedule_task) / sizeof(schedule_task_t);
}

/**
 * @brief ���������к���
 * �����������飬����Ƿ���������Ҫִ�С������ǰʱ���Ѿ����������ִ�����ڣ���ִ�и����񲢸����ϴ�����ʱ��
 */
void schedule_run(void)
{
    // �������������е���������
    for (uint8_t i = 0; i < task_num; i++)
    {
        // ��ȡ��ǰ��ϵͳʱ�䣨���룩
        uint32_t now_time = HAL_GetTick();

        // ��鵱ǰʱ���Ƿ�ﵽ�����ִ��ʱ��
        if (now_time >= schedule_task[i].rate_ms + schedule_task[i].last_run)
        {
            // ����������ϴ�����ʱ��Ϊ��ǰʱ��
            schedule_task[i].last_run = now_time;

            // ִ��������
            schedule_task[i].task_func();
        }
    }
}
