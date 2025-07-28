#include "motor_bsp.h"

Motor_t motor1;  // �����
Motor_t motor2;  // �Ҳ���

float speed_set[3] = {-65,65,0};
uint8_t set_index = 0;

void motor_init(void)
{
	// �������1 - �����
	Motor_Create(&motor1, 
             &htim1,           // TIM1��ʱ��
             TIM_CHANNEL_2,    // ͨ��2 (PE11)
						 GPIOE,            // GPIOE�˿�
             GPIO_PIN_9,       // PE9����
             GPIOE,            // GPIOE�˿�
             GPIO_PIN_10,       // PE9����
             GPIO_PIN_SET);    // ��תʱ����ߵ�ƽ

	// �������2 - �Ҳ���
	Motor_Create(&motor2, 
             &htim1,           // TIM1��ʱ��
             TIM_CHANNEL_4,    // ͨ��4 (PE14)
             GPIOE,            // GPIOE�˿�
             GPIO_PIN_12,      // PE13����
	           GPIOE,            // GPIOE�˿�
             GPIO_PIN_13,      // PE13����
             GPIO_PIN_SET);    // ��תʱ����ߵ�ƽ
}

float pid_speed_left_out ,pid_speed_right_out;//�ٶȻ�pid����ֵ

void motor_proc(void)
{
//    //���ǰ��
//    Motor_SetSpeed(&motor1, 50);   // ����50%�ٶ�
//    Motor_SetSpeed(&motor2, 50);   // �ҵ��50%�ٶ�
//    
//    //���ֹͣ
//    Motor_Stop(&motor1);
//    Motor_Stop(&motor2);

//	pid_speed_left_out = pid_calc(&pid_speed_left,left_encoder.speed_cm_s,speed_set[set_index],0);
//	Motor_SetSpeed(&motor1, pid_speed_left_out);
	
//	pid_speed_right_out = pid_calc(&pid_speed_right,right_encoder.speed_cm_s,speed_set[set_index],0);
//	Motor_SetSpeed(&motor2, pid_speed_right_out);
//	my_printf(&huart1,"%.1f,%.1f\r\n",right_encoder.speed_cm_s,pid_speed_right_out);
	
	Location_Speed_control();
	
}


float location_left_Outval,location_right_Outval;//λ�û�pid����ֵ
void Location_Speed_control(void) 
{
	/*�ٶȻ�*/
	location_left_Outval  =  pid_calc(&pid_location_left, left_encoder.total_count  ,pid_location_left.set ,0 );
	location_right_Outval =  pid_calc(&pid_location_right,right_encoder.total_count ,pid_location_right.set ,0 );
	/*λ�û�*/
	pid_speed_left_out  = pid_calc(&pid_speed_left,left_encoder.speed_cm_s,location_left_Outval,0);
	pid_speed_right_out = pid_calc(&pid_speed_right,right_encoder.speed_cm_s,location_right_Outval,0);
	
	Motor_SetSpeed(&motor1, pid_speed_left_out);
	Motor_SetSpeed(&motor2, pid_speed_right_out);
}








