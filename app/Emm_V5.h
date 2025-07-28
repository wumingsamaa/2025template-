#ifndef __EMM_V5_H
#define __EMM_V5_H

#include "usart.h"
#include "stdbool.h"
#include "string.h"

#define   EMM_UART_TIMEOUT  1000  /* ���ڷ��ͳ�ʱʱ��(ms) */

/**********************************************************
***	Emm_V5.0�����ջ���������
***	��д���ߣ�ZHANGDATOU
***	����֧�֣��Ŵ�ͷ�ջ��ŷ�
***	�Ա����̣�https://zhangdatou.taobao.com
***	CSDN���ͣ�http s://blog.csdn.net/zhangdatou666
***	qq����Ⱥ��262438510
**********************************************************/

#define		ABS(x)		((x) > 0 ? (x) : -(x)) 

typedef enum {
	S_VER   = 0,			/* ��ȡ�̼��汾�Ͷ�Ӧ��Ӳ���汾 */
	S_RL    = 1,			/* ��ȡ��ȡ���������� */
	S_PID   = 2,			/* ��ȡPID���� */
	S_VBUS  = 3,			/* ��ȡ���ߵ�ѹ */
	S_CPHA  = 5,			/* ��ȡ����� */
	S_ENCL  = 7,			/* ��ȡ�������Ի�У׼��ı�����ֵ */
	S_TPOS  = 8,			/* ��ȡ���Ŀ��λ�ýǶ� */
	S_VEL   = 9,			/* ��ȡ���ʵʱת�� */
	S_CPOS  = 10,			/* ��ȡ���ʵʱλ�ýǶ� */
	S_PERR  = 11,			/* ��ȡ���λ�����Ƕ� */
	S_FLAG  = 13,			/* ��ȡʹ��/��λ/��ת״̬��־λ */
	S_Conf  = 14,			/* ��ȡ�������� */
	S_State = 15,			/* ��ȡϵͳ״̬���� */
	S_ORG   = 16,     /* ��ȡ���ڻ���/����ʧ��״̬��־λ */
}SysParams_t;

/* ����������ݽṹ */
typedef struct {
  uint8_t  addr;                  /* �����ַ */
  uint8_t  func;                  /* ������ */
  uint8_t  dir;                   /* ����0ΪCW��1ΪCCW */
  int16_t  speed;                 /* ʵʱ�ٶ�(RPM) */
  int32_t  position;              /* ʵʱλ��(������) */
  uint8_t  status;                /* ���״̬ */
  uint8_t  error;                 /* ������� */
  int32_t  encoder;               /* ������λ�� */
  int16_t  temperature;           /* �¶� */
  uint16_t voltage;               /* ĸ�ߵ�ѹ(V) */
  uint16_t current;               /* �����(mA) */
  int32_t  target_pos;            /* Ŀ��λ�� */
  uint16_t target_speed;          /* Ŀ���ٶ� */
  uint8_t  acceleration;          /* ���ٶ� */
  uint8_t  subdivision;           /* ϸ������ */
  uint8_t  ctrl_mode;             /* ����ģʽ */
  uint8_t  protection;            /* �������� */
  uint16_t pwm_duty;              /* PWMռ�ձ� */
  uint8_t  closed_loop_state;     /* �ջ�״̬ */
  uint8_t  encoder_state;         /* ������У׼״̬ */
  uint8_t  sync_state;            /* ���ͬ��״̬ */
  uint8_t  origin_state;          /* ����״̬ */
  char     version[16];           /* �̼��汾 */
  uint8_t  raw_data[32];          /* ԭʼ���� */
  uint8_t  data_len;              /* ���ݳ��� */
  uint8_t  valid;                 /* ������Ч��־ */
} Emm_V5_Response_t;


/**********************************************************
*** ע�⣺ÿ�������Ĳ����ľ���˵��������Ķ�Ӧ������ע��˵��
**********************************************************/
void Emm_V5_Reset_CurPos_To_Zero(UART_HandleTypeDef* huart, uint8_t addr); // ����ǰλ������
void Emm_V5_Reset_Clog_Pro(UART_HandleTypeDef* huart, uint8_t addr); // �����ת����
void Emm_V5_Read_Sys_Params(UART_HandleTypeDef* huart, uint8_t addr, SysParams_t s); // ��ȡ����
void Emm_V5_Modify_Ctrl_Mode(UART_HandleTypeDef* huart, uint8_t addr, bool svF, uint8_t ctrl_mode); // ���������޸Ŀ���/�ջ�����ģʽ
void Emm_V5_En_Control(UART_HandleTypeDef* huart, uint8_t addr, bool state, bool snF); // ���ʹ�ܿ���
void Emm_V5_Vel_Control(UART_HandleTypeDef* huart, uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF); // �ٶ�ģʽ����
void Emm_V5_Pos_Control(UART_HandleTypeDef* huart, uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF); // λ��ģʽ����
void Emm_V5_Stop_Now(UART_HandleTypeDef* huart, uint8_t addr, bool snF); // �õ������ֹͣ�˶�
void Emm_V5_Synchronous_motion(UART_HandleTypeDef* huart, uint8_t addr); // �������ͬ����ʼ�˶�
void Emm_V5_Origin_Set_O(UART_HandleTypeDef* huart, uint8_t addr, bool svF); // ���õ�Ȧ��������λ��
void Emm_V5_Origin_Modify_Params(UART_HandleTypeDef* huart, uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF); // �޸Ļ������
void Emm_V5_Origin_Trigger_Return(UART_HandleTypeDef* huart, uint8_t addr, uint8_t o_mode, bool snF); // �������������
void Emm_V5_Origin_Interrupt(UART_HandleTypeDef* huart, uint8_t addr); // ǿ���жϲ��˳�����
uint8_t Emm_V5_Parse_Response(uint8_t *buffer, uint8_t len, Emm_V5_Response_t *resp);

#endif
