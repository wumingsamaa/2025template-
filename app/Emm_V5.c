#include "Emm_V5.h"
#include "uart_bsp.h"

/**********************************************************
***	Emm_V5.0�����ջ���������
***	��д���ߣ�ZHANGDATOU
***	����֧�֣��Ŵ�ͷ�ջ��ŷ�
***	�Ա����̣�https://zhangdatou.taobao.com
***	CSDN���ͣ�http s://blog.csdn.net/zhangdatou666
***	qq����Ⱥ��262438510
**********************************************************/

/**
  * @brief    ����ǰλ������
  * @param    huart �����ھ��
  * @param    addr  �������ַ
  * @retval   ��ַ + ������ + ����״̬ + У���ֽ�
  */
 void Emm_V5_Reset_CurPos_To_Zero(UART_HandleTypeDef* huart, uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // װ������
  cmd[0] =  addr;                       // ��ַ
  cmd[1] =  0x0A;                       // ������
  cmd[2] =  0x6D;                       // ������
  cmd[3] =  0x6B;                       // У���ֽ�
  
  // ��������
	HAL_UART_Transmit(huart, (uint8_t *)cmd, 4, EMM_UART_TIMEOUT);
}

/**
  * @brief    �����ת����
  * @param    huart �����ھ��
  * @param    addr  �������ַ
  * @retval   ��ַ + ������ + ����״̬ + У���ֽ�
  */
void Emm_V5_Reset_Clog_Pro(UART_HandleTypeDef* huart, uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // װ������
  cmd[0] =  addr;                       // ��ַ
  cmd[1] =  0x0E;                       // ������
  cmd[2] =  0x52;                       // ������
  cmd[3] =  0x6B;                       // У���ֽ�
  
  // ��������
  HAL_UART_Transmit(huart, (uint8_t *)cmd, 4, EMM_UART_TIMEOUT);
}

/**
  * @brief    ��ȡϵͳ����
  * @param    huart �����ھ��
  * @param    addr  �������ַ
  * @param    s     ��ϵͳ��������
  * @retval   ��ַ + ������ + ����״̬ + У���ֽ�
  */
void Emm_V5_Read_Sys_Params(UART_HandleTypeDef* huart, uint8_t addr, SysParams_t s)
{
  uint8_t i = 0;
  uint8_t cmd[16] = {0};
  
  // װ������
  cmd[i] = addr; ++i;                   // ��ַ

  switch(s)                             // ������
  {
    case S_VER  : cmd[i] = 0x1F; ++i; break;
    case S_RL   : cmd[i] = 0x20; ++i; break;
    case S_PID  : cmd[i] = 0x21; ++i; break;
    case S_VBUS : cmd[i] = 0x24; ++i; break;
    case S_CPHA : cmd[i] = 0x27; ++i; break;
    case S_ENCL : cmd[i] = 0x31; ++i; break;
    case S_TPOS : cmd[i] = 0x33; ++i; break;
    case S_VEL  : cmd[i] = 0x35; ++i; break;
    case S_CPOS : cmd[i] = 0x36; ++i; break;
    case S_PERR : cmd[i] = 0x37; ++i; break;
    case S_FLAG : cmd[i] = 0x3A; ++i; break;
    case S_ORG  : cmd[i] = 0x3B; ++i; break;
    case S_Conf : cmd[i] = 0x42; ++i; cmd[i] = 0x6C; ++i; break;
    case S_State: cmd[i] = 0x43; ++i; cmd[i] = 0x7A; ++i; break;
    default: break;
  }

  cmd[i] = 0x6B; ++i;                   // У���ֽ�
  
  // ���������ȡ����״̬
  HAL_UART_Transmit(huart, (uint8_t *)cmd, i, EMM_UART_TIMEOUT);
  
}

/**
  * @brief    �޸Ŀ���/�ջ�����ģʽ
  * @param    huart    �����ھ��
  * @param    addr     �������ַ
  * @param    svF      ���Ƿ�洢��־��falseΪ���洢��trueΪ�洢
  * @param    ctrl_mode������ģʽ����Ӧ��Ļ�ϵ�P_Pul�˵�����0�ǹر������������ţ�1�ǿ���ģʽ��2�Ǳջ�ģʽ��3����En�˿ڸ���Ϊ��Ȧ��λ�����������ţ�Dir�˿ڸ���Ϊ��λ����ߵ�ƽ����
  * @retval   ��ַ + ������ + ����״̬ + У���ֽ�
  */
void Emm_V5_Modify_Ctrl_Mode(UART_HandleTypeDef* huart, uint8_t addr, bool svF, uint8_t ctrl_mode)
{
  uint8_t cmd[16] = {0};
  
  // װ������
  cmd[0] =  addr;                       // ��ַ
  cmd[1] =  0x46;                       // ������
  cmd[2] =  0x69;                       // ������
  cmd[3] =  svF;                        // �Ƿ�洢��־��falseΪ���洢��trueΪ�洢
  cmd[4] =  ctrl_mode;                  // ����ģʽ����Ӧ��Ļ�ϵ�P_Pul�˵�����0�ǹر������������ţ�1�ǿ���ģʽ��2�Ǳջ�ģʽ��3����En�˿ڸ���Ϊ��Ȧ��λ�����������ţ�Dir�˿ڸ���Ϊ��λ����ߵ�ƽ����
  cmd[5] =  0x6B;                       // У���ֽ�
  
  // ��������
  HAL_UART_Transmit(huart, (uint8_t *)cmd, 6, EMM_UART_TIMEOUT);
}

/**
  * @brief    ʹ���źſ���
  * @param    huart �����ھ��
  * @param    addr  �������ַ
  * @param    state ��ʹ��״̬     ��trueΪʹ�ܵ����falseΪ�رյ��
  * @param    snF   �����ͬ����־ ��falseΪ�����ã�trueΪ����
  * @retval   ��ַ + ������ + ����״̬ + У���ֽ�
  */
void Emm_V5_En_Control(UART_HandleTypeDef* huart, uint8_t addr, bool state, bool snF)
{
  uint8_t cmd[16] = {0};
  
  // װ������
  cmd[0] =  addr;                       // ��ַ
  cmd[1] =  0xF3;                       // ������
  cmd[2] =  0xAB;                       // ������
  cmd[3] =  (uint8_t)state;             // ʹ��״̬
  cmd[4] =  snF;                        // ���ͬ���˶���־
  cmd[5] =  0x6B;                       // У���ֽ�
  
  // ��������
  HAL_UART_Transmit(huart, (uint8_t *)cmd, 6, EMM_UART_TIMEOUT);
}

/**
  * @brief    �ٶ�ģʽ
  * @param    huart�����ھ��
  * @param    addr�������ַ
  * @param    dir ������       ��0ΪCW������ֵΪCCW
  * @param    vel ���ٶ�       ����Χ0 - 5000RPM
  * @param    acc �����ٶ�     ����Χ0 - 255��ע�⣺0��ֱ������
  * @param    snF �����ͬ����־��falseΪ�����ã�trueΪ����
  * @retval   ��ַ + ������ + ����״̬ + У���ֽ�
  */
void Emm_V5_Vel_Control(UART_HandleTypeDef* huart, uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
  uint8_t cmd[16] = {0};

  // װ������
  cmd[0] =  addr;                       // ��ַ
  cmd[1] =  0xF6;                       // ������
  cmd[2] =  dir;                        // ����
  cmd[3] =  (uint8_t)(vel >> 8);        // �ٶ�(RPM)��8λ�ֽ�
  cmd[4] =  (uint8_t)(vel >> 0);        // �ٶ�(RPM)��8λ�ֽ�
  cmd[5] =  acc;                        // ���ٶȣ�ע�⣺0��ֱ������
  cmd[6] =  snF;                        // ���ͬ���˶���־
  cmd[7] =  0x6B;                       // У���ֽ�
  
  // ��������
	HAL_StatusTypeDef status = HAL_UART_Transmit(huart, (uint8_t *)cmd, 8, EMM_UART_TIMEOUT);
	if (status != HAL_OK) 
	{
		my_printf(&huart1, "UART Transmit Failed! Error: %d\r\n", status);  // status �� HAL_TIMEOUT
	} 
	else
	{
		my_printf(&huart1, "Sent Cmd to Addr %d: ", addr);
		for (int i = 0; i < 8; i++) 
		{  // ��ӡ�����ֽ�
			my_printf(&huart1, "0x%02X ", cmd[i]);
		}
    my_printf(&huart1, "\r\n");
	}
}

/**
  * @brief    λ��ģʽ
  * @param    huart�����ھ��
  * @param    addr�������ַ
  * @param    dir ������        ��0ΪCW������ֵΪCCW
  * @param    vel ���ٶ�(RPM)   ����Χ0 - 5000RPM
  * @param    acc �����ٶ�      ����Χ0 - 255��ע�⣺0��ֱ������
  * @param    clk ��������      ����Χ0- (2^32 - 1)��
  * @param    raF ����λ/���Ա�־��falseΪ����˶���trueΪ����ֵ�˶�
  * @param    snF �����ͬ����־ ��falseΪ�����ã�trueΪ����
  * @retval   ��ַ + ������ + ����״̬ + У���ֽ�
  */
void Emm_V5_Pos_Control(UART_HandleTypeDef* huart, uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF)
{
  uint8_t cmd[16] = {0};

  // װ������
  cmd[0]  =  addr;                      // ��ַ
  cmd[1]  =  0xFD;                      // ������
  cmd[2]  =  dir;                       // ����
  cmd[3]  =  (uint8_t)(vel >> 8);       // �ٶ�(RPM)��8λ�ֽ�
  cmd[4]  =  (uint8_t)(vel >> 0);       // �ٶ�(RPM)��8λ�ֽ� 
  cmd[5]  =  acc;                       // ���ٶȣ�ע�⣺0��ֱ������
  cmd[6]  =  (uint8_t)(clk >> 24);      // ������(bit24 - bit31)
  cmd[7]  =  (uint8_t)(clk >> 16);      // ������(bit16 - bit23)
  cmd[8]  =  (uint8_t)(clk >> 8);       // ������(bit8  - bit15)
  cmd[9]  =  (uint8_t)(clk >> 0);       // ������(bit0  - bit7 )
  cmd[10] =  raF;                       // ��λ/���Ա�־��falseΪ����˶���trueΪ����ֵ�˶�
  cmd[11] =  snF;                       // ���ͬ���˶���־��falseΪ�����ã�trueΪ����
  cmd[12] =  0x6B;                      // У���ֽ�
  
  // ��������
  HAL_UART_Transmit(huart, (uint8_t *)cmd, 13, EMM_UART_TIMEOUT);
}

/**
  * @brief    ����ֹͣ�����п���ģʽ��ͨ�ã�
  * @param    huart �����ھ��
  * @param    addr  �������ַ
  * @param    snF   �����ͬ����־��falseΪ�����ã�trueΪ����
  * @retval   ��ַ + ������ + ����״̬ + У���ֽ�
  */
void Emm_V5_Stop_Now(UART_HandleTypeDef* huart, uint8_t addr, bool snF)
{
  uint8_t cmd[16] = {0};
  
  // װ������
  cmd[0] =  addr;                       // ��ַ
  cmd[1] =  0xFE;                       // ������
  cmd[2] =  0x98;                       // ������
  cmd[3] =  snF;                        // ���ͬ���˶���־
  cmd[4] =  0x6B;                       // У���ֽ�
  
  // ��������
  HAL_UART_Transmit(huart, (uint8_t *)cmd, 5, EMM_UART_TIMEOUT);
}

/**
  * @brief    ���ͬ���˶�
  * @param    huart �����ھ��
  * @param    addr  �������ַ
  * @retval   ��ַ + ������ + ����״̬ + У���ֽ�
  */
void Emm_V5_Synchronous_motion(UART_HandleTypeDef* huart, uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // װ������
  cmd[0] =  addr;                       // ��ַ
  cmd[1] =  0xFF;                       // ������
  cmd[2] =  0x66;                       // ������
  cmd[3] =  0x6B;                       // У���ֽ�
  
  // ��������
  HAL_UART_Transmit(huart, (uint8_t *)cmd, 4, EMM_UART_TIMEOUT);
}

/**
  * @brief    ���õ�Ȧ��������λ��
  * @param    huart �����ھ��
  * @param    addr  �������ַ
  * @param    svF   ���Ƿ�洢��־��falseΪ���洢��trueΪ�洢
  * @retval   ��ַ + ������ + ����״̬ + У���ֽ�
  */
void Emm_V5_Origin_Set_O(UART_HandleTypeDef* huart, uint8_t addr, bool svF)
{
   uint8_t cmd[16] = {0};
  
  // װ������
  cmd[0] =  addr;                       // ��ַ
  cmd[1] =  0x93;                       // ������
  cmd[2] =  0x88;                       // ������
  cmd[3] =  svF;                        // �Ƿ�洢��־��falseΪ���洢��trueΪ�洢
  cmd[4] =  0x6B;                       // У���ֽ�
  
  // ��������
  HAL_UART_Transmit(huart, (uint8_t *)cmd, 5, EMM_UART_TIMEOUT);
}

/**
  * @brief    �޸Ļ������
  * @param    huart  �����ھ��
  * @param    addr   �������ַ
  * @param    svF    ���Ƿ�洢��־��falseΪ���洢��trueΪ�洢
  * @param    o_mode ������ģʽ��0Ϊ��Ȧ�ͽ����㣬1Ϊ��Ȧ������㣬2Ϊ��Ȧ����λ��ײ���㣬3Ϊ��Ȧ����λ���ػ���
  * @param    o_dir  �����㷽��0ΪCW������ֵΪCCW
  * @param    o_vel  �������ٶȣ���λ��RPM��ת/���ӣ�
  * @param    o_tm   �����㳬ʱʱ�䣬��λ������
  * @param    sl_vel ������λ��ײ������ת�٣���λ��RPM��ת/���ӣ�
  * @param    sl_ma  ������λ��ײ�������������λ��Ma��������
  * @param    sl_ms ������λ��ײ������ʱ�䣬��λ��Ms�����룩
  * @param    potF   ���ϵ��Զ��������㣬falseΪ��ʹ�ܣ�trueΪʹ��
  * @retval   ��ַ + ������ + ����״̬ + У���ֽ�
  */
void Emm_V5_Origin_Modify_Params(UART_HandleTypeDef* huart, uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF)
{
  uint8_t cmd[32] = {0};
  
  // װ������
  cmd[0] =  addr;                       // ��ַ
  cmd[1] =  0x4C;                       // ������
  cmd[2] =  0xAE;                       // ������
  cmd[3] =  svF;                        // �Ƿ�洢��־��falseΪ���洢��trueΪ�洢
  cmd[4] =  o_mode;                     // ����ģʽ��0Ϊ��Ȧ�ͽ����㣬1Ϊ��Ȧ������㣬2Ϊ��Ȧ����λ��ײ���㣬3Ϊ��Ȧ����λ���ػ���
  cmd[5] =  o_dir;                      // ���㷽��
  cmd[6]  =  (uint8_t)(o_vel >> 8);     // �����ٶ�(RPM)��8λ�ֽ�
  cmd[7]  =  (uint8_t)(o_vel >> 0);     // �����ٶ�(RPM)��8λ�ֽ� 
  cmd[8]  =  (uint8_t)(o_tm >> 24);     // ���㳬ʱʱ��(bit24 - bit31)
  cmd[9]  =  (uint8_t)(o_tm >> 16);     // ���㳬ʱʱ��(bit16 - bit23)
  cmd[10] =  (uint8_t)(o_tm >> 8);      // ���㳬ʱʱ��(bit8  - bit15)
  cmd[11] =  (uint8_t)(o_tm >> 0);      // ���㳬ʱʱ��(bit0  - bit7 )
  cmd[12] =  (uint8_t)(sl_vel >> 8);    // ����λ��ײ������ת��(RPM)��8λ�ֽ�
  cmd[13] =  (uint8_t)(sl_vel >> 0);    // ����λ��ײ������ת��(RPM)��8λ�ֽ� 
  cmd[14] =  (uint8_t)(sl_ma >> 8);     // ����λ��ײ���������(Ma)��8λ�ֽ�
  cmd[15] =  (uint8_t)(sl_ma >> 0);     // ����λ��ײ���������(Ma)��8λ�ֽ� 
  cmd[16] =  (uint8_t)(sl_ms >> 8);     // ����λ��ײ������ʱ��(Ms)��8λ�ֽ�
  cmd[17] =  (uint8_t)(sl_ms >> 0);     // ����λ��ײ������ʱ��(Ms)��8λ�ֽ�
  cmd[18] =  potF;                      // �ϵ��Զ��������㣬falseΪ��ʹ�ܣ�trueΪʹ��
  cmd[19] =  0x6B;                      // У���ֽ�
  
  // ��������
  HAL_UART_Transmit(huart, (uint8_t *)cmd, 20, EMM_UART_TIMEOUT);
}

/**
  * @brief    ��������
  * @param    huart  �����ھ��
  * @param    addr   �������ַ
  * @param    o_mode ������ģʽ��0Ϊ��Ȧ�ͽ����㣬1Ϊ��Ȧ������㣬2Ϊ��Ȧ����λ��ײ���㣬3Ϊ��Ȧ����λ���ػ���
  * @param    snF    �����ͬ����־��falseΪ�����ã�trueΪ����
  * @retval   ��ַ + ������ + ����״̬ + У���ֽ�
  */
void Emm_V5_Origin_Trigger_Return(UART_HandleTypeDef* huart, uint8_t addr, uint8_t o_mode, bool snF)
{
  uint8_t cmd[16] = {0};
  
  // װ������
  cmd[0] =  addr;                       // ��ַ
  cmd[1] =  0x9A;                       // ������
  cmd[2] =  o_mode;                     // ����ģʽ��0Ϊ��Ȧ�ͽ����㣬1Ϊ��Ȧ������㣬2Ϊ��Ȧ����λ��ײ���㣬3Ϊ��Ȧ����λ���ػ���
  cmd[3] =  snF;                        // ���ͬ���˶���־��falseΪ�����ã�trueΪ����
  cmd[4] =  0x6B;                       // У���ֽ�
  
  // ��������
  HAL_UART_Transmit(huart, (uint8_t *)cmd, 5, EMM_UART_TIMEOUT);
}

/**
  * @brief    ǿ���жϲ��˳�����
  * @param    huart �����ھ��
  * @param    addr  �������ַ
  * @retval   ��ַ + ������ + ����״̬ + У���ֽ�
  */
void Emm_V5_Origin_Interrupt(UART_HandleTypeDef* huart, uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // װ������
  cmd[0] =  addr;                       // ��ַ
  cmd[1] =  0x9C;                       // ������
  cmd[2] =  0x48;                       // ������
  cmd[3] =  0x6B;                       // У���ֽ�
  
  // ��������
  HAL_UART_Transmit(huart, (uint8_t *)cmd, 5, EMM_UART_TIMEOUT);
}


/**
 * @brief    ����������ص�����
 * @param    buffer���������ݻ�����
 * @param    len   ���������ݳ���
 * @param    resp  ������������ݽṹ��
 * @retval   0:����ʧ�ܣ�1:�����ɹ�
 */
uint8_t Emm_V5_Parse_Response(uint8_t *buffer, uint8_t len, Emm_V5_Response_t *resp)
{
  uint8_t i;

  // �������
  if (buffer == NULL || resp == NULL || len < 3)
    return 0; // У���������

  // ��սṹ��
  memset(resp, 0, sizeof(Emm_V5_Response_t)); // ��ʼ���ṹ��

  // ����ԭʼ����
  resp->addr = buffer[0]; // �����ַ
  resp->func = buffer[1]; // ���湦����
  resp->data_len = len;   // �������ݳ���
  for (i = 0; i < len && i < 32; i++)
    resp->raw_data[i] = buffer[i]; // ����ԭʼ����

  // ���ݹ������������
  switch (resp->func)
  {
  case 0x1F: // ��ȡ�̼��汾
    if (len >= 4)
    {
      for (i = 0; i < len - 3 && i < 15; i++)
        resp->version[i] = buffer[2 + i]; // ���ư汾��Ϣ
      resp->version[i] = '\0';            // �ַ�������
      resp->valid = 1;                    // ������Ч
    }
    break;

  case 0x20: // ��ȡ����������
    if (len >= 6)
    {
      resp->current = (buffer[2] << 8) | buffer[3]; // ��ȡ����
      resp->voltage = (buffer[4] << 8) | buffer[5]; // ������voltage�ֶδ洢���
      resp->valid = 1;                              // ������Ч
    }
    break;

  case 0x21: // ��ȡPID����
    // PID�������������ʵ��Э���ʽ����
    if (len >= 8)
    {
      resp->valid = 1; // ������Ч
    }
    break;

  case 0x24: // ��ȡ���ߵ�ѹ
    if (len >= 4)
    {
      resp->voltage = (buffer[2] << 8) | buffer[3]; // ��ȡ��ѹ
      resp->valid = 1;                              // ������Ч
    }
    break;

  case 0x27: // ��ȡ�����
    if (len >= 4)
    {
      resp->current = (buffer[2] << 8) | buffer[3]; // ��ȡ����
      resp->valid = 1;                              // ������Ч
    }
    break;

  case 0x31: // ��ȡ������ֵ
    if (len >= 6)
    {
      resp->encoder = (buffer[2] << 24) | (buffer[3] << 16) | (buffer[4] << 8) | buffer[5]; // ��ȡ������ֵ
      resp->valid = 1;                                                                      // ������Ч
    }
    break;

  case 0x33: // ��ȡ���״̬
    if (len >= 3)
    {
      resp->status = buffer[2]; // ��ȡ״̬
      resp->valid = 1;          // ������Ч
    }
    break;

  case 0x35: // ��ȡʵʱת��
    if (len >= 5)
    {
      resp->dir = buffer[2];                      // ����
      resp->speed = (buffer[3] << 8) | buffer[4]; // ת��
      if (resp->dir)
        resp->speed = -resp->speed; // ���ݷ�����������ֵ
      resp->valid = 1;              // ������Ч
    }
    break;

  case 0x36: // ��ȡʵʱλ��
    if (len >= 7)
    {
      resp->dir = buffer[2];                                                                  // ����
      // ������32λλ��ֵ
      uint32_t full_position = (buffer[3] << 24) | (buffer[4] << 16) | (buffer[5] << 8) | buffer[6];
      // ���ڽǶȼ����16λֵ��һȦ�ڵ�λ�ã�
      resp->position = full_position % 65536;
      
      // ���ݷ�����������ֵ
      if (resp->dir)
        resp->position = -resp->position;
      
      resp->valid = 1;                                                                        // ������Ч
    }
    break;

  case 0x37: // ��ȡλ�����
    if (len >= 7)
    {
      int32_t perr = (buffer[2] << 24) | (buffer[3] << 16) | (buffer[4] << 8) | buffer[5]; // ��ȡλ�����
      resp->position = perr;                                                               // ʹ��position�ֶδ洢���ֵ
      resp->valid = 1;                                                                     // ������Ч
    }
    break;

  case 0x39: // ��ȡ�������
    if (len >= 3)
    {
      resp->error = buffer[2]; // ��ȡ�������
      resp->valid = 1;         // ������Ч
    }
    break;

  case 0x3A: // ��ȡ�¶�/ʹ�ܱ�־
    if (len >= 3)
    {
      resp->status = buffer[2]; // ��ȡ״̬��־
      resp->valid = 1;          // ������Ч
    }
    break;

  case 0x3B: // ��ȡ����״̬
    if (len >= 3)
    {
      resp->origin_state = buffer[2]; // ��ȡ����״̬
      resp->valid = 1;                // ������Ч
    }
    break;

  case 0x3D: // ��ȡĿ��λ��
    if (len >= 6)
    {
      resp->target_pos = (buffer[2] << 24) | (buffer[3] << 16) | (buffer[4] << 8) | buffer[5]; // ��ȡĿ��λ��
      resp->valid = 1;                                                                         // ������Ч
    }
    break;

  case 0x3E: // ��ȡĿ���ٶ�
    if (len >= 4)
    {
      resp->target_speed = (buffer[2] << 8) | buffer[3]; // ��ȡĿ���ٶ�
      resp->valid = 1;                                   // ������Ч
    }
    break;

  case 0x3F: // ��ȡ���ٶ�
    if (len >= 3)
    {
      resp->acceleration = buffer[2]; // ��ȡ���ٶ�
      resp->valid = 1;                // ������Ч
    }
    break;

  case 0x40: // ��ȡϸ������
    if (len >= 3)
    {
      resp->subdivision = buffer[2]; // ��ȡϸ������
      resp->valid = 1;               // ������Ч
    }
    break;

  case 0x41: // ��ȡ����ģʽ
    if (len >= 3)
    {
      resp->ctrl_mode = buffer[2]; // ��ȡ����ģʽ
      resp->valid = 1;             // ������Ч
    }
    break;

  case 0x42: // ��ȡ��������
    if (len >= 3)
    {
      resp->protection = buffer[2]; // ��ȡ��������
      resp->valid = 1;              // ������Ч
    }
    break;

  case 0x43: // ��ȡPWMռ�ձ�
    if (len >= 4)
    {
      resp->pwm_duty = (buffer[2] << 8) | buffer[3]; // ��ȡPWMռ�ձ�
      resp->valid = 1;                               // ������Ч
    }
    break;

  case 0x44: // ��ȡ�ջ�״̬
    if (len >= 3)
    {
      resp->closed_loop_state = buffer[2]; // ��ȡ�ջ�״̬
      resp->valid = 1;                     // ������Ч
    }
    break;

  case 0x45: // ��ȡ������У׼״̬
    if (len >= 3)
    {
      resp->encoder_state = buffer[2]; // ��ȡ������У׼״̬
      resp->valid = 1;                 // ������Ч
    }
    break;

  case 0x46: // ��ȡ���ͬ��״̬
    if (len >= 3)
    {
      resp->sync_state = buffer[2]; // ��ȡ���ͬ��״̬
      resp->valid = 1;              // ������Ч
    }
    break;

  case 0x47: // ��ȡ����״̬
    if (len >= 3)
    {
      resp->origin_state = buffer[2]; // ��ȡ����״̬
      resp->valid = 1;                // ������Ч
    }
    break;

  default:
    // ���������룬������ԭʼ����
    resp->valid = 1; // ������Ч(ԭʼ�����ѱ���)
    break;
  }

  return resp->valid; // ���ؽ������
}

