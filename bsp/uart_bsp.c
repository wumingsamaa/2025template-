#include "uart_bsp.h"

struct rt_ringbuffer ringbuffer_x;
struct rt_ringbuffer ringbuffer_y;
struct rt_ringbuffer ringbuffer_pi;
	
uint8_t ringbuffer_pool_x[64];
uint8_t ringbuffer_pool_y[64];
uint8_t ringbuffer_pool_pi[64];

uint8_t output_buffer_x[64];
uint8_t output_buffer_y[64];
uint8_t output_buffer_pi[64];

static char line_buffer[128]; // �㹻������ĵ������ݣ����� 64 �ֽ� + \n + \0
static int line_buffer_idx = 0; // ��ǰ�л�������д��λ��

int my_printf(UART_HandleTypeDef *huart, const char *format, ...)
{
	char buffer[512];
	int retval;
	va_list local_argv;
	
	va_start(local_argv, format);
	retval = vsnprintf(buffer, sizeof(buffer), format, local_argv);
	va_end(local_argv);
	
	HAL_UART_Transmit(huart,(uint8_t *)buffer, retval, HAL_MAX_DELAY);
	return retval;
}

// ȫ�ֱ������ڴ洢XY����Ƕ�
float x_motor_angle = 0.0f;
float y_motor_angle = 0.0f;
// ����Ƿ񳬳��Ƕȷ�Χ��־
uint8_t x_angle_limit_flag = 0;
uint8_t y_angle_limit_flag = 0;

// ����Ƕ��޷�����־��������������Ƿ�����
uint8_t motor_angle_limit_check_enabled = 0;

// �ο�λ�ñ����ͳ�ʼ����־
uint32_t x_reference_position = 0;
uint32_t y_reference_position = 0;
uint8_t x_reference_initialized = 0;
uint8_t y_reference_initialized = 0;
float x_relative_angle = 0.0f;
float y_relative_angle = 0.0f;

// �ϵ��ʼλ�ô洢
uint32_t x_initial_position = 0;
uint32_t y_initial_position = 0;
uint8_t x_initial_direction = 0;
uint8_t y_initial_direction = 0;
uint8_t initial_position_saved = 0;

// �Ƕ��޷���麯��
void check_motor_angle_limits(void)
{
    // �����޷���鹦������ʱִ��
    if (!motor_angle_limit_check_enabled)
        return;

    // ���X������ԽǶ��Ƿ񳬳�����
    if (x_relative_angle > MOTOR_MAX_ANGLE || x_relative_angle < -MOTOR_MAX_ANGLE)
    {
        if (x_angle_limit_flag == 0)
        {
            my_printf(&huart1, "X������ԽǶȳ�������(��%d��)��ֹͣ�˶�!\r\n", MOTOR_MAX_ANGLE);
            x_angle_limit_flag = 1;
            // ֹͣ���
            Emm_V5_Stop_Now(&MOTOR_X_UART, MOTOR_X_ADDR, MOTOR_SYNC_FLAG);
        }
    }
    else
    {
        x_angle_limit_flag = 0;
    }

    // ���Y������ԽǶ��Ƿ񳬳�����
    if (y_relative_angle > MOTOR_MAX_ANGLE || y_relative_angle < -MOTOR_MAX_ANGLE)
    {
        if (y_angle_limit_flag == 0)
        {
            my_printf(&huart1, "Y������ԽǶȳ�������(��%d��)��ֹͣ�˶�!\r\n", MOTOR_MAX_ANGLE);
            y_angle_limit_flag = 1;
            // ֹͣ���
            Emm_V5_Stop_Now(&MOTOR_Y_UART, MOTOR_Y_ADDR, MOTOR_SYNC_FLAG);
        }
    }
    else
    {
        y_angle_limit_flag = 0;
    }
}

// �����λ��ֵת��Ϊ�Ƕ�
float calc_motor_angle(uint8_t dir, uint32_t position)
{
    float angle;
    // ȷ��λ��ֵ��0-65535��Χ��
    position = position % 65536;

    // ����Ƕ�ֵ
    angle = ((float)position * 360.0f) / 65536.0f;

    // ����Ǹ����򣬽Ƕ�ȡ��
    if (dir)
    {
        angle = -angle;
    }

    return angle;
}

// ������ԽǶȵĺ���
float calc_relative_angle(uint8_t dir, uint32_t current_position, uint32_t reference_position)
{
    // ȷ��λ��ֵ��0-65535��Χ��
    current_position = current_position % 65536;
    reference_position = reference_position % 65536;

    // �������λ�ò�
    int32_t relative_position;
    if (current_position >= reference_position)
    {
        relative_position = current_position - reference_position;
    }
    else
    {
        // �����������
        relative_position = 65536 - reference_position + current_position;
    }

    // ������λ�ô��ڰ�Ȧ����Ϊ�Ƿ�����Ľ϶̾���
    if (relative_position > 32768)
    {
        relative_position = relative_position - 65536;
    }

    // ������ԽǶ�
    float angle = ((float)relative_position * 360.0f) / 65536.0f;

    // ����Ǹ����򣬽Ƕ�ȡ��
    if (dir)
    {
        angle = -angle;
    }

    return angle;
}

// X�������ݴ�����
void parse_x_motor_data(Emm_V5_Response_t *resp)
{
    // ���ݹ����봦��ͬ���͵�����
    switch (resp->func)
    {
    case 0x35: // ��ȡʵʱת��
        my_printf(&huart1, "X������ַ:%d ʵʱת��:%d RPM\r\n", resp->addr, resp->speed);
        break;

    case 0x36: // ��ȡʵʱλ��
        // ����X�������ԽǶ�
        x_motor_angle = calc_motor_angle(resp->dir, resp->position);

        // ��ʼ���ο�λ�û������ԽǶ�
        if (!x_reference_initialized)
        {
            x_reference_position = resp->position;
            x_reference_initialized = 1;
            x_relative_angle = 0.0f;
            my_printf(&huart1, "X�����ο�λ���ѳ�ʼ��: %ld ����\r\n", x_reference_position);
        }
        else
        {
            // ������ԽǶ�
            x_relative_angle = calc_relative_angle(resp->dir, resp->position, x_reference_position);
        }
        
        // �����ʼλ����Ϣ
        if (!initial_position_saved && y_reference_initialized)
        {
            x_initial_position = resp->position;
            x_initial_direction = resp->dir;
            initial_position_saved = 1;
            my_printf(&huart1, "��ʼλ���ѱ���: X=%ld Y=%ld\r\n", x_initial_position, y_initial_position);
        }

        my_printf(&huart1, "X������ַ:%d ʵʱλ��:%ld ���� ���ԽǶ�:%.2f�� ��ԽǶ�:%.2f��\r\n",
                  resp->addr, resp->position, x_motor_angle, x_relative_angle);
        break;

    case 0x1F: // ��ȡ�̼��汾
        my_printf(&huart1, "X������ַ:%d �̼��汾:%s\r\n", resp->addr, resp->version);
        break;

    case 0x24: // ��ȡ���ߵ�ѹ
        my_printf(&huart1, "X������ַ:%d ���ߵ�ѹ:%d V\r\n", resp->addr, resp->voltage);
        break;

    case 0x27: // ��ȡ�����
        my_printf(&huart1, "X������ַ:%d �����:%d mA\r\n", resp->addr, resp->current);
        break;

    case 0x33: // ��ȡ���״̬
        my_printf(&huart1, "X������ַ:%d ״ֵ̬:0x%02X\r\n", resp->addr, resp->status);
        // ����״ֵ̬��һ����������״̬
        if (resp->status & 0x01)
            my_printf(&huart1, "  X������ʹ��\r\n");
        if (resp->status & 0x02)
            my_printf(&huart1, "  X�����ѵ�λ\r\n");
        if (resp->status & 0x04)
            my_printf(&huart1, "  X������ת����\r\n");
        break;

    case 0x3B: // ��ȡ����״̬
        my_printf(&huart1, "X������ַ:%d ����״̬:0x%02X\r\n", resp->addr, resp->origin_state);
        // ���ݻ���״ֵ̬��һ������
        if (resp->origin_state == 0)
            my_printf(&huart1, "  X��δ���ڻ���״̬\r\n");
        else if (resp->origin_state == 1)
            my_printf(&huart1, "  X�����ڻ���\r\n");
        else if (resp->origin_state == 2)
            my_printf(&huart1, "  X��������\r\n");
        else if (resp->origin_state == 3)
            my_printf(&huart1, "  X�����ʧ��\r\n");
        break;

    default:
        // ���������룬��ֱ�ӷ���resp�еĽṹ���Ա��ȡ�����������
        my_printf(&huart1, "X������ַ:%d ������:0x%02X �ѽ���δ����\r\n", resp->addr, resp->func);
        break;
    }
}

// Y�������ݴ�����
void parse_y_motor_data(Emm_V5_Response_t *resp)
{
    // ���ݹ����봦��ͬ���͵�����
    switch (resp->func)
    {
    case 0x35: // ��ȡʵʱת��
        my_printf(&huart1, "Y������ַ:%d ʵʱת��:%d RPM\r\n", resp->addr, resp->speed);
        break;

    case 0x36: // ��ȡʵʱλ��
        // ����Y�������ԽǶ�
        y_motor_angle = calc_motor_angle(resp->dir, resp->position);

        // ��ʼ���ο�λ�û������ԽǶ�
        if (!y_reference_initialized)
        {
            y_reference_position = resp->position;
            y_reference_initialized = 1;
            y_relative_angle = 0.0f;
            my_printf(&huart1, "Y�����ο�λ���ѳ�ʼ��: %ld ����\r\n", y_reference_position);
        }
        else
        {
            // ������ԽǶ�
            y_relative_angle = calc_relative_angle(resp->dir, resp->position, y_reference_position);
        }
        
        // �����ʼλ����Ϣ
        if (!initial_position_saved && x_reference_initialized)
        {
            y_initial_position = resp->position;
            y_initial_direction = resp->dir;
            initial_position_saved = 1;
            my_printf(&huart1, "��ʼλ���ѱ���: X=%ld Y=%ld\r\n", x_initial_position, y_initial_position);
        }

        my_printf(&huart1, "Y������ַ:%d ʵʱλ��:%ld ���� ���ԽǶ�:%.2f�� ��ԽǶ�:%.2f��\r\n",
                  resp->addr, resp->position, y_motor_angle, y_relative_angle);
        break;

    case 0x1F: // ��ȡ�̼��汾
        my_printf(&huart1, "Y������ַ:%d �̼��汾:%s\r\n", resp->addr, resp->version);
        break;

    case 0x24: // ��ȡ���ߵ�ѹ
        my_printf(&huart1, "Y������ַ:%d ���ߵ�ѹ:%d V\r\n", resp->addr, resp->voltage);
        break;

    case 0x27: // ��ȡ�����
        my_printf(&huart1, "Y������ַ:%d �����:%d mA\r\n", resp->addr, resp->current);
        break;

    case 0x33: // ��ȡ���״̬
        my_printf(&huart1, "Y������ַ:%d ״ֵ̬:0x%02X\r\n", resp->addr, resp->status);
        // ����״ֵ̬��һ����������״̬
        if (resp->status & 0x01)
            my_printf(&huart1, "  Y������ʹ��\r\n");
        if (resp->status & 0x02)
            my_printf(&huart1, "  Y�����ѵ�λ\r\n");
        if (resp->status & 0x04)
            my_printf(&huart1, "  Y������ת����\r\n");
        break;

    case 0x3B: // ��ȡ����״̬
        my_printf(&huart1, "Y������ַ:%d ����״̬:0x%02X\r\n", resp->addr, resp->origin_state);
        // ���ݻ���״ֵ̬��һ������
        if (resp->origin_state == 0)
            my_printf(&huart1, "  Y��δ���ڻ���״̬\r\n");
        else if (resp->origin_state == 1)
            my_printf(&huart1, "  Y�����ڻ���\r\n");
        else if (resp->origin_state == 2)
            my_printf(&huart1, "  Y��������\r\n");
        else if (resp->origin_state == 3)
            my_printf(&huart1, "  Y�����ʧ��\r\n");
        break;

    default:
        // ���������룬��ֱ�ӷ���resp�еĽṹ���Ա��ȡ�����������
        my_printf(&huart1, "Y������ַ:%d ������:0x%02X �ѽ���δ����\r\n", resp->addr, resp->func);
        break;
    }
}



// ����λָ��õ���ص���ʼλ��
void process_reset_command(void)
{
    // ֻ�е���ʼλ���ѱ���ʱ��ִ��
    if (initial_position_saved)
    {
        my_printf(&huart1, "���ڸ�λ�������ʼλ��...\r\n");
        // ʹ�þ���λ��ģʽ�ص���ʼλ��
        // X����
        Emm_V5_Pos_Control(&MOTOR_X_UART, MOTOR_X_ADDR, x_initial_direction, 
                           MOTOR_MAX_SPEED/2, MOTOR_ACCEL, x_initial_position, 
                           true, MOTOR_SYNC_FLAG);
        
        // Y����
        Emm_V5_Pos_Control(&MOTOR_Y_UART, MOTOR_Y_ADDR, y_initial_direction, 
                           MOTOR_MAX_SPEED/2, MOTOR_ACCEL, y_initial_position, 
                           true, MOTOR_SYNC_FLAG);
    }
    else
    {
        my_printf(&huart1, "����δ�����ʼλ�ã��޷���λ\r\n");
    }
}

// �����ʼλ����Ϣ
void save_initial_position(void)
{
    // �ϵ�ʱ��ȡ��ǰλ�ò�����Ϊ��ʼλ��
    if (!initial_position_saved)
    {
        // ��ȡX��λ��
        Emm_V5_Read_Sys_Params(&MOTOR_X_UART, MOTOR_X_ADDR, S_CPOS);
        // ��ȡY��λ��
        Emm_V5_Read_Sys_Params(&MOTOR_Y_UART, MOTOR_Y_ADDR, S_CPOS);
        
        // ע�⣺λ����Ϣ��ͨ�������жϽ��ղ���parse_x_motor_data��parse_y_motor_data����
        // ���λ���ѱ�������Чλ�����ݱ�����������
        my_printf(&huart1, "���ڶ�ȡ��ʼλ��...\r\n");
    }
}

// ָ�����
void process_command(const char* cmd, uint16_t len)
{
    // ����resetָ��
    if (strncmp(cmd, "reset", 5) == 0)
    {
        // ����λָ��
        process_reset_command();
    }
    // set(x,y)ָ�� - ����Ŀ���
    else if (strncmp(cmd, "set(", 6) == 0)
    {
        int target_x, target_y;
        // ���Խ�������ֵ
        if (sscanf(cmd, "set(%d,%d)", &target_x, &target_y) == 2)
        {
            // ����PIDĿ��ֵ
        }
        else
        {
            my_printf(&huart1, "setָ���ʽ����ӦΪ set(x,y)\r\n");
        }
    }
}

void uart_proc(void)
{
	uint16_t length_x, length_y, length_pi;
	Emm_V5_Response_t resp_x, resp_y; // ΪX���Y��ֱ𴴽���������Ӧ�ṹ��
	// ����X��������
	length_x = rt_ringbuffer_data_len(&ringbuffer_x);
//	my_printf(&huart1, "abc%d\n",length_x);
	if (length_x > 0)
	{
		rt_ringbuffer_get(&ringbuffer_x, output_buffer_x, length_x);
		output_buffer_x[length_x] = '\0';
//		my_printf(&huart1, "%d\n",length_x);
		// ����X������
		if (Emm_V5_Parse_Response(output_buffer_x, length_x, &resp_x))
		{
			parse_x_motor_data(&resp_x);
			my_printf(&huart1,"id:%d",resp_x.addr);
		}
		else
		{
			my_printf(&huart1, "X�����ݽ���ʧ��!\r\n");
		}

		memset(output_buffer_x, 0, length_x);
	}

	// ����Y��������
	length_y = rt_ringbuffer_data_len(&ringbuffer_y);
	if (length_y > 0)
	{
		rt_ringbuffer_get(&ringbuffer_y, output_buffer_y, length_y);
		output_buffer_y[length_y] = '\0';

		// ����Y������
		if (Emm_V5_Parse_Response(output_buffer_y, length_y, &resp_y))
		{
			parse_y_motor_data(&resp_y);
		}
		else
		{
			my_printf(&huart1, "Y�����ݽ���ʧ��!\r\n");
		}

		memset(output_buffer_y, 0, length_y);
	}

	// �����ݴ�����ɺ�ִ���޷����
//	check_motor_angle_limits();
	
	// ������ݮ������
	length_pi = rt_ringbuffer_data_len(&ringbuffer_pi);
//	my_printf(&huart1,"length:%d\n",length_pi);
	if(length_pi > 0)
	{
		
		rt_ringbuffer_get(&ringbuffer_pi, output_buffer_pi, length_pi);
        // ���ֽڱ�����ringbuffer��ȡ��������
        for (int i = 0; i < length_pi; i++)
        {
            char current_char = output_buffer_pi[i];
            
            // ����ǰ�ַ���ӵ��л�������ͬʱ����Ƿ�����
            // ��ȥ 1 ��Ϊ�˸��ַ���ĩβ�Ŀ���ֹ�������ռ�
            if (line_buffer_idx < sizeof(line_buffer) - 1)
            {
                line_buffer[line_buffer_idx++] = current_char;
            }
            else
            {
                // ����л��������ˣ�����û���յ����з���˵����ǰ��̫�����߸�ʽ����
                // ��ʱ������ջ����������ش��󣬻��߽�������������
                my_printf(&huart1, "Error: Line buffer overflow without newline. Discarding line.\r\n");
                line_buffer_idx = 0; // ��ջ�����������Ӱ���������
                continue; // �������ַ����ȴ���һ�����ܵ�֡ͷ
            }
            
            // ����յ����з� ('\n')����ʾһ��������֡����
            // ��������յ��س��� ('\r')����ĳЩϵͳ��������Ҳ��ʾ�н����������� \n ��ϳ� \r\n
            // ����������Ҫ��� \n
            if (current_char == '\n')
            {
                line_buffer[line_buffer_idx] = '\0'; // ����ĩ��ӿ���ֹ��
                
                // my_printf(&huart1, "Processing received line: '%s'\r\n", line_buffer); // ���Դ�ӡÿһ��
                
                // �������������ݴ��ݸ� pi_parse_data ���н���
                int result = pi_parse_data(line_buffer);
                
                if (result != 0) 
				{
                    my_printf(&huart1, "pi_parse_data returned error %d for line: '%s'\r\n", result, line_buffer);
                }
                
                line_buffer_idx = 0; // �����л�����������׼��������һ������
            }
        }
        
        // ����Ҫ memset(output_buffer_pi, 0, length_pi);
        // ��Ϊ�����Ѿ��� ringbuffer ��ȡ��������output_buffer_pi ֻ��һ����ʱ������
    }

}



