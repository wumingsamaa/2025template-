#include "pi_bsp.h"

// Ĭ��ֵ��Ϊ��Ч״̬��X, Y Ϊ 0
LaserCoord_t latest_red_laser_coord = {RED_LASER_ID, 0, 0, 0};
LaserCoord_t latest_green_laser_coord = {GREEN_LASER_ID, 0, 0, 0};

// MaixCam ���ݽ���������������ʽ��red:(x,y)\n �� gre:(x,y)\n
// �����������ֱ�Ӹ���ȫ�ֱ��� latest_red_laser_coord �� latest_green_laser_coord
int pi_parse_data(char *buffer)
{
    if (!buffer)
        return -1; // ��ָ����

    int parsed_x, parsed_y; // ��ʱ�������ڴ洢��������X,Y����
    int parsed_count;

    // ����ƥ�� "red:(x,y)" ��ʽ
    if (strncmp(buffer, "red:", 4) == 0)
    {
        parsed_count = sscanf(buffer, "red:(%d,%d)", &parsed_x, &parsed_y);
        if (parsed_count != 2) // ���������X��Y����ֵ
            return -2; // ����ʧ��

        // �����ɹ�������ȫ�ֺ�ɫ��������
        latest_red_laser_coord.x = parsed_x;
        latest_red_laser_coord.y = parsed_y;
        latest_red_laser_coord.isValid = 1; // �������Ϊ��Ч

        // ��ӡ������Ϣ
		my_printf(&huart1, "Parsed RED: X=%d, Y=%d\r\n", latest_red_laser_coord.x, latest_red_laser_coord.y);
    }
    // ����ƥ�� "gre:(x,y)" ��ʽ
    else if (strncmp(buffer, "gre:", 4) == 0)
    {
        parsed_count = sscanf(buffer, "gre:(%d,%d)", &parsed_x, &parsed_y);
        if (parsed_count != 2) // ���������X��Y����ֵ
            return -2; // ����ʧ��

        // �����ɹ�������ȫ����ɫ��������
        latest_green_laser_coord.x = parsed_x;
        latest_green_laser_coord.y = parsed_y;
        latest_green_laser_coord.isValid = 1; // �������Ϊ��Ч

        // ��ӡ������Ϣ
		my_printf(&huart1, "Parsed GRE: X=%d, Y=%d\r\n", latest_green_laser_coord.x, latest_green_laser_coord.y);
    }
    else
    {
        // �Ȳ��� "red:" Ҳ���� "gre:" ��ͷ����Ϊ��δ֪��ʽ����Ч����
        return -3; // δ֪����Ч��ʽ
    }

    return 0; // �ɹ�
}



void pi_proc(void)
{
	float pos_out_x,pos_out_y=0;


		pos_out_x = pid_calc(&pid_x,latest_green_laser_coord.x, latest_red_laser_coord.x, 0);
		pos_out_y = pid_calc(&pid_y,latest_green_laser_coord.y, latest_red_laser_coord.y, 0);
		Step_Motor_Set_Speed_my(-pos_out_x,pos_out_y);

	
}

