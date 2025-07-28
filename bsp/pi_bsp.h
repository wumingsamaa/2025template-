#ifndef __PI_BSP_H__
#define __PI_BSP_H__

#include "bsp_system.h"

// �������ͱ�ʶ��
#define RED_LASER_ID 'R'
#define GREEN_LASER_ID 'G'

// �����������ݽṹ
typedef struct {
    char type;    // ��������: 'R'��ʾ��ɫ���⣬'G'��ʾ��ɫ����
    int x;        // X����
    int y;        // Y����
    uint8_t isValid; // ������ָʾ��ǰ�����Ƿ���Ч/�Ѹ���
} LaserCoord_t;

int pi_parse_data(char *buffer);
void pi_proc(void);

extern LaserCoord_t latest_red_laser_coord;
extern LaserCoord_t latest_green_laser_coord;

#endif
