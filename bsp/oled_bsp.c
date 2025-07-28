#include "oled_bsp.h"
#include "oled.h"
// ���� OLED ���Ϊ 128 ���أ�ʹ�� 6x8 ����
// ÿ�� 8 ���ظߣ���� 64/8 = 8 �� (y=0~7) �� 32/8 = 4 �� (y=0~3)
// ÿ�� 6 ���ؿ���� 128/6 = 21 ���ַ� (x=0~20? ��������ܻ�������λ��)
// **ע��:** Oled_Printf �� x, y ������λ��Ҫ�ο� OLED_ShowStr ʵ�֣��������ַ�λ�û�����λ��
// �ĵ��е�ע�� (0-127, 0-3) ��ʾ������ 128x32 ��Ļ������ x ������ַ��� y ����

/**
 * @brief	ʹ������printf�ķ�ʽ��ʾ�ַ�������ʾ6x8��С��ASCII�ַ�
 * @param x  ��ʼ X ���� (����) �� �ַ���λ�� (��Ҫ�� OLED_ShowStr)
 * @param y  ��ʼ Y ���� (����) �� �ַ���λ�� (��Ҫ�� OLED_ShowStr, 0-3 �� 0-7)
 * @param format, ... ��ʽ���ַ���������
 * ���磺Oled_Printf(0, 0, "Data = %d", dat);
**/
int Oled_Printf(uint8_t x, uint8_t y, const char *format, ...)
{
	char buffer[128]; // ��������С������Ҫ����
	va_list arg;
	int len;

	va_start(arg, format);
	len = vsnprintf(buffer, sizeof(buffer), format, arg);
	va_end(arg);

	// ���� OLED_ShowStr ʹ���������� x ���ַ��� y
	// ���������С�� 8 (�߶�)
	OLED_ShowStr(x, y * 8, (uint8_t*)buffer, 8); // �� buffer תΪ uint8_t*
	return len;
}

void oled_proc(void)
{
	
}

