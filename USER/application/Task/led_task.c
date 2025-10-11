#include <math.h>
#include "led_task.h"
#include "bsp_led.h"
#include "cmsis_os.h"
#include "main.h"

// ���Ҳ���360�����Ӧ1������
#define SINE_TABLE_SIZE 360
static float sine_table[SINE_TABLE_SIZE];

// ��ر���
volatile float sine_value = 0.0f;
volatile uint32_t phase_index = 0;

#define RGB_FLOW_COLOR_CHANGE_TIME  1000  // ��ɫ����ʱ��
#define RGB_FLOW_COLOR_LENGHT       6     // ��ɫ���鳤��

// ��ɫ���飺�� -> �� -> �� -> �� -> �� -> �� -> ��
uint32_t RGB_flow_color[RGB_FLOW_COLOR_LENGHT + 1] = {
        0xFF0000FF,  // ��ɫ
        0xFF00FF00,  // ��ɫ
        0xFFFF0000,  // ��ɫ
        0xFF0000FF,  // ��ɫ
        0xFF00FF00,  // ��ɫ
        0xFFFF0000,  // ��ɫ
        0xFF0000FF   // ��ɫ
};

// ��ʼ�����ұ�
void init_sine_table(void)
{
    for (int i = 0; i < SINE_TABLE_SIZE; i++)
    {
        sine_table[i] = sinf(2.0f * M_PI * i / SINE_TABLE_SIZE);
    }
}

/**
  * @brief          led RGB����
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void led_Task(void const * argument)
{
    uint16_t i, j;
    uint32_t aRGB;

    // ��ʼ�����ұ�
    init_sine_table();

    // ʹ���ڲ����������osKernelGetTickCount
    uint32_t time_counter = 0;

    while(1)
    {
        for(i = 0; i < RGB_FLOW_COLOR_LENGHT; i++)
        {
            // �ӵ�ǰ��ɫ���䵽��һ����ɫ
            for(j = 0; j < RGB_FLOW_COLOR_CHANGE_TIME; j++)
            {
                // ���㵱ǰ������ɫ
                uint8_t alpha = (RGB_flow_color[i] & 0xFF000000) >> 24;
                uint8_t red = ((RGB_flow_color[i] & 0x00FF0000) >> 16) +
                              ((((RGB_flow_color[i + 1] & 0x00FF0000) >> 16) -
                                ((RGB_flow_color[i] & 0x00FF0000) >> 16)) * j) / RGB_FLOW_COLOR_CHANGE_TIME;
                uint8_t green = ((RGB_flow_color[i] & 0x0000FF00) >> 8) +
                                ((((RGB_flow_color[i + 1] & 0x0000FF00) >> 8) -
                                  ((RGB_flow_color[i] & 0x0000FF00) >> 8)) * j) / RGB_FLOW_COLOR_CHANGE_TIME;
                uint8_t blue = ((RGB_flow_color[i] & 0x000000FF) >> 0) +
                               ((((RGB_flow_color[i + 1] & 0x000000FF) >> 0) -
                                 ((RGB_flow_color[i] & 0x000000FF) >> 0)) * j) / RGB_FLOW_COLOR_CHANGE_TIME;

                // ��ϳ�ARGB��ɫ
                aRGB = ((uint32_t)alpha << 24) | ((uint32_t)red << 16) |
                       ((uint32_t)green << 8) | ((uint32_t)blue << 0);

                // ����RGB����ɫ
                aRGB_led_show(aRGB);

                // �������Ҳ�����
                time_counter++;
                if (time_counter >= 1000) {
                    time_counter = 0; // 1������
                }

                // ������λ����
                phase_index = time_counter;

                // ��ȡ����ֵ
                uint32_t table_index = (phase_index * SINE_TABLE_SIZE) / 1000;
                sine_value = sine_table[table_index];

                // ��ʱ1ms
                osDelay(1);
            }
        }
    }
}