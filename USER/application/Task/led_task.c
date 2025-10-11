#include <math.h>
#include "led_task.h"
#include "bsp_led.h"
#include "cmsis_os.h"
#include "main.h"

// 正弦波表，360个点对应1秒周期
#define SINE_TABLE_SIZE 360
static float sine_table[SINE_TABLE_SIZE];

// 监控变量
volatile float sine_value = 0.0f;
volatile uint32_t phase_index = 0;

#define RGB_FLOW_COLOR_CHANGE_TIME  1000  // 颜色渐变时间
#define RGB_FLOW_COLOR_LENGHT       6     // 颜色数组长度

// 颜色数组：蓝 -> 绿 -> 红 -> 蓝 -> 绿 -> 红 -> 蓝
uint32_t RGB_flow_color[RGB_FLOW_COLOR_LENGHT + 1] = {
        0xFF0000FF,  // 蓝色
        0xFF00FF00,  // 绿色
        0xFFFF0000,  // 红色
        0xFF0000FF,  // 蓝色
        0xFF00FF00,  // 绿色
        0xFFFF0000,  // 红色
        0xFF0000FF   // 蓝色
};

// 初始化正弦表
void init_sine_table(void)
{
    for (int i = 0; i < SINE_TABLE_SIZE; i++)
    {
        sine_table[i] = sinf(2.0f * M_PI * i / SINE_TABLE_SIZE);
    }
}

/**
  * @brief          led RGB任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void led_Task(void const * argument)
{
    uint16_t i, j;
    uint32_t aRGB;

    // 初始化正弦表
    init_sine_table();

    // 使用内部计数器替代osKernelGetTickCount
    uint32_t time_counter = 0;

    while(1)
    {
        for(i = 0; i < RGB_FLOW_COLOR_LENGHT; i++)
        {
            // 从当前颜色渐变到下一个颜色
            for(j = 0; j < RGB_FLOW_COLOR_CHANGE_TIME; j++)
            {
                // 计算当前渐变颜色
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

                // 组合成ARGB颜色
                aRGB = ((uint32_t)alpha << 24) | ((uint32_t)red << 16) |
                       ((uint32_t)green << 8) | ((uint32_t)blue << 0);

                // 设置RGB灯颜色
                aRGB_led_show(aRGB);

                // 更新正弦波变量
                time_counter++;
                if (time_counter >= 1000) {
                    time_counter = 0; // 1秒周期
                }

                // 计算相位索引
                phase_index = time_counter;

                // 获取正弦值
                uint32_t table_index = (phase_index * SINE_TABLE_SIZE) / 1000;
                sine_value = sine_table[table_index];

                // 延时1ms
                osDelay(1);
            }
        }
    }
}