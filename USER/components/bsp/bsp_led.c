#include "bsp_led.h"
#include "main.h"

extern TIM_HandleTypeDef htim5;
/**
  * @brief          aRGB show
  * @param[in]      aRGB: 0xaaRRGGBB, 'aa' is alpha, 'RR' is red, 'GG' is green, 'BB' is blue
  * @retval         none
  */
/**
  * @brief          显示RGB
  * @param[in]      aRGB:0xaaRRGGBB,'aa' 是透明度,'RR'是红色,'GG'是绿色,'BB'是蓝色
  * @retval         none
  */


void aRGB_led_show(uint32_t aRGB)
{
            static uint8_t alpha;
            static uint16_t red, green, blue;

            // 提取 alpha 和颜色分量
            alpha = (aRGB & 0xFF000000) >> 24;
            red = (aRGB & 0x00FF0000) >> 16;
            green = (aRGB & 0x0000FF00) >> 8;
            blue = (aRGB & 0x000000FF) >> 0;

            // 将 alpha 归一化到 0-1 的范围，并调整颜色分量
            float alpha_scale = (float)alpha / 255.0f;
            red = (uint16_t)(red * alpha_scale * 65535 / 255);
            green = (uint16_t)(green * alpha_scale * 65535 / 255);
            blue = (uint16_t)(blue * alpha_scale * 65535 / 255);

            // 如果是共阳极 LED，需要反转占空比
            // red = 65535 - red;
            // green = 65535 - green;
            // blue = 65535 - blue;

            // 设置 PWM 占空比（确保通道与颜色映射正确）
                    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, red);   // TIM_CHANNEL_1 -> 红色
                    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, green); // TIM_CHANNEL_2 -> 绿色
                    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, blue);  // TIM_CHANNEL_3 -> 蓝色
 }



