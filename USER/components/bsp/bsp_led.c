#include "bsp_led.h"
#include "main.h"

extern TIM_HandleTypeDef htim5;
/**
  * @brief          aRGB show
  * @param[in]      aRGB: 0xaaRRGGBB, 'aa' is alpha, 'RR' is red, 'GG' is green, 'BB' is blue
  * @retval         none
  */
/**
  * @brief          ��ʾRGB
  * @param[in]      aRGB:0xaaRRGGBB,'aa' ��͸����,'RR'�Ǻ�ɫ,'GG'����ɫ,'BB'����ɫ
  * @retval         none
  */


void aRGB_led_show(uint32_t aRGB)
{
            static uint8_t alpha;
            static uint16_t red, green, blue;

            // ��ȡ alpha ����ɫ����
            alpha = (aRGB & 0xFF000000) >> 24;
            red = (aRGB & 0x00FF0000) >> 16;
            green = (aRGB & 0x0000FF00) >> 8;
            blue = (aRGB & 0x000000FF) >> 0;

            // �� alpha ��һ���� 0-1 �ķ�Χ����������ɫ����
            float alpha_scale = (float)alpha / 255.0f;
            red = (uint16_t)(red * alpha_scale * 65535 / 255);
            green = (uint16_t)(green * alpha_scale * 65535 / 255);
            blue = (uint16_t)(blue * alpha_scale * 65535 / 255);

            // ����ǹ����� LED����Ҫ��תռ�ձ�
            // red = 65535 - red;
            // green = 65535 - green;
            // blue = 65535 - blue;

            // ���� PWM ռ�ձȣ�ȷ��ͨ������ɫӳ����ȷ��
                    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, red);   // TIM_CHANNEL_1 -> ��ɫ
                    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, green); // TIM_CHANNEL_2 -> ��ɫ
                    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, blue);  // TIM_CHANNEL_3 -> ��ɫ
 }



