//
// Created by ROG on 2026/3/11.
//
#include "i6x.h"
#include <math.h>

/* 拨杆归一化 */
#define TO_STICK(v) (((v) < 0) - ((v) > 0))

#define MAPPING_ENABLE 1

static i6x_ctrl_t i6x_ctrl_1;

/* -784~783 -> -660~660 */
static int16_t map_to_660(const int16_t val)
{
    if(val >= 0)
        return (int16_t)floorf((660.0f/783.0f)*(float)val + 0.5f);
    else
        return (int16_t)floorf((660.0f/784.0f)*(float)val + 0.5f);
}

/* SBUS解包 */
void sbus_to_i6x(i6x_ctrl_t *i6x_ctrl,const uint8_t *sbus_data)
{
    if(sbus_data[0] != 0x0F || sbus_data[24] != 0x00)
        return;

    /* 解析通道 */
    i6x_ctrl->ch[0] = ((sbus_data[1] | (sbus_data[2] << 8)) & 0x07FF) - 1024;
    i6x_ctrl->ch[1] = (((sbus_data[2] >> 3) | (sbus_data[3] << 5)) & 0x07FF) - 1024;
    i6x_ctrl->ch[2] = (((sbus_data[3] >> 6) | (sbus_data[4] << 2) | (sbus_data[5] << 10)) & 0x07FF) - 1024;
    i6x_ctrl->ch[3] = (((sbus_data[5] >> 1) | (sbus_data[6] << 7)) & 0x07FF) - 1024;
    i6x_ctrl->ch[4] = (((sbus_data[6] >> 4) | (sbus_data[7] << 4)) & 0x07FF) - 1024;
    i6x_ctrl->ch[5] = (((sbus_data[7] >> 7) | (sbus_data[8] << 1) | (sbus_data[9] << 9)) & 0x07FF) - 1024;

    /* 拨杆 */
    i6x_ctrl->s[0] = TO_STICK((((sbus_data[9] >> 2) | (sbus_data[10] << 6)) & 0x07FF) - 1024);
    i6x_ctrl->s[1] = TO_STICK((((sbus_data[10] >> 5) | (sbus_data[11] << 3)) & 0x07FF) - 1024);
    i6x_ctrl->s[2] = TO_STICK(((sbus_data[12] | (sbus_data[13] << 8)) & 0x07FF) - 1024);              //3档
    i6x_ctrl->s[3] = TO_STICK((((sbus_data[13] >> 3) | (sbus_data[14] << 5)) & 0x07FF) - 1024);

#if MAPPING_ENABLE

    for(int i=0;i<6;i++)
        i6x_ctrl->ch[i] = map_to_660(i6x_ctrl->ch[i]);

#endif

    /* 状态位 */

    uint8_t flag = sbus_data[23];

    i6x_ctrl->frame_lost = (flag >> 2) & 0x01;
    i6x_ctrl->failsafe   = (flag >> 3) & 0x01;

    /* failsafe保护 */

    if(i6x_ctrl->failsafe)
    {
        for(int i=0;i<6;i++)
            i6x_ctrl->ch[i] = 0;
    }
}

/* 数据异常检测 */

uint8_t i6x_data_is_error(void)
{
    for(int i=0;i<6;i++)
    {
        if(i6x_ctrl_1.ch[i] > 700 || i6x_ctrl_1.ch[i] < -700)
            return 1;
    }

    return 0;
}
