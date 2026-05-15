//
// Created by ROG on 2026/3/11.
//
#ifndef I6X_H
#define I6X_H

#include <stdint.h>

#define I6X_FRAME_LENGTH 25u
#define I6X_RX_BUF_NUM   36u

/* ---------------- RC Switch Definition ---------------- */

#define I6X_SW_UP   ((int8_t)1)
#define I6X_SW_MID  ((int8_t)0)
#define I6X_SW_DOWN ((int8_t)-1)

#define i6x_switch_is_down(s) (s == I6X_SW_DOWN)
#define i6x_switch_is_mid(s)  (s == I6X_SW_MID)
#define i6x_switch_is_up(s)   (s == I6X_SW_UP)

/* ---------------- RC通用宏 (兼容上层调用) ---------------- */
#define switch_is_down(s) i6x_switch_is_down(s)
#define switch_is_mid(s)  i6x_switch_is_mid(s)
#define switch_is_up(s)   i6x_switch_is_up(s)

/* ---------------- Data Struct ---------------- */

typedef struct
{
    int16_t ch[6];      // 六个通道
    int8_t  s[4];       // 四个拨杆
    uint8_t failsafe;   // 失控标志
    uint8_t frame_lost; // 丢帧标志

} __attribute__((packed)) i6x_ctrl_t;

/* ---------------- API ---------------- */

void sbus_to_i6x(i6x_ctrl_t *i6x_ctrl,const uint8_t *sbus_data);

/* 初始化遥控器 */
void i6x_remote_init(void);

/* 数据异常检测 */
uint8_t i6x_data_is_error(void);

/* 获取i6x遥控器数据指针（在DT7.c中实现） */
i6x_ctrl_t *get_i6x_point(void);

#endif //I6X_H
