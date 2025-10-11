#include "usb_callback.hpp"
#include <cstdio>
#include <cstring>
#include <gimbal.h>
#include <CRC8_CRC16.h>


static VisionToGimbal latest_vision_data;

/**
  * @brief  USB接收数据处理回调函数
  * @param  Buf: 接收到的数据缓冲区指针
  * @param  Len: 接收到的数据长度指针
  * @retval None
  */
void usb_receive_callback(uint8_t* Buf, uint32_t *Len)
{
    if (*Len != sizeof(VisionToGimbal))
    {
        return;
    }

    // 将缓冲区指针强制转换为VisionToGimbal结构体指针，方便访问
    const VisionToGimbal* vision_data = (const VisionToGimbal*)Buf;

    // 2. 帧头校验
    if (vision_data->head[0] != 'S' || vision_data->head[1] != 'P')
    {
        return;
    }

    // 3. CRC16校验
    // 计算数据部分(除最后两个CRC字节)的CRC
    uint16_t crc_calculated = get_CRC16_check_sum(Buf, sizeof(VisionToGimbal) - 2);
    if (crc_calculated != vision_data->crc16)
    {
        return;
    }
    memcpy(&latest_vision_data, Buf, sizeof(VisionToGimbal));
    // // 4. 所有校验通过，更新云台控制数据
    // // 直接使用接收到的数据更新目标角度
    // gimbal_control.vcom_ctrl.pitch_vcom = vision_data->pitch;
    // gimbal_control.vcom_ctrl.yaw_vcom = vision_data->yaw;
    
    // // 更新收到数据的时间戳，用于模式切换判断
    // gimbal_control.vcom_ctrl.last_time = HAL_GetTick();
}

void get_latest_vision_data(VisionToGimbal* data_out)
{
    // 进入临界区，防止在拷贝数据时被USB接收中断打断
    __disable_irq();
    memcpy(data_out, &latest_vision_data, sizeof(VisionToGimbal));
    // 退出临界区
    __enable_irq();
}
