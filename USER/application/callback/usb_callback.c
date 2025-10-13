#include "usb_callback.h"
#include <stdio.h>
#include <string.h>
#include "gimbal.h"
#include "CRC8_CRC16.h"
#include "usbd_cdc_if.h"

// 外部声明全局变量
extern gimbal_control_t gimbal_control;

static VisionToGimbal latest_vision_data;

/**
  * @brief  USB接收数据处理回调函数
  * @param  Buf: 接收到的数据缓冲区指针
  * @param  Len: 接收到的数据长度指针
  * @retval None
  */
void usb_receive_callback(uint8_t* Buf, uint32_t *Len)
{
    taskENTER_CRITICAL();
    if (*Len != sizeof(VisionToGimbal))
    {
        return;
    }

    // 将缓冲区指针强制转换为struct VisionToGimbal结构体指针，方便访问
    const VisionToGimbal* vision_data = (const VisionToGimbal*)Buf;

    // 2. 帧头校验
    if (vision_data->header.head[0] != 'S' || vision_data->header.head[1] != 'P')
    {
        return;
    }

    // 3. CRC16校验
    // 计算数据部分(除最后两个CRC字节)的CRC
    uint16_t crc_calculated = get_CRC16_check_sum(Buf, sizeof(VisionToGimbal) - 2, 0xFFFF);
    if (crc_calculated != vision_data->checksum.crc16)
    {
        return;
    }
    memcpy(&latest_vision_data, Buf, sizeof(VisionToGimbal));
    taskEXIT_CRITICAL();
}

void get_latest_vision_data(VisionToGimbal* data_out)
{
    // 进入临界区，防止在拷贝数据时被USB接收中断打断
    __disable_irq();
    memcpy(data_out, &latest_vision_data, sizeof(VisionToGimbal));
    // 退出临界区
    __enable_irq();
}

void usb_send_gimbal_data(void)
{
    taskENTER_CRITICAL();
    static GimbalToVision gimbal_data_to_send;
    static uint16_t bullet_count_local = 0;
    
    // 清零结构体
    memset(&gimbal_data_to_send, 0, sizeof(GimbalToVision));
    
    // 填充帧头
    gimbal_data_to_send.header.head[0] = 'S';
    gimbal_data_to_send.header.head[1] = 'P';
    gimbal_data_to_send.header.mode = (gimbal_control.mode == AUTO_MODE) ? 1 : 0;
    
    // 填充姿态信息
    float tmp_q[4] = {0.707, 0.0, 0.0, 0.707};
    memcpy(gimbal_data_to_send.attitude.q, tmp_q, sizeof(gimbal_data_to_send.attitude.q));
    
    // 填充云台状态
    gimbal_data_to_send.gimbal.yaw = gimbal_control.yaw.absolute_angle;
    gimbal_data_to_send.gimbal.yaw_vel = gimbal_control.yaw.current_speed;
    gimbal_data_to_send.gimbal.pitch = gimbal_control.pitch.absolute_angle;
    gimbal_data_to_send.gimbal.pitch_vel = gimbal_control.pitch.current_speed;
    
    // 填充射击信息
    gimbal_data_to_send.gimbal.bullet_speed = 15.0f;
    gimbal_data_to_send.shooting.bullet_count = bullet_count_local++;
    
    // 计算CRC16
    uint32_t crc_length = offsetof(GimbalToVision, checksum);
    gimbal_data_to_send.checksum.crc16 = get_CRC16_check_sum(
        (uint8_t*)&gimbal_data_to_send, crc_length, 0xFFFF);
    
    // 发送数据
    CDC_Transmit_FS((uint8_t*)&gimbal_data_to_send, sizeof(GimbalToVision));
    taskEXIT_CRITICAL();
}
