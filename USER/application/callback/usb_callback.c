#include "usb_callback.h"
#include <stdio.h>
#include <string.h>
#include "gimbal.h"
#include "CRC8_CRC16.h"
#include "usbd_cdc_if.h"

// 外部声明全局变量
extern gimbal_control_t gimbal_control;

static VisionToGimbal latest_vision_data;
static VisionToGimbal vision_data;
static GimbalToVision gimbal_data;

void data_init(void){
    memset(&latest_vision_data, 0, sizeof(VisionToGimbal));
    memset(&vision_data, 0, sizeof(VisionToGimbal));
    memset(&gimbal_data, 0, sizeof(GimbalToVision));
}

/**
  * @brief  USB接收数据处理回调函数
  * @param  Buf: 接收到的数据缓冲区指针
  * @param  Len: 接收到的数据长度指针
  * @retval None
  */
  void usb_receive_callback(uint8_t* Buf, uint32_t *Len)
  {
      // 检查数据长度
      if (*Len != sizeof(VisionToGimbal)) {
          return;
      }
      
      // 进入临界区，防止数据竞争
      taskENTER_CRITICAL();
      
      // 将接收到的数据复制到本地缓冲区
      memcpy(&vision_data, Buf, sizeof(VisionToGimbal));
      
      // 验证帧头
      if (vision_data.head[0] == 'S' && vision_data.head[1] == 'P') {
          // 计算CRC校验
          uint32_t crc_length = offsetof(VisionToGimbal, crc16);
          uint16_t crc_calculated = get_CRC16_check_sum(
              (uint8_t*)&vision_data, crc_length, 0xFFFF);
          
          // 验证CRC
          if (crc_calculated == vision_data.crc16) {
              // 数据有效，更新最新数据
              memcpy(&latest_vision_data, &vision_data, sizeof(VisionToGimbal));
          }
      }
      
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
    // 清零发送数据
    memset(&gimbal_data, 0, sizeof(GimbalToVision));
    
    // 填充帧头
    gimbal_data.head[0] = 'S';
    gimbal_data.head[1] = 'P';
    
    // 填充模式信息
    gimbal_data.mode = (gimbal_control.mode == AUTO_MODE) ? 1 : 0;
    
    // 填充四元数（这里使用默认值，实际应该从IMU获取）
    gimbal_data.q[0] = 1.0f;  // w
    gimbal_data.q[1] = 0.0f;  // x
    gimbal_data.q[2] = 0.0f;  // y
    gimbal_data.q[3] = 0.0f;  // z
    
    // 填充云台状态
    gimbal_data.yaw = gimbal_control.yaw.absolute_angle;
    gimbal_data.yaw_vel = gimbal_control.yaw.current_speed;
    gimbal_data.pitch = gimbal_control.pitch.absolute_angle;
    gimbal_data.pitch_vel = gimbal_control.pitch.current_speed;
    
    // 填充射击信息（示例值）
    gimbal_data.bullet_speed = 15.0f;
    gimbal_data.bullet_count = 0;  // 可以根据实际情况更新
    
    // 计算CRC16校验
    uint32_t crc_length = offsetof(GimbalToVision, crc16);
    gimbal_data.crc16 = get_CRC16_check_sum(
        (uint8_t*)&gimbal_data, crc_length, 0xFFFF);
    
    // 发送数据
    CDC_Transmit_FS((uint8_t*)&gimbal_data, sizeof(GimbalToVision));
}
