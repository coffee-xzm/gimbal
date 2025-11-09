#include "usb_callback.h"
#include <stdio.h>
#include <string.h>
#include "gimbal.h"
#include "CRC8_CRC16.h"
#include "usbd_cdc_if.h"
#include "INS_task.h"

// 外部声明全局变量
extern gimbal_control_t gimbal_control;

static VisionToGimbal latest_vision_data;
static VisionToGimbal vision_data;
static GimbalToVision gimbal_data;

void data_init(void){
    memset(&latest_vision_data, 0, sizeof(VisionToGimbal));
    memset(&vision_data, 0, sizeof(VisionToGimbal));
    memset(&gimbal_data, 0, sizeof(GimbalToVision));
    // vision_data.yaw += 1.5f;
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

  }

void get_vision_data(VisionToGimbal* data_out)
{
    memcpy(data_out, &latest_vision_data, sizeof(VisionToGimbal));
}

void usb_send_gimbal_data(void)
{
    uint8_t send_buffer[sizeof(GimbalToVision)];

    // 清零发送缓冲区
    memset(send_buffer, 0, sizeof(send_buffer));

    // 填充帧头
    uint8_t head[2] = {'S', 'P'};
    memcpy(send_buffer + 0, head, 2);

    // 填充模式信息
    uint8_t mode = (gimbal_control.mode == AUTO_MODE) ? 1 : 0;
    memcpy(send_buffer + 2, &mode, 1);

    // 填充四元数 - 使用固定值测试用
    // fp32 quat_data[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // w, x, y, z
    const fp32* quat_data = get_INS_quat_point();
    memcpy(send_buffer + 3, quat_data, 16);  // 4个float = 16字节  //这个要改成

    // 填充云台状态
    // fp32 yaw_data = gimbal_control.yaw.absolute_angle;
    // fp32 yaw_vel_data = gimbal_control.yaw.motor_measure.motor_DM->velocity;  //萌神，这个是电机速度，记得修改成陀螺仪速度
    // fp32 pitch_data = gimbal_control.pitch.absolute_angle;
    // fp32 pitch_vel_data = gimbal_control.pitch.motor_measure.motor_DJI->Now_Omega;
    //? 减去温飘，就是上位机在odom下期望的位置
    fp32 yaw_data = gimbal_control.yaw.absolute_angle - (gimbal_control.yaw.absolute_angle- gimbal_control.yaw.relative_angle);
    fp32 yaw_vel_data = gimbal_control.yaw.motor_gyro;  //萌神，这个是电机速度，记得修改成陀螺仪速度  已改
    fp32 pitch_data = gimbal_control.pitch.absolute_angle - (gimbal_control.pitch.absolute_angle- gimbal_control.pitch.relative_angle);
    fp32 pitch_vel_data = gimbal_control.pitch.motor_gyro;
    memcpy(send_buffer + 19, &yaw_data, 4);
    memcpy(send_buffer + 23, &yaw_vel_data, 4);
    memcpy(send_buffer + 27, &pitch_data, 4);
    memcpy(send_buffer + 31, &pitch_vel_data, 4);

    // 填充射击信息
    fp32 bullet_speed_data = 15.0f;
    uint16_t bullet_count_data = 0;
    memcpy(send_buffer + 35, &bullet_speed_data, 4);
    memcpy(send_buffer + 39, &bullet_count_data, 2);

    // 计算CRC16校验
    uint32_t crc_length = sizeof(GimbalToVision) - 2; // 减去CRC16本身的2字节
    uint16_t crc_data = get_CRC16_check_sum(send_buffer, crc_length, 0xFFFF);
    memcpy(send_buffer + 41, &crc_data, 2);

    // 发送数据
    CDC_Transmit_FS(send_buffer, sizeof(GimbalToVision));
}