#pragma once
#include <cstdio>
#ifdef __cplusplus
extern "C"{
#endif

typedef struct __attribute__((packed)) GimbalToVision
{
  uint8_t head[2] = {'S', 'P'};
  uint8_t mode;  // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
  float q[4];    // wxyz顺序
  float yaw;
  float yaw_vel;
  float pitch;
  float pitch_vel;
  float bullet_speed;
  uint16_t bullet_count;  // 子弹累计发送次数
  uint16_t crc16;
}GimbalToVision;


typedef struct __attribute__((packed)) VisionToGimbal
{
  uint8_t head[2] = {'S', 'P'};
  uint8_t mode;  // 0: 不控制, 1: 控制云台但不开火，2: 控制云台且开火
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
  uint16_t crc16;
}VisionToGimbal;

void usb_receive_callback(uint8_t* Buf, uint32_t *Len);
void get_latest_vision_data(VisionToGimbal* data_out);

#ifdef __cplusplus
}
#endif