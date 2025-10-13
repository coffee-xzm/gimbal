#ifndef USB_CALLBACK_H
#define USB_CALLBACK_H

#include <stdio.h>
#include <stdint.h> // For uint8_t, uint32_t, etc.
#include <main.h>

// 函数声明
extern uint8_t get_CRC8_check_sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
extern uint16_t get_CRC16_check_sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);


typedef __attribute__((__packed__)) struct
{
    // 帧头和控制信息
    __attribute__((__packed__)) struct
    {
        uint8_t head[2];  // 帧头 "SP"
        uint8_t mode;     // 模式 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
        uint8_t reserved; // 保留字节，用于对齐
    } header;
    
    // 姿态信息
    __attribute__((__packed__)) struct
    {
        float q[4];       // 四元数 wxyz
    } attitude;
    
    // 云台状态
    __attribute__((__packed__)) struct
    {
        float yaw; 
        float yaw_vel;
        float pitch;
        float pitch_vel;
        float bullet_speed;
    }gimbal;
    __attribute__((__packed__)) struct
    {
        uint16_t bullet_count;
    }shooting;
    __attribute__((__packed__)) struct
    {
        uint16_t crc16;  // CRC16校验
    } checksum;
}GimbalToVision;

typedef __attribute__((__packed__)) struct
{
    // 帧头和控制信息
    __attribute__((__packed__)) struct
    {
        uint8_t head[2];  // 帧头 "SP"
        uint8_t mode;     // 模式
        uint8_t reserved; // 保留字节
    } header;
    
    // 目标信息
    __attribute__((__packed__)) struct
    {
        float yaw;        // 目标偏航角
        float yaw_vel;    // 目标偏航角速度
        float yaw_acc;    // 目标偏航角加速度
        float pitch;      // 目标俯仰角
        float pitch_vel;  // 目标俯仰角速度
        float pitch_acc;  // 目标俯仰角加速度
    } target;
    
    // 校验信息
    __attribute__((__packed__)) struct
    {
        uint16_t crc16;  // CRC16校验
    } checksum;
    
} VisionToGimbal;

// 公共函数原型
void usb_receive_callback(uint8_t* Buf, uint32_t *Len);
void get_latest_vision_data(VisionToGimbal* data_out);
void usb_send_gimbal_data(void);

#endif // USB_CALLBACK_H