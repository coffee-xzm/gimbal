#ifndef USB_CALLBACK_H
#define USB_CALLBACK_H

#include <stdio.h>
#include <stdint.h>
#include <main.h>
#include "fifo.h"

extern uint8_t get_CRC8_check_sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
extern uint16_t get_CRC16_check_sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);

typedef struct
{
    uint8_t head[2];
    uint8_t mode;
    float q[4];
    float yaw;
    float yaw_vel;
    float pitch;
    float pitch_vel;
    float bullet_speed;
    uint16_t bullet_count;
    uint16_t crc16;
} __attribute__((__packed__)) GimbalToVision;

typedef struct
{
    uint8_t head[2];
    uint8_t mode;
    float yaw;
    float yaw_vel;
    float yaw_acc;
    float pitch;
    float pitch_vel;
    float pitch_acc;
    uint16_t crc16;
} __attribute__((__packed__)) VisionToGimbal;

extern fifo_s_t usb_rx_fifo;

void usb_rx_init(void);
void usb_rx_data_parse(void);
void get_vision_data(VisionToGimbal* data_out);
void usb_send_gimbal_data(void);

#endif // USB_CALLBACK_H
