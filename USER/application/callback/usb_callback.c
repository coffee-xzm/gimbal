#include "usb_callback.h"
#include <string.h>
#include "gimbal.h"
#include "CRC8_CRC16.h"
#include "usbd_cdc_if.h"
#include "ins_task.h"
#include "core_cm4.h"

extern gimbal_control_t gimbal_control;

static VisionToGimbal latest_vision_data;
static VisionToGimbal vision_data;
static GimbalToVision gimbal_data;

#define USB_RX_FIFO_SIZE 1024
static uint8_t usb_rx_fifo_buf[USB_RX_FIFO_SIZE];
fifo_s_t usb_rx_fifo;

void usb_rx_init(void)
{
    fifo_s_init(&usb_rx_fifo, usb_rx_fifo_buf, USB_RX_FIFO_SIZE);
    memset(&latest_vision_data, 0, sizeof(VisionToGimbal));
    memset(&vision_data, 0, sizeof(VisionToGimbal));
    memset(&gimbal_data, 0, sizeof(GimbalToVision));
}


void usb_rx_data_parse(void)
{
    static uint8_t state = 0;
    static uint16_t bytes_received = 0;
    static uint8_t protocol_packet[sizeof(VisionToGimbal)];
    const uint16_t data_len = sizeof(VisionToGimbal);

    while (fifo_s_used(&usb_rx_fifo) > 0) {
        uint8_t byte = fifo_s_get(&usb_rx_fifo);

        if (state == 0) {
            if (byte == 'S') {
                protocol_packet[0] = byte;
                bytes_received = 1;
                state = 1;
            }
        } else if (state == 1) {
            if (byte == 'P') {
                protocol_packet[1] = byte;
                bytes_received = 2;
                state = 2;
            } else if (byte == 'S') {
                protocol_packet[0] = byte;
                bytes_received = 1;
            } else {
                bytes_received = 0;
                state = 0;
            }
        } else {
            protocol_packet[bytes_received++] = byte;
            if (bytes_received == data_len) {
                uint16_t received_crc = (protocol_packet[data_len - 1] << 8) | protocol_packet[data_len - 2];
                uint16_t calculated_crc = get_CRC16_check_sum(protocol_packet, data_len - 2, 0xFFFF);
                if (calculated_crc == received_crc) {
                    memcpy(&vision_data, protocol_packet, data_len);
                    memcpy(&latest_vision_data, &vision_data, sizeof(VisionToGimbal));
                }
                state = 0;
                bytes_received = 0;
            }
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

    memset(send_buffer, 0, sizeof(send_buffer));

    uint8_t head[2] = {'S', 'P'};
    memcpy(send_buffer + 0, head, 2);

    uint8_t mode = (gimbal_control.mode == AUTO_MODE) ? 1 : 0;
    memcpy(send_buffer + 2, &mode, 1);

    const fp32* quat_data = get_whx_quat_point();
    memcpy(send_buffer + 3, quat_data, 16);

    fp32 yaw_data = gimbal_control.yaw.absolute_angle;
    fp32 yaw_vel_data = gimbal_control.yaw.motor_gyro;
    fp32 pitch_data = gimbal_control.pitch.absolute_angle;
    fp32 pitch_vel_data = gimbal_control.pitch.motor_gyro;
    memcpy(send_buffer + 19, &yaw_data, 4);
    memcpy(send_buffer + 23, &yaw_vel_data, 4);
    memcpy(send_buffer + 27, &pitch_data, 4);
    memcpy(send_buffer + 31, &pitch_vel_data, 4);

    fp32 bullet_speed_data = 15.0f;
    uint16_t bullet_count_data = 0;
    memcpy(send_buffer + 35, &bullet_speed_data, 4);
    memcpy(send_buffer + 39, &bullet_count_data, 2);

    uint32_t crc_length = sizeof(GimbalToVision) - 2;
    uint16_t crc_data = get_CRC16_check_sum(send_buffer, crc_length, 0xFFFF);
    memcpy(send_buffer + 41, &crc_data, sizeof(uint16_t));

    CDC_Transmit_FS(send_buffer, sizeof(GimbalToVision));
}
