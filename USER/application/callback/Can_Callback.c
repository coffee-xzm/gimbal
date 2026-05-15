#include "Can_Callback.h"
#include "DJI_Motor.h"

//fifo0 CAN中断
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t _rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, _rx_data);
    //从CAN1中接收  三个达妙电机 一个6020 两个2006
    if(hcan->Instance == CAN1) {
        //标识符比对
        switch (rx_header.StdId) {
            //大疆电机 - Pitch (0x205)
            case CAN1_6020_PITCH_RXID:
            {
                dji_motor_can_callback(rx_header.StdId, _rx_data);  //大疆电机回调函数
                break;
            }
            //大疆电机 - Yaw (0x206)
            case CAN1_6020_YAW_RXID:
            {
                dji_motor_can_callback(rx_header.StdId, _rx_data);  //大疆电机回调函数
                break;
            }

            default:
            {
                break;
            }
        }
    }
}