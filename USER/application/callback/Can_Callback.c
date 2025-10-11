#include "Can_Callback.h"
#include "DJI_Motor.h"
#include "DM_4310.h"

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
            //大疆电机
            case Joint4_6020_RXID: //0x205
            {
                dji_motor_can_callback(rx_header.StdId, _rx_data);  //大疆电机回调函数
                break;
            }
            //达妙电机
            case CAN1_4310_YAW_RXID: //0x12
                {
                dm_motor_can_callback(rx_header.StdId, _rx_data);  //达妙电机回调函数
                    break;
                }

            default:
            {
                break;
            }
        }
    }
}