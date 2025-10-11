//
// Created by 16844 on 2025/3/20.
//

#ifndef STANDARD_ROBOT_CAN_CALLBACK_H
#define STANDARD_ROBOT_CAN_CALLBACK_H

#include "struct_typedef.h"
#include "main.h"

#define CHASSIS_CAN hcan1


/* CAN send and receive ID */
typedef enum
{
    //CAN通讯
    CAN2_FLAG_AND_PUMP_RX_ID =  0x302,

    //第二块板子的反馈
    CAN2_B2_REACT_TX_ID = 0x306,

    //大疆电机一个CAN前四位STDID
    CAN_DEFAUT_FRONT_STD_ID = 0x200,
    //大疆电机一个CAN后四位STDID
    CAN_DEFAUT_NEXT_STD_ID = 0x1FF,

    //GM6020电流控制前四位STDID
//  CAN_GM6020_FRONT_STD_ID = 0x1FE,

    //CAN1：机械臂joint1 joint2 达妙大8009*2（ID：1 2）机械臂joint3 达妙小4310*（ID：3）

    CAN1_4310_YAW_RXID = 0x12,


    Joint4_6020_RXID = 0x205,
    Joint5_2006_right_RXID = 0x207,
    Joint5_2006_left_RXID = 0x208,

    //CAN1：机械臂joint1 joint2 达妙大8009*2（ID：1 2）机械臂joint3 达妙小4310*（ID：3）
    CAN1_8009_Jpint1_TXID = 0x01,//关节电机发送ID
    CAN1_8009_Jpint2_TXID = 0x02,
    CAN1_4310_Jpint3_TXID = 0x03,

    //CAN2：抬升机构 M3508
    CAN2_3508_Updown_RXID = 0x201,

    //达妙电机标识符偏移
    DM_SERIAL_MODE_OFFSET = 0x100,//串级PID标识符偏移

} can_msg_id_e;


extern uint8_t test_flag;

#endif //STANDARD_ROBOT_CAN_CALLBACK_H