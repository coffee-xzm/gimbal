//
// Created by sykk on 2025/3/20.
//

#ifndef STANDARD_ROBOT_DJI_MOTOR_H
#define STANDARD_ROBOT_DJI_MOTOR_H

#include "main.h"
#include "struct_typedef.h"
#include "math.h"



#define DJI_MOTOR_6020_RXID  0x205  //大疆6020电机反馈ID
#define Encoder_Num_Per_Round 8192  //编码器每圈数

#define RPM_TO_RADPS (2.0f * 3.14159f / 60.0f)  //RPM换算到rad/s

//rm motor data
//电机结构体
typedef struct
{
    uint16_t raw_ecd;   //角度编码值
    int16_t raw_speed_rpm;  //转速
    int16_t raw_given_current;  //给定电流
    uint8_t raw_temperate;  //温度
    int16_t raw_last_ecd;   //上一次的编码器计数
    int32_t raw_ecd_count;  //编码器计数
    float Now_Angle;    //当前角度
    float Now_Omega;    //当前角速度
    float Now_Current;  //当前电流
    float Now_Temperature;  //当前温度
    float Pre_Omega;    //上一次角速度
    uint32_t Pre_Encoder;   //上一次编码器值
    int32_t Total_Encoder;  //总编码器值
    int32_t Total_Round;    //总圈数
} motor_measure_t;

void dji_motor_can_callback(uint32_t can_id, const uint8_t* rx_data);   //大疆电机数据处理回调函数
void DJI_6020_data_Get(uint8_t* data);    //大疆6020电机反馈数据处理
const motor_measure_t* get_motor_measure_point();  //获取指定索引的电机结构体指针
/**
 * 大疆电机控制函数
 * @param target1
 * @param target2
 * @param target3
 * @param target4
 */
void CAN_cmd_DJI_control(int16_t motor1, int16_t motor2, int16_t motor3, int16_t rev);
#endif //STANDARD_ROBOT_DJI_MOTOR_H
