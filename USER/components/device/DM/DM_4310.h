/*******************************************************************************
 * @file: dm_4310.h
 * @author: Javen
 * @date: 2024年12月30日
 * @brief: DM 4310电机驱动头文件
 * @note: 该文件定义了DM 4310电机的相关数据结构、枚举类型和函数声明。
 *        支持CAN通信解析、电机控制模式切换以及PID控制器配置。
 *
 * @copyright: Copyright (c) 2024
 * @license: MIT
 ******************************************************************************/

#ifndef DM_4310_H_
#define DM_4310_H_

#include "main.h"
#include "struct_typedef.h"
//#include "pid.h"


/**
 * @brief 定义电机控制参数的取值范围
 */
#define P_MIN -12.5		//位置最小值
#define P_MAX 12.5		//位置最大值
#define V_MIN -45			//速度最小值
#define V_MAX 45			//速度最大值
#define KP_MIN 0.0		//Kp最小值
#define KP_MAX 500.0	//Kp最大值
#define KD_MIN 0.0		//Kd最小值
#define KD_MAX 5.0		//Kd最大值
#define T_MIN -18			//转矩最大值
#define T_MAX 18			//转矩最小值

// C板接收电机数据时电机的ID
#define JOINT1_8009_RXID 0xf1
#define JOINT2_8009_RXID 0xf2
#define JOINT3_4310_RXID 0x12

// C板向电机发送时电机的ID
#define JOINT1_8009_TXID 0x01
#define JOINT2_8009_TXID 0x02
#define JOINT3_4310_TXID 0x03
#define DM_SERIAL_MODE_OFFSET 0x02
/**
 * @brief 定义电机控制模式的枚举类型
 */
typedef enum
{
    DM_MOTOR_RAW = 0,       // 电机原始值控制模式
    DM_MOTOR_POSITION,      // 电机位置控制模式
    DM_MOTOR_SPEED          // 电机速度控制模式
} dm_motor_mode_e;

/**
 * @brief 定义电机测量数据结构体
 */
typedef struct {
    uint8_t error;       // 错误码，4 位
    uint8_t ID;          // 电机 ID，4 位
    uint16_t p_int;      // 位置原始数据，16 位
    uint16_t v_int;      // 速度原始数据，12 位
    uint16_t t_int;      // 扭矩原始数据，12 位
    float position;      // 解析后的位置，单位为弧度 (rad)
    float velocity;      // 解析后的速度，单位为弧度每秒 (rad/s)
    float torque;        // 解析后的扭矩，单位为牛顿米 (Nm)
    float T_mos;         // MOSFET 温度，单位为摄氏度 (°C)
    float T_motor;       // 电机线圈温度，单位为摄氏度 (°C)
} dm_motor_measure_t;

/**
 * @brief 定义电机控制数据结构体
 */
typedef struct
{
    const dm_motor_measure_t* motor_measure;  // 指向电机测量数据的常量指针

    dm_motor_mode_e motor_mode;               // 当前电机控制模式
    dm_motor_mode_e last_motor_mode;          // 上一次电机控制模式

    fp32 position_set;                       // 设定位置，单位：弧度 (rad)
    fp32 speed_set;                          // 设定速度，单位：弧度/秒 (rad/s)
    fp32 current_set;                        // 电流设定值
    int16_t given_current;                   // 实际给定电流值
} dm_motor_t;

/**
 * @brief 获取指定索引的motor_4310_measure_t结构体指针。
 *
 * @param i 数组索引，范围为0到2。
 * @return 指向指定索引的motor_4310_measure_t结构体的指针。
 */
const dm_motor_measure_t* get_dm_motor_measure_point(uint8_t i);

/**
 * @brief 达妙电机集体使能
 * @param index
 */
void DM_ALL_MotorEnable(uint8_t index);

/**
 * @brief 使能电机。
 * @param hcan CAN句柄。
 * @param tx_message CAN发送消息结构体。
 * @param id 数据帧的ID。
 */
void DM_MotorEnable(CAN_HandleTypeDef* hcan,uint16_t id);


/**
 * @brief 位置速度模式控制电机。
 *
 * @param hfdcan FDCAN的句柄。
 * @param id 数据帧的ID。
 * @param _pos 位置给定。
 * @param _vel 速度给定。
 */
void DMmotor_PosSpeed_CtrlMotor(CAN_HandleTypeDef* hfdcan, uint16_t id, float _pos, float _vel);


/**
 * @brief 将无符号整数转换为浮点数。
 *
 * @param x_int 要转换的无符号整数。
 * @param x_min 目标浮点数的最小值。
 * @param x_max 目标浮点数的最大值。
 * @param bits 无符号整数的位数。
 * @return 转换后的浮点数。
 */
float DMmotor_UintToFloat(int x_int, float x_min, float x_max, int bits);

/**
 * @brief 将浮点数转换为无符号整数。
 *
 * @param x 要转换的浮点数。
 * @param x_min 浮点数的最小值。
 * @param x_max 浮点数的最大值。
 * @param bits 无符号整数的位数。
 * @return 转换后的无符号整数。
 */
int DMmotor_FloatToUint(float x, float x_min, float x_max, int bits);

/**
 * @brief 解析电机测量数据。
 * @param ptr
 * @param data
 */
void dm_motor_measure_parse(dm_motor_measure_t* ptr, const uint8_t* data);

/**
 * @brief 电机CAN回调函数。
 * @param can_id
 * @param rx_data
 */
void dm_motor_can_callback(uint32_t can_id, const uint8_t* rx_data);

/**
 * @brief 电机控制
 * @param posi1
 * @param vel1
 */
void CAN_cmd_DM_joint_control(float posi1,float vel1);
void MIT_CtrlMotor(CAN_HandleTypeDef* hcan, uint16_t id, float _pos, float _vel,
                   float _KP, float _KD, float _torq);
#endif // DM_4310_H_