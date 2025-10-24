#ifndef STANDARD_ROBOT_GIMBAL_H
#define STANDARD_ROBOT_GIMBAL_H

#include <stdbool.h>
#include "main.h"
#include "DM_4310.h"
#include "DJI_Motor.h"
#include "pid.h"
#include "DT7.h"
#include "cmsis_os.h"

#define PI	3.14159265358979f

//6020Joint4角度内环
#define M6020_MOTOR_POSITION_PID_KP 6000.0f
#define M6020_MOTOR_POSITION_PID_KI 0
#define M6020_MOTOR_POSITION_PID_KD 0
#define M6020_MOTOR_POSITION_PID_MAX_OUT 1500.0f
#define M6020_MOTOR_POSITION_PID_MAX_IOUT 400.0f
//6020Joint4速度外环
#define M6020_MOTOR_SPEED_PID_KP 12.0f//10
#define M6020_MOTOR_SPEED_PID_KI 0.06f
#define M6020_MOTOR_SPEED_PID_KD 0
#define M6020_MOTOR_SPEED_PID_MAX_OUT 30000.0f
#define M6020_MOTOR_SPEED_PID_MAX_IOUT 10000.0f

typedef enum {
    INIT_MODE = 0,      // 初始化模式
    FORCELESS_MODE,
    REMOTE_MODE,        // 遥控器模式
    AUTO_MODE,          // 自动模式
    NUM_MODES           // 边界值
} gimbal_mode_e;

// 模式处理函数指针
typedef void (*ModeHandler)(void);

// 模式处理函数注册表
typedef struct {
    gimbal_mode_e mode;        // 模式
    ModeHandler handler;    // 该模式对应得处理函数
} ModeHandlerEntry;

/**
 * @brief 定义云台电机状态数据结构体
 */
typedef struct
{
    union
    {
        const motor_measure_t *motor_DJI;                // 指向大疆电机测量数据的常量指针
        const dm_motor_measure_t *motor_DM;              // 指向达妙电机测量数据的常量指针
    } motor_measure;
    float current_angle;    // 当前角度
    float absolute_angle;   // 陀螺仪角度
    float absolute_gyro;    // 陀螺仪角速度
    float target_angle;     // 目标角度
    float error_angle;      // 角度误差
    float bias_angle;       // 电机角度偏差
    float speed_set;        // 目标速度
    float current_speed;    // 当前速度
    float given_current;    // 电流编码值
    float torque;           // 力矩值
    float max_angle;        // 最大值
    float min_angle;        // 最小值
    float middle_angle;     // 中间值 = (最大值 + 最小值)*0.5
    uint8_t motor_id;       // 电机ID
    pid_type_def position_pid;               // 位置PID控制器
    pid_type_def speed_pid;                  // 速度PID控制器
} gimbal_status_t;

/**
 * @brief 定义云台控制数据结构体
 */
typedef struct
{
    const RC_ctrl_t *gimbal_rc_ctrl;   // the point to remote control
    gimbal_status_t pitch; // pitch电机
    gimbal_status_t yaw;  // yaw电机
    gimbal_mode_e mode;       // 云台控制模式
    gimbal_mode_e last_mode;  // 上一次的云台控制模式
    ModeHandlerEntry modeHandlers[NUM_MODES];   // 模式处理函数注册表
    uint8_t registeredModes;                      // 已注册的模式数量
    const fp32 *gimbal_INT_angle_point;
    const fp32 *gimbal_INT_gyro_point;
} gimbal_control_t;

typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 set;
    fp32 get;
    fp32 err;

    fp32 max_out;
    fp32 max_iout;

    fp32 Pout;
    fp32 Iout;
    fp32 Dout;

    fp32 out;
} gimbal_PID_t;

void gimbal_init(gimbal_control_t * init);
void HandleCurrentMode(gimbal_control_t * init);
void gimbal_Mode_Set(gimbal_control_t *mode);
void ChangeMode(gimbal_control_t *mode, gimbal_mode_e newMode);
void gimbal_position_control(gimbal_control_t *gimbal_position);
void gimbal_mode_change_control_transit(gimbal_control_t* gimbal_mode_change);
void gimbal_feedback_update(gimbal_control_t* feedback_update);
float gimbal_angle_limit(float angle,float max,float min);
void gimbal_mode_set(gimbal_control_t* set_mode);
extern gimbal_control_t gimbal_control;


#endif //STANDARD_ROBOT_GIMBAL_H