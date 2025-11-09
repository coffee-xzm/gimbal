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

// ==================== Pitch轴-GM6020 PID控制参数初始化 ====================
// 大疆6020电机位置环PID参数
#define M6020_MOTOR_SPEED_PID_KP 1300.0f//10
#define M6020_MOTOR_SPEED_PID_KI 15.0f
#define M6020_MOTOR_SPEED_PID_KD 0
#define M6020_MOTOR_SPEED_PID_MAX_OUT 30000.0f
#define M6020_MOTOR_SPEED_PID_MAX_IOUT 10000.0f

//pitch encode angle close-loop PID params, max out and max iout
//pitch 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
#define PITCH_ENCODE_RELATIVE_PID_KP 10.0f
#define PITCH_ENCODE_RELATIVE_PID_KI 0.00f
#define PITCH_ENCODE_RELATIVE_PID_KD 0.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_OUT 10.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_IOUT 0.0f

#define PITCH_GYRO_ABSOLUTE_PID_KP        10.0f
#define PITCH_GYRO_ABSOLUTE_PID_KI        0.0f
#define PITCH_GYRO_ABSOLUTE_PID_KD        0.3f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT   10.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT  0.0f

// ==================== YAW轴-4310电机 PID控制参数初始化 ====================
#define DM4310_MOTOR_SPEED_PID_KP      1.0f
#define DM4310_MOTOR_SPEED_PID_KI      0.0f
#define DM4310_MOTOR_SPEED_PID_KD      0.0f
#define DM4310_MOTOR_SPEED_PID_MAX_OUT  30000.0f
#define DM4310_MOTOR_SPEED_PID_MAX_IOUT 10000.0f
//yaw gyro angle close-loop PID params, max out and max iout
//yaw 角度环 角度由陀螺仪解算 PID参数以及 PID最大输出，积分输出
#define YAW_GYRO_ABSOLUTE_PID_KP        5.0f
#define YAW_GYRO_ABSOLUTE_PID_KI        0.0f
#define YAW_GYRO_ABSOLUTE_PID_KD        0.3f
#define YAW_GYRO_ABSOLUTE_PID_MAX_OUT   10.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_IOUT  0.0f
//yaw encode angle close-loop PID params, max out and max iout
//yaw 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
#define YAW_ENCODE_RELATIVE_PID_KP        1.0f
#define YAW_ENCODE_RELATIVE_PID_KI        0.0f
#define YAW_ENCODE_RELATIVE_PID_KD        0.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_OUT   10.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT  0.0f


typedef enum {
    INIT_MODE = 0,      // 初始化模式
    FORCELESS_MODE,
    REMOTE_MODE,        // 遥控器模式
    AUTO_MODE,          // 自动模式
    NUM_MODES           // 边界值
} gimbal_mode_e;


// ==================== 前向声明 ====================
typedef struct gimbal_control_s gimbal_control_t;

// ==================== 函数指针类型定义 ====================
// 模式处理函数指针
typedef void (*ModeHandler)(void);

// 控制策略函数指针
typedef void (*ControlStrategyFn)(gimbal_control_t*);

// ==================== 模式处理函数注册表 ====================
typedef struct {
    gimbal_mode_e mode;        // 模式
    ModeHandler handler;       // 该模式对应的处理函数
} ModeHandlerEntry;

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
    fp32 relative_angle;     //rad
    fp32 relative_angle_set; //rad
    fp32 absolute_angle;     //rad
    fp32 absolute_angle_set; //rad
    fp32 motor_gyro;         //rad/s
    fp32 motor_gyro_set;
    fp32 motor_speed;
    float given_current;    // 电流编码值
    float torque;           // 力矩值
    uint8_t motor_id;       // 电机ID

    pid_type_def speed_pid;                  // 速度PID控制器
    gimbal_PID_t gimbal_motor_absolute_angle_pid;
    gimbal_PID_t gimbal_motor_relative_angle_pid;
} gimbal_motor_status_t;

/**
 * @brief 定义云台控制数据结构体
 */
struct gimbal_control_s
{
    // ==================== 输入数据 ====================
    const RC_ctrl_t *gimbal_rc_ctrl;        // 遥控器数据指针
    const fp32 *gimbal_INT_angle_point;     // 陀螺仪角度数据指针
    const fp32 *gimbal_INT_gyro_point;      // 陀螺仪角速度数据指针

    // ==================== 执行机构 ====================
    gimbal_motor_status_t yaw;        // Yaw轴电机状态
    gimbal_motor_status_t pitch;        // pitch轴电机状态
    fp32 initial_yaw_motor_angle;
    fp32 initial_pitch_motor_angle;

    // ==================== 模式管理 ====================
    gimbal_mode_e mode;                     // 当前控制模式
    gimbal_mode_e last_mode;                // 上一次控制模式
    ModeHandlerEntry modeHandlers[NUM_MODES]; // 模式处理函数表
    uint8_t registeredModes;                // 已注册模式数量

    // ==================== 控制策略 ====================
    ControlStrategyFn control_strategy;     // 当前控制策略函数指针

};



void gimbal_init(gimbal_control_t * init);
static void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd);
void HandleCurrentMode(gimbal_control_t * init);
void gimbal_Mode_Set(gimbal_control_t *mode);
void ChangeMode(gimbal_control_t *mode, gimbal_mode_e newMode);
void gimbal_position_control(gimbal_control_t *gimbal_position);
void gimbal_mode_change_control_transit(gimbal_control_t* gimbal_mode_change);
void gimbal_feedback_update(gimbal_control_t* feedback_update);
float gimbal_angle_limit(float angle,float max,float min);
void gimbal_mode_set(gimbal_control_t* set_mode);
extern gimbal_control_t gimbal_control;

// 在gimbal.h中添加
// 控制策略函数声明
void NormalControlStrategy(gimbal_control_t* control);
void ForcelessControlStrategy(gimbal_control_t* control);
void InitControlStrategy(gimbal_control_t* control);
#endif //STANDARD_ROBOT_GIMBAL_H