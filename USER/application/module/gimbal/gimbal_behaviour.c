#include "gimbal_behaviour.h"
#include "user_lib.h"
#include <usb_callback.h>

void gimbal_behaviour_mode_set(gimbal_control_t *set_mode)
{
    if (set_mode == NULL){
        return;
    }

    //初始化模式判断是否到达中值位置
    if (gimbal_control.mode == INIT_MODE)
    {
        static uint16_t init_time = 0;
        static uint16_t init_stop_time = 0;
        init_time++;

        if ((fabs(set_mode->pitch.relative_angle) < 0.1f))
        {
            if (init_stop_time < 100)
            {
                init_stop_time++;
            }
        }
        else
        {
            if (init_time < 6000)
            {
                init_time++;
            }
        }

        //超过初始化最大时间，或者已经稳定到中值一段时间，退出初始化状态开关打下档，或者掉线
        if (init_time < 6000 && init_stop_time < 100 &&
            !switch_is_down(set_mode->gimbal_rc_ctrl->rc.s[0]) )
        {
            return;
        }
        else
        {
            init_stop_time = 0;
            init_time = 0;
        }
    }

    // 如果收到的数据是控制器模式标志位，则切换到控制器模式
    if(switch_is_up(gimbal_control.gimbal_rc_ctrl->rc.s[0]))
    {
        ChangeMode(set_mode,AUTO_MODE);
    }
    else if(switch_is_mid(gimbal_control.gimbal_rc_ctrl->rc.s[0]))
    {
        ChangeMode(set_mode,REMOTE_MODE);

    }
    else {
        ChangeMode(set_mode,FORCELESS_MODE);
    }

    //判断进入init状态机
    {
        static gimbal_mode_e last_gimbal_behaviour = FORCELESS_MODE;
        if (last_gimbal_behaviour == FORCELESS_MODE && gimbal_control.mode != FORCELESS_MODE)
        {
            ChangeMode(set_mode,INIT_MODE);
        }
        last_gimbal_behaviour = gimbal_control.mode;
    }
}


#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

// 自动模式
void HandleAutoMode() {
#define MIN_YAW_ANGLE -1.5f
#define MAX_YAW_ANGLE 1.5f
#define MIN_PITCH_ANGLE -1.5f
#define MAX_PITCH_ANGLE 1.5f

    static fp32 last_yaw = 0.0f;
    static fp32 last_pitch = 0.0f;

    VisionToGimbal data;
    get_vision_data(&data);

    // 角度限制
    fp32 limited_yaw = data.yaw;
    fp32 limited_pitch = data.pitch;

    if (limited_yaw > MAX_YAW_ANGLE) limited_yaw = MAX_YAW_ANGLE;
    else if (limited_yaw < MIN_YAW_ANGLE) limited_yaw = MIN_YAW_ANGLE;

    if (limited_pitch > MAX_PITCH_ANGLE) limited_pitch = MAX_PITCH_ANGLE;
    else if (limited_pitch < MIN_PITCH_ANGLE) limited_pitch = MIN_PITCH_ANGLE;

    // 低通滤波器 (alpha越小越平滑，但延迟越大)
    #define FILTER_ALPHA 0.3f
    fp32 filtered_yaw = FILTER_ALPHA * limited_yaw + (1.0f - FILTER_ALPHA) * last_yaw;
    fp32 filtered_pitch = FILTER_ALPHA * limited_pitch + (1.0f - FILTER_ALPHA) * last_pitch;

    last_yaw = filtered_yaw;
    last_pitch = filtered_pitch;

    // 设置目标角度
    gimbal_control.yaw.relative_angle_set = filtered_yaw;
    gimbal_control.pitch.relative_angle_set = -filtered_pitch;
}

// 无力模式
void HandleGravityCompensationMode() {
    // 无力模式逻辑
}


// 自定义控制器（遥操作）模式
void HandleRemoteMode()
{
#define MIN_YAW_ANGLE -1.5f
#define MAX_YAW_ANGLE 1.5f
#define MIN_PITCH_ANGLE -1.5f
#define MAX_PITCH_ANGLE 1.5f

    fp32 add_yaw_angle = 0.0f;
    fp32 add_pitch_angle = 0.0f;
    const float yaw_rc_sen = 0.00001f;
    const float pitch_rc_sen = -0.000024f;
    static int16_t yaw_channel = 0, pitch_channel = 0;

    rc_deadband_limit(gimbal_control.gimbal_rc_ctrl->rc.ch[2], yaw_channel, 10);
    rc_deadband_limit(gimbal_control.gimbal_rc_ctrl->rc.ch[3], pitch_channel, 10);

    add_yaw_angle = -(float)yaw_channel * yaw_rc_sen;
    add_pitch_angle = -(float)pitch_channel * pitch_rc_sen;

    // 设置控制策略
    gimbal_control.control_strategy = NormalControlStrategy;

    // Yaw轴角度限制处理（按照gimbal_absolute_angle_limit逻辑）
    static fp32 yaw_bias_angle;
    static fp32 yaw_angle_set;

    // 当前控制误差角度
    yaw_bias_angle = rad_format(gimbal_control.yaw.absolute_angle_set - gimbal_control.yaw.absolute_angle);

    // 云台相对角度 + 误差角度 + 新增角度 如果大于 最大机械角度
    if (gimbal_control.yaw.relative_angle + yaw_bias_angle + add_yaw_angle > MAX_YAW_ANGLE)
    {
        // 如果是往最大机械角度控制方向
        if (add_yaw_angle > 0.0f)
        {
            // 计算出一个最大的添加角度
            add_yaw_angle = MAX_YAW_ANGLE - gimbal_control.yaw.relative_angle - yaw_bias_angle;
        }
    }
    else if (gimbal_control.yaw.relative_angle + yaw_bias_angle + add_yaw_angle < MIN_YAW_ANGLE)
    {
        if (add_yaw_angle < 0.0f)
        {
            add_yaw_angle = MIN_YAW_ANGLE - gimbal_control.yaw.relative_angle - yaw_bias_angle;
        }
    }
    yaw_angle_set = gimbal_control.yaw.absolute_angle_set;
    gimbal_control.yaw.absolute_angle_set = rad_format(yaw_angle_set + add_yaw_angle);

    // Pitch轴角度限制处理（按照gimbal_absolute_angle_limit逻辑）
    static fp32 pitch_bias_angle;
    static fp32 pitch_angle_set;

    // 当前控制误差角度
    pitch_bias_angle = rad_format(gimbal_control.pitch.absolute_angle_set - gimbal_control.pitch.absolute_angle);

    // 云台相对角度 + 误差角度 + 新增角度 如果大于 最大机械角度
    if (gimbal_control.pitch.relative_angle + pitch_bias_angle + add_pitch_angle > MAX_PITCH_ANGLE)
    {
        // 如果是往最大机械角度控制方向
        if (add_pitch_angle > 0.0f)
        {
            // 计算出一个最大的添加角度
            add_pitch_angle = MAX_PITCH_ANGLE - gimbal_control.pitch.relative_angle - pitch_bias_angle;
        }
    }
    else if (gimbal_control.pitch.relative_angle + pitch_bias_angle + add_pitch_angle < MIN_PITCH_ANGLE)
    {
        if (add_pitch_angle < 0.0f)
        {
            add_pitch_angle = MIN_PITCH_ANGLE - gimbal_control.pitch.relative_angle - pitch_bias_angle;
        }
    }
    pitch_angle_set = gimbal_control.pitch.absolute_angle_set;
    gimbal_control.pitch.absolute_angle_set = rad_format(pitch_angle_set + add_pitch_angle);

}


void HandleInitMode()
{
    gimbal_control.yaw.relative_angle_set = 0.0f;
    gimbal_control.pitch.relative_angle_set = 0.0f;
}