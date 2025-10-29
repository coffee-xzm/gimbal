#include <string.h>
#include "gimbal.h"
#include "struct_typedef.h"
#include "gimbal_behaviour.h"
#include "can.h"
#include "INS_task.h"
#include "user_lib.h"
#include "imu.h"
#include "arm_math.h"

/**
 * @brief 初始化云台控制数据结构体
 */
gimbal_control_t gimbal_control={
        .mode = INIT_MODE,   // 当前云台控制模式为初始模式，初始化为初始模式
        .last_mode = INIT_MODE,  // 上一次控制模式默认为初始模式，初始化为初始模式
        .registeredModes = 0,        // 已注册的模式数量，初始化为0
        .modeHandlers = {0},  // 模式处理函数注册表，初始化为0
};

/**
 * @brief 模式处理函数注册
 * @param mode
 * @param handler
 */
void RegisterModeHandler(gimbal_mode_e mode, ModeHandler handler) {
    if (mode < NUM_MODES) {
        gimbal_control.modeHandlers[gimbal_control.registeredModes].mode = mode;    // 注册模式
        gimbal_control.modeHandlers[gimbal_control.registeredModes].handler = handler;  // 注册模式对应的处理函数
        gimbal_control.registeredModes++; // 已注册的模式数量加1
    } else {
    }
}

/**
 * @brief 控制模式函数注册
 */
void InitializeModeHandlers() {
    RegisterModeHandler(INIT_MODE, HandleInitMode); // 初始化模式
    RegisterModeHandler(FORCELESS_MODE,HandleGravityCompensationMode);  // 无力模式
    RegisterModeHandler(REMOTE_MODE, HandleRemoteMode); // 遥控器模式
    RegisterModeHandler(AUTO_MODE,HandleAutoMode);  //自动模式

}

/**
 * @brief 云台控制模式处理函数
 */
void HandleCurrentMode(gimbal_control_t * move_init) {
    for (uint8_t i = 0; i < move_init->registeredModes; i++) {
        if (move_init->modeHandlers[i].mode == move_init->mode) {
            move_init->modeHandlers[i].handler();  // 如果模式匹配，调用对应的处理函数
            return;
        }
    }
}

/**
 * @brief 切换模式函数
 * @param engineer_mode 机械臂控制数据结构体
 * @param newMode 目标模式
 */
void ChangeMode(gimbal_control_t *move, gimbal_mode_e newMode) {
    if (move == NULL) return;
    // 切换到新模式
    move->mode = newMode;
}

/**
 * @brief 机械臂控制初始化
 */
void gimbal_init(gimbal_control_t * init){

    if (init == NULL){
        return;
    }

    // 初始化时使能大喵电机
    DM_MotorEnable(&hcan1,0x02);
    // 初始化模式处理函数
    InitializeModeHandlers();

    // ==================== Pitch轴-GM6020 PID控制参数初始化 ====================
    const static fp32 pitch_speed_pid[3] = {M6020_MOTOR_SPEED_PID_KP, M6020_MOTOR_SPEED_PID_KI, M6020_MOTOR_SPEED_PID_KD};
    PID_init(&init->pitch.speed_pid, PID_POSITION,pitch_speed_pid,M6020_MOTOR_SPEED_PID_MAX_OUT, M6020_MOTOR_SPEED_PID_MAX_IOUT);//6020
    gimbal_PID_init(&init->pitch.gimbal_motor_relative_angle_pid, PITCH_ENCODE_RELATIVE_PID_MAX_OUT, PITCH_ENCODE_RELATIVE_PID_MAX_IOUT, PITCH_ENCODE_RELATIVE_PID_KP, PITCH_ENCODE_RELATIVE_PID_KI, PITCH_ENCODE_RELATIVE_PID_KD);
    gimbal_PID_init(&init->pitch.gimbal_motor_absolute_angle_pid, PITCH_ENCODE_RELATIVE_PID_MAX_OUT, PITCH_ENCODE_RELATIVE_PID_MAX_IOUT, PITCH_ENCODE_RELATIVE_PID_KP, PITCH_ENCODE_RELATIVE_PID_KI, PITCH_ENCODE_RELATIVE_PID_KD);

    // ==================== Yaw轴-4310电机 PID控制参数初始化 ====================
    const static fp32 yaw_speed_pid[3] = {DM4310_MOTOR_POSITION_PID_KP, DM4310_MOTOR_POSITION_PID_KI, DM4310_MOTOR_POSITION_PID_KD};
    PID_init(&init->yaw.speed_pid, PID_POSITION,yaw_speed_pid,M6020_MOTOR_POSITION_PID_MAX_OUT, M6020_MOTOR_POSITION_PID_MAX_IOUT);//4310
    gimbal_PID_init(&init->yaw.gimbal_motor_absolute_angle_pid, YAW_GYRO_ABSOLUTE_PID_MAX_OUT, YAW_GYRO_ABSOLUTE_PID_MAX_IOUT, YAW_GYRO_ABSOLUTE_PID_KP, YAW_GYRO_ABSOLUTE_PID_KI, YAW_GYRO_ABSOLUTE_PID_KD);
    gimbal_PID_init(&init->yaw.gimbal_motor_relative_angle_pid, YAW_ENCODE_RELATIVE_PID_MAX_OUT, YAW_ENCODE_RELATIVE_PID_MAX_IOUT, YAW_ENCODE_RELATIVE_PID_KP, YAW_ENCODE_RELATIVE_PID_KI, YAW_ENCODE_RELATIVE_PID_KD);

    // 获取Yaw轴 - 4310电机测量数据指针
    init->yaw.motor_measure.motor_DM = get_dm_motor_measure_point(2);
    // 获取Pitch轴 - 6020电机测量数据指针
    init->pitch.motor_measure.motor_DJI = get_motor_measure_point();

    //陀螺仪数据指针获取
    init->gimbal_INT_angle_point = get_INS_angle_point();
    init->gimbal_INT_gyro_point = get_gyro_data_point();

    //更新数据
    gimbal_feedback_update(init);

    //获取遥控器数据指针
    init->gimbal_rc_ctrl = get_remote_control_point();

    init->mode = FORCELESS_MODE;
    init->last_mode = FORCELESS_MODE;

    init->yaw.absolute_angle_set = init->yaw.absolute_angle;
    init->yaw.relative_angle_set = init->yaw.relative_angle;
    init->yaw.motor_gyro_set = init->yaw.motor_gyro;

    init->pitch.absolute_angle_set = init->pitch.absolute_angle;
    init->pitch.relative_angle_set = init->pitch.relative_angle;
    init->pitch.motor_gyro_set = init->pitch.motor_gyro;

}

/**
 * @brief 模式切换状态保存
 * @param engineer_mode_change 控制数据结构体指针
 */
void gimbal_mode_change_control_transit(gimbal_control_t* mode_change)
{
    if (mode_change == NULL){
        return;
    }
    if (mode_change->last_mode != INIT_MODE && mode_change->mode == INIT_MODE)
    {  // 切换到初始化模式
        mode_change->yaw.relative_angle_set = mode_change->yaw.relative_angle;
        mode_change->pitch.relative_angle_set = mode_change->pitch.relative_angle;
    }
    if (mode_change->last_mode != REMOTE_MODE && mode_change->mode == REMOTE_MODE)
    {  // 切换到遥控模式

        mode_change->yaw.absolute_angle_set = mode_change->yaw.absolute_angle;
        mode_change->pitch.absolute_angle_set = mode_change->pitch.absolute_angle;
    }
    else if (mode_change->last_mode != AUTO_MODE && mode_change->mode == AUTO_MODE)
    {    // 切换到自动模式
        mode_change->yaw.absolute_angle_set = mode_change->yaw.absolute_angle_set;
        mode_change->pitch.absolute_angle_set = mode_change->pitch.absolute_angle_set;
    }
    else if (mode_change->last_mode != FORCELESS_MODE && mode_change->mode == FORCELESS_MODE)
    {    // 切换到无力模式
        mode_change->yaw.absolute_angle_set = mode_change->yaw.absolute_angle_set;
        mode_change->pitch.absolute_angle_set = mode_change->pitch.absolute_angle_set;
    }
    mode_change->last_mode = mode_change->mode;
}


/**
 * @brief 数据更新函数
 * @param feedback_update
 */
void gimbal_feedback_update(gimbal_control_t* feedback_update){
    if (feedback_update == NULL)
    {
        return;
    }
    //云台数据更新
    feedback_update->pitch.absolute_angle = *(feedback_update->gimbal_INT_angle_point + INS_PITCH_ADDRESS_OFFSET);

#if PITCH_TURN
    feedback_update->gimbal_pitch_motor.relative_angle = -motor_ecd_to_angle_change(feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,
                                                                                          feedback_update->gimbal_pitch_motor.offset_ecd);
#else
    // 由于安装的是GM6020电机，电机的相对角度需要转换
    feedback_update->pitch.relative_angle = feedback_update->pitch.motor_measure.motor_DJI->Now_Angle;
#endif

    feedback_update->pitch.motor_gyro = *(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Y_ADDRESS_OFFSET);

    feedback_update->yaw.absolute_angle = *(feedback_update->gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET);

#if YAW_TURN
    feedback_update->gimbal_yaw_motor.relative_angle = -motor_ecd_to_angle_change(feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,
                                                                                        feedback_update->gimbal_yaw_motor.offset_ecd);

#else
    feedback_update->yaw.relative_angle = feedback_update->pitch.motor_measure.motor_DM->position;
#endif
    feedback_update->yaw.motor_gyro = arm_cos_f32(feedback_update->pitch.relative_angle) * (*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET))
                                                   - arm_sin_f32(feedback_update->pitch.relative_angle) * (*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET));
}


//角度限幅函数，防止电机堵转
float gimbal_angle_limit(float angle,float max,float min)
{
    float target_angle = angle;

    if(target_angle > max)
    {
        target_angle = max;
    }
    else if (target_angle < min)
    {
        target_angle = min;
    }
    return target_angle;
}
/**
  * @brief          设置云台控制模式，主要在'gimbal_behaviour_mode_set'函数中改变
  * @param[out]     gimbal_set_mode:"gimbal_control"变量指针.
  * @retval         none
  */
void gimbal_mode_set(gimbal_control_t* set_mode)
{
    if (set_mode == NULL){
        return;
    }
    gimbal_behaviour_mode_set(set_mode);
}


/**
  * @brief          "gimbal_control" valiable initialization, include pid initialization, remote control data point initialization, gimbal motors
  *                 data point initialization, and gyro sensor angle point initialization.
  * @param[out]     gimbal_init: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          初始化"gimbal_control"变量，包括pid初始化， 遥控器指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
  * @param[out]     gimbal_init:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
{
    if (pid == NULL)
    {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = maxout;
}

static fp32 gimbal_PID_calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
{
    fp32 err;
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->get = get;
    pid->set = set;

    err = set - get;
    pid->err = rad_format(err);
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
    pid->Dout = pid->kd * error_delta;
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
    return pid->out;
}

/**
  * @brief          gimbal PID clear, clear pid.out, iout.
  * @param[out]     gimbal_pid_clear: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          云台PID清除，清除pid的out,iout
  * @param[out]     gimbal_pid_clear:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_PID_clear(gimbal_PID_t *gimbal_pid_clear)
{
    if (gimbal_pid_clear == NULL)
    {
        return;
    }
    gimbal_pid_clear->err = gimbal_pid_clear->set = gimbal_pid_clear->get = 0.0f;
    gimbal_pid_clear->out = gimbal_pid_clear->Pout = gimbal_pid_clear->Iout = gimbal_pid_clear->Dout = 0.0f;
}

// ==================== 控制策略实现 ====================

/**
 * @brief 正常控制策略 - 双环PID控制
 */
/**
 * @brief 正常控制策略 - 角度环+速度环双环PID控制
 * @param control 云台控制数据结构体指针
 * @note 使用角度环(外环)输出角速度设定值，速度环(内环)输出电流值
 */
void NormalControlStrategy(gimbal_control_t* control) {
    if (control == NULL) return;

    // ==================== Pitch轴 GM6020 双环PID控制 ====================
    // 角度环PID计算：角度误差 → 角速度设定值
    control->pitch.motor_gyro_set = gimbal_PID_calc(&control->pitch.gimbal_motor_absolute_angle_pid,
                                                    control->pitch.absolute_angle,
                                                    control->pitch.absolute_angle_set,
                                                    control->pitch.motor_gyro);

    // 速度环PID计算：速度误差 → 电流值
    if (control->pitch.motor_measure.motor_DJI != NULL) {
        control->pitch.given_current = PID_calc(&control->pitch.speed_pid,
                                                control->pitch.motor_measure.motor_DJI->Now_Omega,
                                                control->pitch.motor_gyro_set);
    } else {
        control->pitch.given_current = 0;
    }

    // ==================== Yaw轴 DM4310 双环PID控制 ====================
    // 角度环PID计算：角度误差 → 角速度设定值
    control->yaw.motor_gyro_set = gimbal_PID_calc(&control->yaw.gimbal_motor_absolute_angle_pid,
                                                  control->yaw.absolute_angle,
                                                  control->yaw.absolute_angle_set,
                                                  control->yaw.motor_gyro);

    // 速度环PID计算：速度误差 → 电流值
    if (control->yaw.motor_measure.motor_DM != NULL) {
        control->yaw.given_current = PID_calc(&control->yaw.speed_pid,
                                              control->yaw.motor_measure.motor_DM->velocity,
                                              control->yaw.motor_gyro_set);
    } else {
        control->yaw.given_current = 0;
    }
}

/**
 * @brief 无力模式控制策略 - 输出0电流
 */
void ForcelessControlStrategy(gimbal_control_t* control) {
    if (control == NULL) return;

    // 直接输出0电流，电机自由转动
    control->yaw.torque = 0;
    control->pitch.given_current = 0;

}

/**
 * @brief 初始化控制策略
 */
void InitControlStrategy(gimbal_control_t* control) {
    if (control == NULL) return;

    // 使用正常的PID控制，但目标角度设为0（初始化位置）
    NormalControlStrategy(control);
}

/**
 * @brief 云台位置控制函数
 * @param control 云台控制数据结构体指针
 */
void gimbal_position_control(gimbal_control_t *control){
    if (control == NULL)
        return;

    // 调用当前控制策略
    if (control->control_strategy != NULL) {
        control->control_strategy(control);
    } else {
        // 默认策略：无力模式
        ForcelessControlStrategy(control);
    }

    static int time = 0;
    time++;
    if(time==1)
    {
        MIT_CtrlMotor(&hcan1, 0x02, 0, 0.0f, 0, 0, control->yaw.torque);
    }
    else if (time==2)
    {
        CAN_cmd_DJI_control((int16_t)control->pitch.given_current,0,0,0);
        time= 0;
    }
}