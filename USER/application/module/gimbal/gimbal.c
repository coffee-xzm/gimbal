#include <string.h>
#include "gimbal.h"
#include "struct_typedef.h"
#include "gimbal_behaviour.h"
#include "can.h"
#include "ins_task.h"
#include "user_lib.h"
#include "arm_math.h"

/**
 * @brief ��ʼ����̨�������ݽṹ��
 */
gimbal_control_t gimbal_control={
        .mode = INIT_MODE,   // ��ǰ��̨����ģʽΪ��ʼģʽ����ʼ��Ϊ��ʼģʽ
        .last_mode = INIT_MODE,  // ��һ�ο���ģʽĬ��Ϊ��ʼģʽ����ʼ��Ϊ��ʼģʽ
        .registeredModes = 0,        // ��ע���ģʽ��������ʼ��Ϊ0
        .modeHandlers = {0},  // ģʽ��������ע�������ʼ��Ϊ0
};

/**
 * @brief ģʽ��������ע��
 * @param mode
 * @param handler
 */
void RegisterModeHandler(gimbal_mode_e mode, ModeHandler handler) {
    if (mode < NUM_MODES) {
        gimbal_control.modeHandlers[gimbal_control.registeredModes].mode = mode;    // ע��ģʽ
        gimbal_control.modeHandlers[gimbal_control.registeredModes].handler = handler;  // ע��ģʽ��Ӧ�Ĵ�������
        gimbal_control.registeredModes++; // ��ע���ģʽ������1
    } else {
    }
}

/**
 * @brief ����ģʽ����ע��
 */
void InitializeModeHandlers()
{
    RegisterModeHandler(INIT_MODE, HandleInitMode); // ��ʼ��ģʽ
    RegisterModeHandler(FORCELESS_MODE,HandleGravityCompensationMode);  // ����ģʽ
    RegisterModeHandler(REMOTE_MODE, HandleRemoteMode); // ң����ģʽ
    RegisterModeHandler(AUTO_MODE,HandleAutoMode);  //�Զ�ģʽ

}

/**
 * @brief ��̨����ģʽ��������
 */
void HandleCurrentMode(gimbal_control_t * move_init) {
    for (uint8_t i = 0; i < move_init->registeredModes; i++) {
        if (move_init->modeHandlers[i].mode == move_init->mode) {
            move_init->modeHandlers[i].handler();  // ���ģʽƥ�䣬���ö�Ӧ�Ĵ�������
            return;
        }
    }
}

/**
 * @brief �л�ģʽ����
 * @param engineer_mode ��е�ۿ������ݽṹ��
 * @param newMode Ŀ��ģʽ
 */
void ChangeMode(gimbal_control_t *move, gimbal_mode_e newMode) {
    if (move == NULL) return;
    // �л�����ģʽ
    move->mode = newMode;
}

/**
 * @brief ��е�ۿ��Ƴ�ʼ��
 */
void gimbal_init(gimbal_control_t * init){

    if (init == NULL){
        return;
    }

    // ��ʼ��ģʽ��������
    InitializeModeHandlers();

    // ==================== Pitch��-GM6020 PID���Ʋ�����ʼ�� ====================
    const static fp32 pitch_speed_pid[3] = {M6020_MOTOR_SPEED_PID_KP, M6020_MOTOR_SPEED_PID_KI, M6020_MOTOR_SPEED_PID_KD};
    PID_init(&init->pitch.speed_pid, PID_POSITION,pitch_speed_pid,M6020_MOTOR_SPEED_PID_MAX_OUT, M6020_MOTOR_SPEED_PID_MAX_IOUT);//6020
    gimbal_PID_init(&init->pitch.gimbal_motor_absolute_angle_pid, PITCH_GYRO_ABSOLUTE_PID_MAX_OUT, PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT, PITCH_GYRO_ABSOLUTE_PID_KP, PITCH_GYRO_ABSOLUTE_PID_KI, PITCH_GYRO_ABSOLUTE_PID_KD);
    gimbal_PID_init(&init->pitch.gimbal_motor_relative_angle_pid, PITCH_ENCODE_RELATIVE_PID_MAX_OUT, PITCH_ENCODE_RELATIVE_PID_MAX_IOUT, PITCH_ENCODE_RELATIVE_PID_KP, PITCH_ENCODE_RELATIVE_PID_KI, PITCH_ENCODE_RELATIVE_PID_KD);

    // ==================== Yaw��-GM6020 PID���Ʋ�����ʼ�� ====================
    const static fp32 yaw_speed_pid[3] = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};
    PID_init(&init->yaw.speed_pid, PID_POSITION, yaw_speed_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);
    gimbal_PID_init(&init->yaw.gimbal_motor_absolute_angle_pid, YAW_GYRO_ABSOLUTE_PID_MAX_OUT, YAW_GYRO_ABSOLUTE_PID_MAX_IOUT, YAW_GYRO_ABSOLUTE_PID_KP, YAW_GYRO_ABSOLUTE_PID_KI, YAW_GYRO_ABSOLUTE_PID_KD);
    gimbal_PID_init(&init->yaw.gimbal_motor_relative_angle_pid, YAW_ENCODE_RELATIVE_PID_MAX_OUT, YAW_ENCODE_RELATIVE_PID_MAX_IOUT, YAW_ENCODE_RELATIVE_PID_KP, YAW_ENCODE_RELATIVE_PID_KI, YAW_ENCODE_RELATIVE_PID_KD);

    // ��ȡYaw�� - GM6020�����������ָ�� (dji_motor[1])
    init->yaw.motor_measure.motor_DJI = get_motor_measure_point(1);
    // ��ȡPitch�� - 6020�����������ָ�� (dji_motor[0])
    init->pitch.motor_measure.motor_DJI = get_motor_measure_point(0);

    //����������ָ���ȡ
    init->gimbal_INT_angle_point = get_whx_angle_point();
    init->gimbal_INT_gyro_point = get_whx_gyro_point();

    //��������
    gimbal_feedback_update(init);

    //��ȡң��������ָ��
    init->gimbal_rc_ctrl = get_i6x_point();

    init->mode = FORCELESS_MODE;
    init->last_mode = FORCELESS_MODE;

    init->yaw.absolute_angle_set = init->yaw.absolute_angle;
    init->yaw.relative_angle_set = init->yaw.relative_angle;
    init->yaw.motor_gyro_set = init->yaw.motor_gyro;

    init->pitch.absolute_angle_set = init->pitch.absolute_angle;
    init->pitch.relative_angle_set = init->pitch.relative_angle;
    init->pitch.motor_gyro_set = init->pitch.motor_gyro;

    init->initial_yaw_motor_angle = init->yaw.motor_measure.motor_DJI->Now_Angle;
    init->initial_pitch_motor_angle = init->pitch.motor_measure.motor_DJI->Now_Angle;

}

/**
 * @brief ģʽ�л�״̬����
 * @param engineer_mode_change �������ݽṹ��ָ��
 */
void gimbal_mode_change_control_transit(gimbal_control_t* mode_change)
{
    if (mode_change == NULL){
        return;
    }
    if (mode_change->last_mode != INIT_MODE && mode_change->mode == INIT_MODE)
    {  // �л�����ʼ��ģʽ
        mode_change->yaw.relative_angle_set = mode_change->yaw.relative_angle;
        mode_change->pitch.relative_angle_set = mode_change->pitch.relative_angle;
    }
    if (mode_change->last_mode != REMOTE_MODE && mode_change->mode == REMOTE_MODE)
    {  // �л���ң��ģʽ

        // mode_change->yaw.absolute_angle_set = mode_change->yaw.absolute_angle;
        // mode_change->pitch.absolute_angle_set = mode_change->pitch.absolute_angle;
        mode_change->yaw.absolute_angle_set = mode_change->yaw.absolute_angle_set;
        mode_change->pitch.absolute_angle_set = mode_change->pitch.absolute_angle_set;
    }
    else if (mode_change->last_mode != AUTO_MODE && mode_change->mode == AUTO_MODE)
    {    // �л����Զ�ģʽ
        //! ����һ�£���û��ͨΪɶ������
        // mode_change->yaw.absolute_angle_set = mode_change->yaw.absolute_angle;
        // mode_change->pitch.absolute_angle_set = mode_change->pitch.absolute_angle;
        mode_change->yaw.absolute_angle_set = mode_change->yaw.absolute_angle_set;
        mode_change->pitch.absolute_angle_set = mode_change->pitch.absolute_angle_set;
    }
    else if (mode_change->last_mode != FORCELESS_MODE && mode_change->mode == FORCELESS_MODE)
    {    // �л�������ģʽ
        mode_change->yaw.absolute_angle_set = mode_change->yaw.absolute_angle_set;
        mode_change->pitch.absolute_angle_set = mode_change->pitch.absolute_angle_set;
    }
    mode_change->last_mode = mode_change->mode;
}


#define PITCH_TURN 1

/**
 * @brief ���ݸ��º���
 * @param feedback_update
 */
void gimbal_feedback_update(gimbal_control_t* feedback_update){
    if (feedback_update == NULL)
    {
        return;
    }
    //��̨���ݸ���
    feedback_update->pitch.absolute_angle = *(feedback_update->gimbal_INT_angle_point + WHX_INS_PITCH_ADDRESS_OFFSET);

#if PITCH_TURN
    // feedback_update->pitch.relative_angle = (-feedback_update->pitch.motor_measure.motor_DJI->Now_Angle+ feedback_update->initial_pitch_motor_angle);
    feedback_update->pitch.relative_angle = -feedback_update->pitch.motor_measure.motor_DJI->Now_Angle;
    #else
    // ���ڰ�װ����GM6020������������ԽǶ���Ҫת��
    feedback_update->pitch.relative_angle = feedback_update->pitch.motor_measure.motor_DJI->Now_Angle;
#endif

    feedback_update->pitch.motor_gyro = *(feedback_update->gimbal_INT_gyro_point + WHX_INS_GYRO_Y_ADDRESS_OFFSET);

    feedback_update->yaw.absolute_angle = *(feedback_update->gimbal_INT_angle_point + WHX_INS_YAW_ADDRESS_OFFSET);

    feedback_update->yaw.relative_angle = feedback_update->yaw.motor_measure.motor_DJI->Now_Angle;
    feedback_update->yaw.motor_gyro = arm_cos_f32(feedback_update->pitch.relative_angle) * (*(feedback_update->gimbal_INT_gyro_point + WHX_INS_GYRO_Z_ADDRESS_OFFSET))
                                                   - arm_sin_f32(feedback_update->pitch.relative_angle) * (*(feedback_update->gimbal_INT_gyro_point + WHX_INS_GYRO_X_ADDRESS_OFFSET));
}


//�Ƕ��޷���������ֹ�����ת
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
  * @brief          ������̨����ģʽ����Ҫ��'gimbal_behaviour_mode_set'�����иı�
  * @param[out]     gimbal_set_mode:"gimbal_control"����ָ��.
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
  * @brief          ��ʼ��"gimbal_control"����������pid��ʼ���� ң����ָ���ʼ������̨���ָ���ʼ���������ǽǶ�ָ���ʼ��
  * @param[out]     gimbal_init:"gimbal_control"����ָ��.
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
    pid->Iout = abs_limit(pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    pid->out = abs_limit(pid->out, pid->max_out);
    return pid->out;
}

/**
  * @brief          gimbal PID clear, clear pid.out, iout.
  * @param[out]     gimbal_pid_clear: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ��̨PID��������pid��out,iout
  * @param[out]     gimbal_pid_clear:"gimbal_control"����ָ��.
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

// ==================== ���Ʋ���ʵ�� ====================

/**
 * @brief �������Ʋ��� - ˫��PID����
 */
/**
 * @brief �������Ʋ��� - �ǶȻ�+�ٶȻ�˫��PID����
 * @param control ��̨�������ݽṹ��ָ��
 * @note ʹ�ýǶȻ�(�⻷)������ٶ��趨ֵ���ٶȻ�(�ڻ�)�������ֵ
 */
void NormalControlStrategy(gimbal_control_t* control) {
    if (control == NULL) return;

    // ==================== Pitch�� GM6020 ˫��PID���� ====================
    // �ǶȻ�PID���㣺�Ƕ���� �� ���ٶ��趨ֵ
    control->pitch.motor_gyro_set = gimbal_PID_calc(&control->pitch.gimbal_motor_absolute_angle_pid,
                                                    control->pitch.absolute_angle,
                                                    control->pitch.absolute_angle_set,
                                                    control->pitch.motor_gyro);

    // �ٶȻ�PID���㣺�ٶ���� �� ����ֵ
    if (control->pitch.motor_measure.motor_DJI != NULL) {
        control->pitch.given_current = PID_calc(&control->pitch.speed_pid,
                                                control->pitch.motor_gyro,
                                                control->pitch.motor_gyro_set);
    } else {
        control->pitch.given_current = 0;
    }

    // ==================== Yaw�� GM6020 ˫��PID���� ====================
    // �ǶȻ�PID���㣺�Ƕ���� �� ���ٶ��趨ֵ
    control->yaw.motor_gyro_set = gimbal_PID_calc(&control->yaw.gimbal_motor_absolute_angle_pid,
                                                  control->yaw.absolute_angle,
                                                  control->yaw.absolute_angle_set,
                                                  control->yaw.motor_gyro);

    // �ٶȻ�PID���㣺�ٶ���� �� ����ֵ
    if (control->yaw.motor_measure.motor_DJI != NULL) {
        control->yaw.given_current = PID_calc(&control->yaw.speed_pid,
                                              control->yaw.motor_gyro,
                                              control->yaw.motor_gyro_set);
    } else {
        control->yaw.given_current = 0;
    }
}

/**
 * @brief ����ģʽ���Ʋ��� - ���0����
 */
void ForcelessControlStrategy(gimbal_control_t* control) {
    if (control == NULL) return;

    // ֱ�����0�������������ת��
    control->yaw.given_current = 0;
    control->pitch.given_current = 0;

}

/**
 * @brief ��ʼ�����Ʋ���
 */
void InitControlStrategy(gimbal_control_t* control) {
    if (control == NULL) return;

    // ʹ��������PID���ƣ���Ŀ��Ƕ���Ϊ0����ʼ��λ�ã�
    NormalControlStrategy(control);
}

/**
 * @brief ��̨λ�ÿ��ƺ���
 * @param control ��̨�������ݽṹ��ָ��
 */
void gimbal_position_control(gimbal_control_t *control){
    if (control == NULL)
        return;

    // ���õ�ǰ���Ʋ���
    if (control->control_strategy != NULL) {
        control->control_strategy(control);
    } else {
        // Ĭ�ϲ��ԣ�����ģʽ
        ForcelessControlStrategy(control);
    }

    // Yaw(pitch)��0x1FF -> 0x205/0x206 �ֱ���ƶ���GM6020
    CAN_cmd_DJI_control((int16_t)(-control->pitch.given_current), (int16_t)(control->yaw.given_current), 0, 0);
}