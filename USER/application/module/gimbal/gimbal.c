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
 * @brief ��ʼ����̨�������ݽṹ��
 */
gimbal_control_t gimbal_control={
        .mode = INIT_MODE,   // ��ǰ��̨����ģʽΪ��ʼģʽ����ʼ��Ϊ��ʼģʽ
        .last_mode = INIT_MODE,  // ��һ�ο���ģʽĬ��Ϊ��ʼģʽ����ʼ��Ϊ��ʼģʽ
        .registeredModes = 0,        // ��ע���ģʽ��������ʼ��Ϊ0
        .modeHandlers = {0},  // ģʽ������ע�����ʼ��Ϊ0
};

/**
 * @brief ģʽ������ע��
 * @param mode
 * @param handler
 */
void RegisterModeHandler(gimbal_mode_e mode, ModeHandler handler) {
    if (mode < NUM_MODES) {
        gimbal_control.modeHandlers[gimbal_control.registeredModes].mode = mode;    // ע��ģʽ
        gimbal_control.modeHandlers[gimbal_control.registeredModes].handler = handler;  // ע��ģʽ��Ӧ�Ĵ�����
        gimbal_control.registeredModes++; // ��ע���ģʽ������1
    } else {
    }
}

/**
 * @brief ����ģʽ����ע��
 */
void InitializeModeHandlers() {
    RegisterModeHandler(INIT_MODE, HandleInitMode); // ��ʼ��ģʽ
    RegisterModeHandler(FORCELESS_MODE,HandleGravityCompensationMode);  // ����ģʽ
    RegisterModeHandler(REMOTE_MODE, HandleRemoteMode); // ң����ģʽ
    RegisterModeHandler(AUTO_MODE,HandleAutoMode);  //�Զ�ģʽ

}

/**
 * @brief ��̨����ģʽ������
 */
void HandleCurrentMode(gimbal_control_t * move_init) {
    for (uint8_t i = 0; i < move_init->registeredModes; i++) {
        if (move_init->modeHandlers[i].mode == move_init->mode) {
            move_init->modeHandlers[i].handler();  // ���ģʽƥ�䣬���ö�Ӧ�Ĵ�����
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

    DM_MotorEnable(&hcan1,0x02);
    InitializeModeHandlers();  // ��ʼ��ģʽ������

    //Pitch��-GM6020��ѹ����PID
    const static fp32 speed_pid[3] = {M6020_MOTOR_SPEED_PID_KP, M6020_MOTOR_SPEED_PID_KI, M6020_MOTOR_SPEED_PID_KD};
    const static fp32 posi_pid[3] = {M6020_MOTOR_POSITION_PID_KP, M6020_MOTOR_POSITION_PID_KI, M6020_MOTOR_POSITION_PID_KD};

    //6020������Ʋ�����ʼ��
    PID_init(&init->pitch.speed_pid, PID_POSITION,speed_pid,M6020_MOTOR_SPEED_PID_MAX_OUT, M6020_MOTOR_SPEED_PID_MAX_IOUT);//6020
    PID_init(&init->pitch.position_pid, PID_POSITION,posi_pid,M6020_MOTOR_POSITION_PID_MAX_OUT, M6020_MOTOR_POSITION_PID_MAX_IOUT);//6020

    // ��ȡYaw��-4310�����������ָ��
    init->yaw.motor_measure.motor_DM = get_dm_motor_measure_point(2);
    init->pitch.motor_measure.motor_DJI = get_motor_measure_point();              // ��ȡ3508�����������ָ��
    //����������ָ���ȡ
    init->gimbal_INT_angle_point = get_INS_angle_point();
    init->gimbal_INT_gyro_point = get_gyro_data_point();

    //��������
    gimbal_feedback_update(init);
    // ͬ���Ƕ�
    init->yaw.target_angle =  init->yaw.current_angle;
    init->pitch.target_angle =  init->pitch.current_angle;
    // ͬ���ٶ�
    init->yaw.speed_set = init->yaw.current_speed;
    init->pitch.speed_set = init->pitch.current_speed;

    //��ȡң��������ָ��
    init->gimbal_rc_ctrl = get_remote_control_point();

    init->mode = FORCELESS_MODE;
    init->last_mode = FORCELESS_MODE;
}



/**
 * @brief ģʽ�л�״̬����
 * @param engineer_mode_change �������ݽṹ��ָ��
 */
void gimbal_mode_change_control_transit(gimbal_control_t* engineer_mode_change)
{
    if (engineer_mode_change == NULL){
        return;
    }
    if (engineer_mode_change->last_mode != INIT_MODE && engineer_mode_change->mode == INIT_MODE){  // �л���ң��ģʽ

        engineer_mode_change->yaw.target_angle = engineer_mode_change->yaw.current_angle;
        engineer_mode_change->pitch.target_angle = engineer_mode_change->pitch.current_angle;
    }
    if (engineer_mode_change->last_mode != REMOTE_MODE && engineer_mode_change->mode == REMOTE_MODE){  // �л���ң��ģʽ

        engineer_mode_change->yaw.target_angle = engineer_mode_change->yaw.current_angle;
        engineer_mode_change->pitch.target_angle = engineer_mode_change->pitch.current_angle;
    }
    else if (engineer_mode_change->last_mode != AUTO_MODE && engineer_mode_change->mode == AUTO_MODE){    // �л����Զ�ģʽ
        engineer_mode_change->yaw.target_angle = engineer_mode_change->yaw.current_angle;
        engineer_mode_change->pitch.target_angle = engineer_mode_change->pitch.current_angle;
    }
    else if (engineer_mode_change->last_mode != FORCELESS_MODE && engineer_mode_change->mode == FORCELESS_MODE){    // �л�������ģʽ
        engineer_mode_change->yaw.target_angle = engineer_mode_change->yaw.current_angle;
        engineer_mode_change->pitch.target_angle = engineer_mode_change->pitch.current_angle;
    }
    engineer_mode_change->last_mode = engineer_mode_change->mode;
}



/**
 * @brief ���ݸ��º���
 * @param feedback_update
 */
void gimbal_feedback_update(gimbal_control_t* feedback_update){
    if (feedback_update == NULL){
        return;}
    feedback_update->pitch.absolute_angle = *(feedback_update->gimbal_INT_angle_point + INS_PITCH_ADDRESS_OFFSET);
    feedback_update->pitch.absolute_gyro = *(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Y_ADDRESS_OFFSET);

    feedback_update->yaw.absolute_angle = *(feedback_update->gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET);
    feedback_update->yaw.absolute_gyro = arm_cos_f32(feedback_update->pitch.current_angle) * (*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET))
                                                   - arm_sin_f32(feedback_update->pitch.current_angle) * (*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET));
    /***************************���½Ƕ�*************************/
    feedback_update->yaw.current_angle = feedback_update->yaw.motor_measure.motor_DM->position;
    feedback_update->pitch.current_angle = feedback_update->pitch.motor_measure.motor_DJI->Now_Angle;

    /***************************�����ٶ�*************************/

    feedback_update->yaw.current_speed = feedback_update->yaw.motor_measure.motor_DM->velocity;
    feedback_update->pitch.current_speed = feedback_update->pitch.motor_measure.motor_DJI->Now_Omega;
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


void gimbal_position_control(gimbal_control_t *control){
    if (control == NULL)
        return;


    if(control->mode == FORCELESS_MODE)
    {
        control->pitch.given_current = 0;
    }
    else
    {
        control->pitch.speed_set = PID_calc(&control->pitch.position_pid,
                                            control->pitch.current_angle,
                                            control->pitch.target_angle);

        control->pitch.given_current = PID_calc(&control->pitch.speed_pid,
                                                control->pitch.motor_measure.motor_DJI->raw_speed_rpm,
                                                control->pitch.speed_set);
    }
    static int time = 0;
    time++;
    if(time==1)
    {
        MIT_CtrlMotor(&hcan1, 0x02, control->yaw.target_angle, 0.0f, 5.0f, 0.2f, 0.0f);
    }
    else if (time==2)
    {
        CAN_cmd_DJI_control((int16_t)control->pitch.given_current,0,0,0);
        time= 0;
    }
}