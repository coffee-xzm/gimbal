#ifndef STANDARD_ROBOT_GIMBAL_H
#define STANDARD_ROBOT_GIMBAL_H

#include <stdbool.h>
#include "main.h"
#include "DJI_Motor.h"
#include "pid.h"
#include "i6x.h"
#include "cmsis_os.h"

#define PI	3.14159265358979f

// ==================== Pitch��-GM6020 PID���Ʋ�����ʼ�� ====================
// ��6020���λ�û�PID����
#define M6020_MOTOR_SPEED_PID_KP 1300.0f//10
#define M6020_MOTOR_SPEED_PID_KI 15.0f
#define M6020_MOTOR_SPEED_PID_KD 0
#define M6020_MOTOR_SPEED_PID_MAX_OUT 30000.0f
#define M6020_MOTOR_SPEED_PID_MAX_IOUT 10000.0f

//pitch encode angle close-loop PID params, max out and max iout
//pitch �ǶȻ� �Ƕ��ɱ����� PID�����Լ� PID���������������
#define PITCH_ENCODE_RELATIVE_PID_KP 50.0f
#define PITCH_ENCODE_RELATIVE_PID_KI 5.00f
#define PITCH_ENCODE_RELATIVE_PID_KD 0.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_OUT 10.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_IOUT 0.0f

#define PITCH_GYRO_ABSOLUTE_PID_KP        10.0f
#define PITCH_GYRO_ABSOLUTE_PID_KI        0.0f
#define PITCH_GYRO_ABSOLUTE_PID_KD        0.3f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT   10.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT  0.0f

// ==================== YAW��-GM6020 PID���Ʋ�����ʼ�� ====================
//���radar_gimbal-main����
//yaw speed close-loop PID params, max out and max iout
#define YAW_SPEED_PID_KP        3000.0f
#define YAW_SPEED_PID_KI        20.0f
#define YAW_SPEED_PID_KD        0.0f
#define YAW_SPEED_PID_MAX_OUT   30000.0f
#define YAW_SPEED_PID_MAX_IOUT  5000.0f
//yaw gyro angle close-loop PID params, max out and max iout
#define YAW_GYRO_ABSOLUTE_PID_KP        50.0f
#define YAW_GYRO_ABSOLUTE_PID_KI        0.8f
#define YAW_GYRO_ABSOLUTE_PID_KD        0.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_OUT   300.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_IOUT  300.0f
//yaw encode angle close-loop PID params, max out and max iout
#define YAW_ENCODE_RELATIVE_PID_KP        50.0f
#define YAW_ENCODE_RELATIVE_PID_KI        1.0f
#define YAW_ENCODE_RELATIVE_PID_KD        0.2f
#define YAW_ENCODE_RELATIVE_PID_MAX_OUT   300.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT  200.0f


typedef enum {
    INIT_MODE = 0,      // ��ʼ��ģʽ
    FORCELESS_MODE,
    REMOTE_MODE,        // ң����ģʽ
    AUTO_MODE,          // �Զ�ģʽ
    NUM_MODES           // �߽�ֵ
} gimbal_mode_e;


// ==================== ǰ������ ====================
typedef struct gimbal_control_s gimbal_control_t;

// ==================== ����ָ�����Ͷ��� ====================
// ģʽ��������ָ��
typedef void (*ModeHandler)(void);

// ���Ʋ��Ժ���ָ��
typedef void (*ControlStrategyFn)(gimbal_control_t*);

// ==================== ģʽ��������ע��� ====================
typedef struct {
    gimbal_mode_e mode;        // ģʽ
    ModeHandler handler;       // ��ģʽ��Ӧ�Ĵ�������
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
 * @brief ������̨���״̬���ݽṹ��
 */
typedef struct
{
    union
    {
        const motor_measure_t *motor_DJI;                // 指向大疆电机测量数据的常量指针
    } motor_measure;
    fp32 relative_angle;     //rad
    fp32 relative_angle_set; //rad
    fp32 absolute_angle;     //rad
    fp32 absolute_angle_set; //rad
    fp32 motor_gyro;         //rad/s
    fp32 motor_gyro_set;
    fp32 motor_speed;
    float given_current;    // ��������ֵ
    float torque;           // ����ֵ
    uint8_t motor_id;       // ���ID

    pid_type_def speed_pid;                  // �ٶ�PID������
    gimbal_PID_t gimbal_motor_absolute_angle_pid;
    gimbal_PID_t gimbal_motor_relative_angle_pid;
} gimbal_motor_status_t;

/**
 * @brief ������̨�������ݽṹ��
 */
struct gimbal_control_s
{
    // ==================== �������� ====================
    const i6x_ctrl_t *gimbal_rc_ctrl;        // ң��������ָ��
    const fp32 *gimbal_INT_angle_point;     // �����ǽǶ�����ָ��
    const fp32 *gimbal_INT_gyro_point;      // �����ǽ��ٶ�����ָ��

    // ==================== ִ�л��� ====================
    gimbal_motor_status_t yaw;        // Yaw����״̬
    gimbal_motor_status_t pitch;        // pitch����״̬
    fp32 initial_yaw_motor_angle;
    fp32 initial_pitch_motor_angle;

    // ==================== ģʽ���� ====================
    gimbal_mode_e mode;                     // ��ǰ����ģʽ
    gimbal_mode_e last_mode;                // ��һ�ο���ģʽ
    ModeHandlerEntry modeHandlers[NUM_MODES]; // ģʽ����������
    uint8_t registeredModes;                // ��ע��ģʽ����

    // ==================== ���Ʋ��� ====================
    ControlStrategyFn control_strategy;     // ��ǰ���Ʋ��Ժ���ָ��

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

// ��gimbal.h������
// ���Ʋ��Ժ�������
void NormalControlStrategy(gimbal_control_t* control);
void ForcelessControlStrategy(gimbal_control_t* control);
void InitControlStrategy(gimbal_control_t* control);
#endif //STANDARD_ROBOT_GIMBAL_H