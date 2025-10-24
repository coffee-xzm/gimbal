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

//6020Joint4�Ƕ��ڻ�
#define M6020_MOTOR_POSITION_PID_KP 6000.0f
#define M6020_MOTOR_POSITION_PID_KI 0
#define M6020_MOTOR_POSITION_PID_KD 0
#define M6020_MOTOR_POSITION_PID_MAX_OUT 1500.0f
#define M6020_MOTOR_POSITION_PID_MAX_IOUT 400.0f
//6020Joint4�ٶ��⻷
#define M6020_MOTOR_SPEED_PID_KP 12.0f//10
#define M6020_MOTOR_SPEED_PID_KI 0.06f
#define M6020_MOTOR_SPEED_PID_KD 0
#define M6020_MOTOR_SPEED_PID_MAX_OUT 30000.0f
#define M6020_MOTOR_SPEED_PID_MAX_IOUT 10000.0f

typedef enum {
    INIT_MODE = 0,      // ��ʼ��ģʽ
    FORCELESS_MODE,
    REMOTE_MODE,        // ң����ģʽ
    AUTO_MODE,          // �Զ�ģʽ
    NUM_MODES           // �߽�ֵ
} gimbal_mode_e;

// ģʽ������ָ��
typedef void (*ModeHandler)(void);

// ģʽ������ע���
typedef struct {
    gimbal_mode_e mode;        // ģʽ
    ModeHandler handler;    // ��ģʽ��Ӧ�ô�����
} ModeHandlerEntry;

/**
 * @brief ������̨���״̬���ݽṹ��
 */
typedef struct
{
    union
    {
        const motor_measure_t *motor_DJI;                // ָ��󽮵���������ݵĳ���ָ��
        const dm_motor_measure_t *motor_DM;              // ָ��������������ݵĳ���ָ��
    } motor_measure;
    float current_angle;    // ��ǰ�Ƕ�
    float absolute_angle;   // �����ǽǶ�
    float absolute_gyro;    // �����ǽ��ٶ�
    float target_angle;     // Ŀ��Ƕ�
    float error_angle;      // �Ƕ����
    float bias_angle;       // ����Ƕ�ƫ��
    float speed_set;        // Ŀ���ٶ�
    float current_speed;    // ��ǰ�ٶ�
    float given_current;    // ��������ֵ
    float torque;           // ����ֵ
    float max_angle;        // ���ֵ
    float min_angle;        // ��Сֵ
    float middle_angle;     // �м�ֵ = (���ֵ + ��Сֵ)*0.5
    uint8_t motor_id;       // ���ID
    pid_type_def position_pid;               // λ��PID������
    pid_type_def speed_pid;                  // �ٶ�PID������
} gimbal_status_t;

/**
 * @brief ������̨�������ݽṹ��
 */
typedef struct
{
    const RC_ctrl_t *gimbal_rc_ctrl;   // the point to remote control
    gimbal_status_t pitch; // pitch���
    gimbal_status_t yaw;  // yaw���
    gimbal_mode_e mode;       // ��̨����ģʽ
    gimbal_mode_e last_mode;  // ��һ�ε���̨����ģʽ
    ModeHandlerEntry modeHandlers[NUM_MODES];   // ģʽ������ע���
    uint8_t registeredModes;                      // ��ע���ģʽ����
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