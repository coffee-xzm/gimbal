#include "gimbal_behaviour.h"
#include "user_lib.h"
#include <usb_callback.h>

void gimbal_behaviour_mode_set(gimbal_control_t *set_mode)
{
    if (set_mode == NULL){
        return;
    }

    //��ʼ��ģʽ�ж��Ƿ񵽴���ֵλ��
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

        //������ʼ�����ʱ�䣬�����Ѿ��ȶ�����ֵһ��ʱ�䣬�˳���ʼ��״̬���ش��µ������ߵ���
        if (init_time < 6000 && init_stop_time < 100 &&
            !switch_is_down(set_mode->gimbal_rc_ctrl->s[2]) )
        {
            return;
        }
        else
        {
            init_stop_time = 0;
            init_time = 0;
        }
    }

    // ����յ��������ǿ�����ģʽ��־λ�����л���������ģʽ
    if(switch_is_up(gimbal_control.gimbal_rc_ctrl->s[2]))
    {
        ChangeMode(set_mode,AUTO_MODE);
    }
    else if(switch_is_mid(gimbal_control.gimbal_rc_ctrl->s[2]))
    {
        ChangeMode(set_mode,REMOTE_MODE);

    }
    else {
        ChangeMode(set_mode,FORCELESS_MODE);
    }
    // ChangeMode(set_mode,AUTO_MODE);

    //�жϽ���init״̬��
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

// �Զ�ģʽ
void HandleAutoMode() {
#define MIN_YAW_ANGLE -1.5f
#define MAX_YAW_ANGLE 1.5f
#define MIN_PITCH_ANGLE -1.5f
#define MAX_PITCH_ANGLE 1.5f

    static fp32 last_yaw = 0.0f;
    static fp32 last_pitch = 0.0f;

    VisionToGimbal data;
    get_vision_data(&data);

    gimbal_control.control_strategy = NormalControlStrategy;

    // �Ƕ�����
    fp32 limited_yaw = data.yaw;
    fp32 limited_pitch = data.pitch;

    if (limited_yaw > MAX_YAW_ANGLE) limited_yaw = MAX_YAW_ANGLE;
    else if (limited_yaw < MIN_YAW_ANGLE) limited_yaw = MIN_YAW_ANGLE;

    if (limited_pitch > MAX_PITCH_ANGLE) limited_pitch = MAX_PITCH_ANGLE;
    else if (limited_pitch < MIN_PITCH_ANGLE) limited_pitch = MIN_PITCH_ANGLE;

    // // ��ͨ�˲��� (alphaԽСԽƽ�������ӳ�Խ��)
    // #define FILTER_ALPHA 0.3f
    // fp32 filtered_yaw = FILTER_ALPHA * limited_yaw + (1.0f - FILTER_ALPHA) * last_yaw;
    // fp32 filtered_pitch = FILTER_ALPHA * limited_pitch + (1.0f - FILTER_ALPHA) * last_pitch;

    // last_yaw = filtered_yaw;
    // last_pitch = filtered_pitch;


    // gimbal_control.yaw.absolute_angle_set = filtered_yaw ;
    // gimbal_control.pitch.absolute_angle_set = -filtered_pitch;
    gimbal_control.yaw.absolute_angle_set = limited_yaw;
    gimbal_control.pitch.absolute_angle_set = limited_pitch;//todo ��������λ����ȥ�����ţ����������
    //TODO ֮���볢����tracealyzer
}

// ����ģʽ
void HandleGravityCompensationMode() {
    // ����ģʽ�߼�
    gimbal_control.control_strategy = ForcelessControlStrategy;
}


// ����ģʽ
void HandleRemoteMode()
{
#define MIN_YAW_ANGLE -1.5f
#define MAX_YAW_ANGLE 1.5f
#define MIN_PITCH_ANGLE -1.5f
#define MAX_PITCH_ANGLE 1.5f

    fp32 add_yaw_angle = 0.0f;
    fp32 add_pitch_angle = 0.0f;
    const float yaw_rc_sen = 0.00001f;
    const float pitch_rc_sen = 0.000024f;
    static int16_t yaw_channel = 0, pitch_channel = 0;

    rc_deadband_limit(gimbal_control.gimbal_rc_ctrl->ch[2], yaw_channel, 10);
    rc_deadband_limit(gimbal_control.gimbal_rc_ctrl->ch[3], pitch_channel, 10);

    add_yaw_angle = -(float)yaw_channel * yaw_rc_sen;
    add_pitch_angle = -(float)pitch_channel * pitch_rc_sen;

    // ���ÿ��Ʋ���
    gimbal_control.control_strategy = NormalControlStrategy;

    // Yaw��Ƕ����ƴ���������gimbal_absolute_angle_limit�߼���
    static fp32 yaw_bias_angle;
    static fp32 yaw_angle_set;

    // ��ǰ�������Ƕ�
    yaw_bias_angle = gimbal_control.yaw.absolute_angle_set - gimbal_control.yaw.absolute_angle;

    // ��̨��ԽǶ� + ���Ƕ� + �����Ƕ� ������� ����е�Ƕ�
    if (gimbal_control.yaw.relative_angle + yaw_bias_angle + add_yaw_angle > MAX_YAW_ANGLE)
    {
        // �����������е�Ƕȿ��Ʒ���
        if (add_yaw_angle > 0.0f)
        {
            // �����һ���������ӽǶ�
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
    gimbal_control.yaw.absolute_angle_set = yaw_angle_set + add_yaw_angle;

    // Pitch��Ƕ����ƴ���������gimbal_absolute_angle_limit�߼���
    static fp32 pitch_bias_angle;
    static fp32 pitch_angle_set;

    // ��ǰ�������Ƕ�
    pitch_bias_angle = gimbal_control.pitch.absolute_angle_set - gimbal_control.pitch.absolute_angle;

    // ��̨��ԽǶ� + ���Ƕ� + �����Ƕ� ������� ����е�Ƕ�
    if (gimbal_control.pitch.relative_angle + pitch_bias_angle + add_pitch_angle > MAX_PITCH_ANGLE)
    {
        // �����������е�Ƕȿ��Ʒ���
        if (add_pitch_angle > 0.0f)
        {
            // �����һ���������ӽǶ�
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
    gimbal_control.pitch.absolute_angle_set = pitch_angle_set + add_pitch_angle;

}


void HandleInitMode()
{
    //* �ϵ�ʱ���û�е���
    //! ��Ӧ�����������ã��ᱻ�ı�
    // gimbal_control.initial_yaw_motor_angle = gimbal_control.yaw.motor_measure.motor_DM->position;
    // gimbal_control.initial_pitch_motor_angle = gimbal_control.pitch.motor_measure.motor_DJI->Now_Angle;

    gimbal_control.yaw.relative_angle_set = 0.0f;
    gimbal_control.pitch.relative_angle_set = 0.0f;
}