#include "gimbal_task.h"
#include "cmsis_os.h"
#include "gimbal.h"

void gimbal_task(void const *argument)
{
    osDelay(2000);
    gimbal_init(&gimbal_control); // ��ʼ����̨�������ݽṹ��
    while (1)
    {

        gimbal_mode_set(&gimbal_control);  // ������̨����ģʽ

        gimbal_mode_change_control_transit(&gimbal_control);  // ģʽ�л�״̬����

        gimbal_feedback_update(&gimbal_control);    // ������̨��������

        HandleCurrentMode(&gimbal_control);  // ����ǰģʽ

        gimbal_position_control(&gimbal_control);  // λ�ÿ���

        osDelay(1);
    }
}