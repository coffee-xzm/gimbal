#ifndef STANDARD_ROBOT_CONTROLLER_MAPPING_H
#define STANDARD_ROBOT_CONTROLLER_MAPPING_H

#include "gimbal.h"
// ��ʼ��ģʽ
void HandleInitMode() ;
// ���̿���ģʽ
void HandleKeyboardMode() ;
// ң����ģʽ
void HandleRemoteMode();
// ����ģʽ
void HandleGravityCompensationMode();
// ����ģʽ
void HandleTransitionMode();
// �Զ�ģʽ
void HandleAutoMode();
// ��е�ۿ���ģʽ�л�����
void gimbal_behaviour_mode_set(gimbal_control_t* set_mode);

#endif //STANDARD_ROBOT_ENGINEER_BEHAVIOUR_H