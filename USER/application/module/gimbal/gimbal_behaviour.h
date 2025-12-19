#ifndef STANDARD_ROBOT_CONTROLLER_MAPPING_H
#define STANDARD_ROBOT_CONTROLLER_MAPPING_H

#include "gimbal.h"
// 初始化模式
void HandleInitMode() ;
// 键盘控制模式
void HandleKeyboardMode() ;
// 遥操作模式
void HandleRemoteMode();
// 无力模式
void HandleGravityCompensationMode();
// 过渡模式
void HandleTransitionMode();
// 自动模式
// void HandleAutoMode();
// 机械臂控制模式切换函数
void gimbal_behaviour_mode_set(gimbal_control_t* set_mode);

#endif //STANDARD_ROBOT_ENGINEER_BEHAVIOUR_H