#include "gimbal_task.h"
#include "cmsis_os.h"
#include "gimbal.h"
#include "SEGGER_SYSVIEW.h"

void gimbal_task(void const *argument)
{
    osDelay(2000);
    gimbal_init(&gimbal_control); // 初始化云台控制数据结构体
    
    while (1)
    {

        SEGGER_SYSVIEW_OnTaskStartExec(osThreadGetId());

        gimbal_mode_set(&gimbal_control);  // 设置云台控制模式

        gimbal_mode_change_control_transit(&gimbal_control);  // 模式切换状态保存

        gimbal_feedback_update(&gimbal_control);    // 更新云台反馈数据

        HandleCurrentMode(&gimbal_control);  // 处理当前模式

        gimbal_position_control(&gimbal_control);  // 位置控制

        SEGGER_SYSVIEW_OnTaskStopExec();

        osDelay(1);
        // taskYIELD();
    }
}