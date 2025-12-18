#include "usb_task.h"
#include "usb_device.h"
#include "cmsis_os.h"
#include "usb_callback.h"
#include "SEGGER_SYSVIEW.h"

void usbSendTask(void const * argument)
{

    osDelay(1000);
    // USB任务实现
    while(1)
    {
        SEGGER_SYSVIEW_OnTaskStartExec(osThreadGetId());
        // 这里可以添加USB相关的处理逻辑
        usb_send_gimbal_data();
        osDelay(1);
        SEGGER_SYSVIEW_OnTaskStopExec();
    }
}
