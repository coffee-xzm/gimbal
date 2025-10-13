#include "usb_task.h"
#include "usb_device.h"
#include "cmsis_os.h"
#include "usb_callback.h"

void StartTask06(void const * argument)
{
    MX_USB_DEVICE_Init();
    osDelay(2000);
    // USB任务实现
    while(1)
    {
        // 这里可以添加USB相关的处理逻辑
        taskENTER_CRITICAL();
        usb_send_gimbal_data();
        taskEXIT_CRITICAL();
        osDelay(1);
    }
}
