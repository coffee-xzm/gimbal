#include "usb_task.h"
#include "usb_device.h"
#include "cmsis_os.h"
#include "usb_callback.h"

void usbSendTask(void const * argument)
{
    usb_rx_init();

    osDelay(1000);
    while(1)
    {
        usb_rx_data_parse();
        usb_send_gimbal_data();
        osDelay(1);
    }
}
