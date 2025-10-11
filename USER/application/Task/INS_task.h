/**
****************************(C) COPYRIGHT 2025 WDR****************************
  * @file       imu_task.h
  * @brief      IMU task header file - BMI088 gyroscope for attitude calculation
  *             Provides external triggering via BMI088's data ready interrupt
  *             Uses DMA SPI transfers to save CPU time
  *             IMU任务头文件 - 使用BMI088陀螺仪进行姿态解算
  *             通过BMI088的数据就绪中断提供外部触发
  *             使用DMA SPI传输以节省CPU时间
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     10-03-2025     sykk             1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 WDR****************************
  */

#ifndef IMU_TASK_H
#define IMU_TASK_H

#include "AHRS_middleware.h"

_Noreturn void imu_task(void const * argument);
const fp32 *get_INS_angle_point(void);
const fp32 *get_INS_gyro_point(void);
#endif /* IMU_TASK_H */