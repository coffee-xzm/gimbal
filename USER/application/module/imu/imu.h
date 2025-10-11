//
// Created by Du.X.Y on 2025/3/26.
//

#ifndef IMU_H
#define IMU_H

#include "BMI088driver.h"
#include "pid.h"
#define IMU_temp_PWM(pwm)  imu_pwm_set(pwm)                    //pwm给定

#define INS_TASK_INIT_TIME 7

#define TEMPERATURE_PID_KP 1600.0f //kp of temperature control PID
#define TEMPERATURE_PID_KI 0.2f    //ki of temperature control PID
#define TEMPERATURE_PID_KD 0.0f    //kd of temperature control PID

#define TEMPERATURE_PID_MAX_OUT 4500.0f  //max out of temperature control PID
#define TEMPERATURE_PID_MAX_IOUT 4400.0f //max iout of temperature control PID

#define SPI_DMA_GYRO_LENGHT       8
#define SPI_DMA_ACCEL_LENGHT      9
#define SPI_DMA_ACCEL_TEMP_LENGHT 4

#define IMU_NOTIFY_SHFITS    3
#define IMU_UPDATE_SHFITS    2
#define IMU_SPI_SHFITS       1
#define IMU_DR_SHFITS        0

#define BMI088_GYRO_RX_BUF_DATA_OFFSET  1
#define BMI088_ACCEL_RX_BUF_DATA_OFFSET 2

#define BMI088_TEMP_PWM_MAX 5000 //BMI088控制温度的设置TIM的重载值，即给PWM最大为 BMI088_TEMP_PWM_MAX - 1

#define INS_YAW_ADDRESS_OFFSET    0
#define INS_PITCH_ADDRESS_OFFSET  1
#define INS_ROLL_ADDRESS_OFFSET   2
// 陀螺仪数据偏移地址
#define INS_GYRO_X_ADDRESS_OFFSET 0
#define INS_GYRO_Y_ADDRESS_OFFSET 1
#define INS_GYRO_Z_ADDRESS_OFFSET 2


void Waiting_BMI088_Init(void);
void BMI088_pid_init(pid_type_def* imu_temp_pid);
void Waiting_BMI088_SPI_init(void);
void imu_cali_slove(fp32 gyro[3], fp32 accel[3], const bmi088_real_data_t *bmi088);
float get_control_temperature(void);

#endif //IMU_H
