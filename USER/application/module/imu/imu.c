//
// Created by Du.X.Y on 2025/3/26.
//
#include "imu.h"

#include "bsp_adc.h"
#include "bsp_spi.h"
#include "cmsis_os.h"
#include "pid.h"

#define BMI088_BOARD_INSTALL_SPIN_MATRIX    \
{0.0f, 1.0f, 0.0f},                     \
{-1.0f, 0.0f, 0.0f},                     \
{0.0f, 0.0f, 1.0f}                      \


#define IST8310_BOARD_INSTALL_SPIN_MATRIX   \
{1.0f, 0.0f, 0.0f},                     \
{0.0f, 1.0f, 0.0f},                     \
{0.0f, 0.0f, 1.0f}                      \

extern SPI_HandleTypeDef hspi1;

fp32 gyro_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 gyro_offset[3];
fp32 accel_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 accel_offset[3];

const fp32 imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};

/**
 * @brief 等待BMI088初始化
 */
void Waiting_BMI088_Init(void)
{
    while (BMI088_init())
    {
        ;
    }
}

void BMI088_pid_init(pid_type_def* imu_temp_pid)
{
    osDelay(10);
    PID_init(imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);
}

void Waiting_BMI088_SPI_init(void)
{
    //set spi frequency
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;

    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief          旋转陀螺仪,加速度计和磁力计,并计算零漂,因为设备有不同安装方式
  * @param[out]     gyro: 加上零漂和旋转
  * @param[out]     accel: 加上零漂和旋转
  * @param[in]      bmi088: 陀螺仪和加速度计数据
  * @retval         none
  */
void imu_cali_slove(fp32 gyro[3], fp32 accel[3], const bmi088_real_data_t *bmi088)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        gyro[i] = bmi088->gyro[0] * gyro_scale_factor[i][0] + bmi088->gyro[1] * gyro_scale_factor[i][1] + bmi088->gyro[2] * gyro_scale_factor[i][2] + gyro_offset[i];
        accel[i] = bmi088->accel[0] * accel_scale_factor[i][0] + bmi088->accel[1] * accel_scale_factor[i][1] + bmi088->accel[2] * accel_scale_factor[i][2] + accel_offset[i];
    }
}

/**
 *
 * @return 目标IMU温度
 */
float get_control_temperature(void)
{
    const float temperature = get_temprate() + 10.0f;
    return temperature;
}
