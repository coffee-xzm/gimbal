/**
****************************(C) COPYRIGHT 2025 WDR****************************
  * @file       imu_task.c
  * @brief       Only the gyroscope bmi088 was used to complete the attitude calculation and obtain the euler Angle,
  *              while the magnetometer IST8310 was susceptible to interference and was not used
  *              Provides external triggering via bmi088's data ready interrupt to reduce data wait latency
  *              Save CPU time with DMA SPI transfers.
  *             ֻ����������bmi088�����̬���㣬�ó�ŷ���ǣ�������IST8310���ܸ��Ų�δʹ��
  *             �ṩͨ��bmi088��data ready �ж�����ⲿ�������������ݵȴ��ӳ�
  *             ͨ��DMA��SPI�����ԼCPUʱ��.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Mar-27-2025     Du.X.Y              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 WDR****************************
  */
#include "AHRS.h"
#include "bsp_imu_pwm.h"
#include "bsp_spi.h"
#include "cmsis_os.h"
#include "imu.h"
#include "pid.h"

//task handler ������
TaskHandle_t INS_task_local_handler;

pid_type_def imu_temp_pid;
bmi088_real_data_t bmi088_real_data;

extern SPI_HandleTypeDef hspi1;

static fp32 INS_gyro[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_accel[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_mag[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
fp32 INS_angle[3] = {0.0f, 0.0f, 0.0f};      //euler angle, unit rad.ŷ���� ��λ rad

//���ٶȼƵ�ͨ�˲�
static fp32 accel_filter_1[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_filter_2[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_filter_3[3] = {0.0f, 0.0f, 0.0f};
static const fp32 filter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};

uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LENGHT];
uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGHT] = {0x82,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LENGHT];
uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGHT] = {0x92,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

uint8_t accel_temp_dma_rx_buf[SPI_DMA_ACCEL_TEMP_LENGHT];
uint8_t accel_temp_dma_tx_buf[SPI_DMA_ACCEL_TEMP_LENGHT] = {0xA2,0xFF,0xFF,0xFF};

static uint8_t first_temperate;
static const float timing_time = 0.001f;   //tasK run time , unit s.�������е�ʱ�� ��λ s

volatile uint8_t imu_start_flag = 0;
volatile uint8_t gyro_update_flag = 0;
volatile uint8_t accel_update_flag = 0;
volatile uint8_t accel_temp_update_flag = 0;
volatile uint8_t mag_update_flag = 0;
volatile uint8_t imu_start_dma_flag = 0;

static void imu_temp_control(fp32 temp);
static void imu_cmd_spi_dma(void);


/**
 * @brief IMU��������ֻ�õ���BMI088�����������ڿ�����������δʹ��
 * @param argument none
 */
_Noreturn void imu_task(void const * argument)
{
    //wait a time
    osDelay(INS_TASK_INIT_TIME);

    Waiting_BMI088_Init();

    BMI088_read(bmi088_real_data.gyro,bmi088_real_data.accel,&bmi088_real_data.temp);

    imu_cali_slove(INS_gyro,INS_accel,&bmi088_real_data);

    BMI088_pid_init(&imu_temp_pid);

    AHRS_init(INS_quat,INS_accel,INS_mag);

    accel_filter_1[0] = accel_filter_2[0] = accel_filter_3[0] = INS_accel[0];
    accel_filter_1[1] = accel_filter_2[1] = accel_filter_3[1] = INS_accel[1];
    accel_filter_1[2] = accel_filter_2[2] = accel_filter_3[2] = INS_accel[2];

    INS_task_local_handler = xTaskGetHandle(pcTaskGetName(NULL));

    Waiting_BMI088_SPI_init();

    SPI1_DMA_init((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);

    imu_start_dma_flag = 1;

    while (1)
    {
        //�ȴ�SPI DMA����
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
        {
        }

        if(gyro_update_flag & (1 << IMU_NOTIFY_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_NOTIFY_SHFITS);
            BMI088_gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, bmi088_real_data.gyro);
        }

        if(accel_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, bmi088_real_data.accel, &bmi088_real_data.time);

        }

        if(accel_temp_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_temperature_read_over(accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, &bmi088_real_data.temp);
            imu_temp_control(bmi088_real_data.temp);
        }

        imu_cali_slove(INS_gyro,INS_accel,&bmi088_real_data);

        //accel low-pass filter
        accel_filter_1[0] = accel_filter_2[0];
        accel_filter_2[0] = accel_filter_3[0];

        accel_filter_3[0] = accel_filter_2[0] * filter_num[0] + accel_filter_1[0] * filter_num[1] + INS_accel[0] * filter_num[2];

        accel_filter_1[1] = accel_filter_2[1];
        accel_filter_2[1] = accel_filter_3[1];

        accel_filter_3[1] = accel_filter_2[1] * filter_num[0] + accel_filter_1[1] * filter_num[1] + INS_accel[1] * filter_num[2];

        accel_filter_1[2] = accel_filter_2[2];
        accel_filter_2[2] = accel_filter_3[2];

        accel_filter_3[2] = accel_filter_2[2] * filter_num[0] + accel_filter_1[2] * filter_num[1] + INS_accel[2] * filter_num[2];

        AHRS_update(INS_quat, timing_time, INS_gyro, accel_filter_3, INS_mag);
        get_angle(INS_quat, INS_angle + INS_YAW_ADDRESS_OFFSET, INS_angle + INS_PITCH_ADDRESS_OFFSET, INS_angle + INS_ROLL_ADDRESS_OFFSET);

        if(mag_update_flag &= 1 << IMU_DR_SHFITS)
        {
            mag_update_flag &= ~(1<< IMU_DR_SHFITS);
            mag_update_flag |= (1 << IMU_SPI_SHFITS);

        }

        osDelay(1);
    }
}

/**
  * @brief          ����bmi088���¶�
  * @param[in]      temp:bmi088���¶�
  * @retval         none
  */
static void imu_temp_control(fp32 temp)
{
    static uint8_t temp_constant_time = 0;
    if (first_temperate)
    {
        PID_calc(&imu_temp_pid, temp, get_control_temperature());
        if (imu_temp_pid.out < 0.0f)
        {
            imu_temp_pid.out = 0.0f;
        }
        const uint16_t tempPWM = (uint16_t)imu_temp_pid.out;
        IMU_temp_PWM(tempPWM);
    }
    else
    {
        //��û�дﵽ���õ��¶ȣ�һֱ����ʼ���
        //in beginning, max power
        if (temp > get_control_temperature())
        {
            temp_constant_time++;
            if (temp_constant_time > 200)
            {
                //�ﵽ�����¶ȣ�������������Ϊһ������ʣ���������
                //
                first_temperate = 1;
                imu_temp_pid.Iout = BMI088_TEMP_PWM_MAX / 2.0f;
            }
        }

        IMU_temp_PWM(BMI088_TEMP_PWM_MAX - 1);
    }
}


extern const fp32 *get_mag_data_point(void)
{
    return INS_mag;
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == INT1_ACCEL_Pin)
    {

        accel_update_flag |= 1 << IMU_DR_SHFITS;
        accel_temp_update_flag |= 1 << IMU_DR_SHFITS;
        if(imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if(GPIO_Pin == INT1_GYRO_Pin)
    {

        gyro_update_flag |= 1 << IMU_DR_SHFITS;
        if(imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if(GPIO_Pin == DRDY_IST8310_Pin)
    {

        mag_update_flag |= 1 << IMU_DR_SHFITS;
    }
    else if(GPIO_Pin == GPIO_PIN_0)
    {

        //wake up the task
        //��������
        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        {
            static BaseType_t xHigherPriorityTaskWoken;
            vTaskNotifyGiveFromISR(INS_task_local_handler, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }

    }


}

/**
  * @brief          open the SPI DMA accord to the value of imu_update_flag
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ����imu_update_flag��ֵ����SPI DMA
  * @param[in]      temp:bmi088���¶�
  * @retval         none
  */
static void imu_cmd_spi_dma(void)
{

    //���������ǵ�DMA����
    if( (gyro_update_flag & (1 << IMU_DR_SHFITS) ) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
        && !(accel_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        gyro_update_flag &= ~(1 << IMU_DR_SHFITS);
        gyro_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
        return;
    }
    //�������ٶȼƵ�DMA����
    if((accel_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
       && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        accel_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)accel_dma_tx_buf, (uint32_t)accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);
        return;
    }




    if((accel_temp_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
       && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        accel_temp_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_temp_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)accel_temp_dma_tx_buf, (uint32_t)accel_temp_dma_rx_buf, SPI_DMA_ACCEL_TEMP_LENGHT);
        return;
    }
}


void DMA2_Stream2_IRQHandler_Data_process(void)
{

    if(__HAL_DMA_GET_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx)) != RESET)
    {
        __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));

        //gyro read over
        //�����Ƕ�ȡ���
        if(gyro_update_flag & (1 << IMU_SPI_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_SPI_SHFITS);
            gyro_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);

        }

        //accel read over
        //���ٶȼƶ�ȡ���
        if(accel_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }
        //temperature read over
        //�¶ȶ�ȡ���
        if(accel_temp_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_temp_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }

        imu_cmd_spi_dma();

        if(gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            gyro_update_flag |= (1 << IMU_NOTIFY_SHFITS);
            __HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_0);
        }
    }
}

/**
  * @brief          get the euler angle, 0:yaw, 1:pitch, 2:roll unit rad
  * @retval         the point of INS_angle
  */
const fp32 *get_INS_angle_point(void)
{
    return INS_angle;
}
const fp32 *get_INS_gyro_point(void)
{
    return INS_gyro;
}