#include "DJI_Motor.h"
#include "can.h"

motor_measure_t dji_motor ;  //大疆电机
#define ZERO_ANGLE 4832
/**
 * @brief 解析CAN数据并填充到motor_3508_measure_t结构体中。
 *
 * 该函数接收一个指向motor_3508_measure_t结构体的指针和一个包含原始CAN数据的字节数组。
 * 从数据中提取编码器计数（ecd）、转速（RPM）、给定电流和温度，并将其存储到结构体中。
 *
 * @param ptr 指向motor_3508_measure_t结构体的指针，用于存储解析后的数据。
 * @param data 指向包含原始CAN数据的字节数组的指针。
 */
void dji_motor_measure_parse(motor_measure_t *ptr, const uint8_t *data) {
    ptr->raw_last_ecd = ptr->raw_ecd; // 存储上一次的编码器计数
    ptr->raw_ecd = (uint16_t)((data[0] << 8) | data[1]); // 提取当前编码器计数
    ptr->raw_speed_rpm = (uint16_t)((data[2] << 8) | data[3]); // 提取转速（RPM） NOLINT(*-narrowing-conversions)
    ptr->raw_given_current = (uint16_t)((data[4] << 8) | data[5]); // 提取给定电流 NOLINT(*-narrowing-conversions)
    ptr->raw_temperate = data[6]; // 提取温度
}

/**
 * @brief 简化版本：将指定位置设为0，自动将对面位置设为π
 * @param encoder_value 当前编码器值 (0-8191)
 * @param zero_position 要设为0弧度的机械位置
 * @return 映射后的弧度值 (0-2π)
 */
float simple_encoder_to_radians(int encoder_value, int zero_position) {
    // 计算相对于零点的位置
    const int ENCODER_RES = 8192;

    int relative_pos = (encoder_value - zero_position + ENCODER_RES) % ENCODER_RES;

    // 映射到0-2π
    float radians = (float)relative_pos / ENCODER_RES * 2.0f * M_PI;

    // 转换为-π到π范围
    if (radians > M_PI) {
        radians -= 2.0f * M_PI;
    }

    return radians;
}



/**
 * @brief 大疆6020电机反馈数据处理
 * @param can_id
 * @param rx_data
 */
void DJI_6020_data_Get(uint8_t* data)
{
    dji_motor_measure_parse(&dji_motor,data);
    int16_t delta_encoder;  //编码器增量
    uint16_t tmp_encoder;   //编码器值
    int16_t tmp_omega, tmp_torque, tmp_temperature; //角速度、扭矩、温度
    tmp_omega = dji_motor.raw_speed_rpm;
    tmp_encoder = dji_motor.raw_ecd;
    tmp_torque = dji_motor.raw_given_current;
    tmp_temperature = dji_motor.raw_temperate;
    delta_encoder = tmp_encoder - dji_motor.raw_last_ecd; //计算编码器增量
    if(delta_encoder < -Encoder_Num_Per_Round / 2)
    {
        //正方向转过了一圈
        dji_motor.Total_Round++;
    }
    else if(delta_encoder > Encoder_Num_Per_Round / 2)
    {
        //反方向转过了一圈
        dji_motor.Total_Round--;
    }
    dji_motor.Total_Encoder = dji_motor.Total_Round * Encoder_Num_Per_Round + tmp_encoder;
    //计算电机本身信息
    dji_motor.Now_Angle = simple_encoder_to_radians(dji_motor.raw_ecd,4832) ;
    dji_motor.Now_Omega = (float) tmp_omega * RPM_TO_RADPS ;
    dji_motor.Now_Current = tmp_torque / 1000.0f;
    dji_motor.Now_Temperature = (float)tmp_temperature ;

    //存储预备信息
    dji_motor.Pre_Encoder = tmp_encoder;
}

/**
* @brief 电机数据处理回调函数
* @param can_id
* @param rx_data
*/
void dji_motor_can_callback(uint32_t can_id, const uint8_t* rx_data)
{
    switch (can_id){
        case DJI_MOTOR_6020_RXID:  //0x206
            DJI_6020_data_Get(rx_data);
            break;
        default:
            break;
    }
}

/**
 * 大疆电机控制
 * @param motor
 */
void CAN_cmd_DJI_control(int16_t motor1, int16_t motor2, int16_t motor3, int16_t rev)
{
    uint32_t send_mail_box; // 发送邮箱编号
    uint8_t gimbal_can_send_data[8]; // CAN发送数据缓冲区

    // 配置CAN消息头
    CAN_TxHeaderTypeDef tx_message = {
            .StdId = 0x1FF, // 标准ID
            .IDE = CAN_ID_STD,          // 标准帧
            .RTR = CAN_RTR_DATA,        // 数据帧
            .DLC = 0x08,                // 数据长度为8字节
    };

    // 将电机控制数据填充到CAN数据缓冲区
    gimbal_can_send_data[0] = motor1 >> 8; // motor1高字节
    gimbal_can_send_data[1] = motor1 & 0xFF;       // motor1低字节
    gimbal_can_send_data[2] = motor2 >> 8; // motor2高字节
    gimbal_can_send_data[3] = motor2 & 0xFF;       // motor2低字节
    gimbal_can_send_data[4] = motor3 >> 8; // motor3高字节
    gimbal_can_send_data[5] = motor3 & 0xFF;       // motor3低字节
    gimbal_can_send_data[6] = rev >> 8;   // rev高字节
    gimbal_can_send_data[7] = rev & 0xFF;          // rev低字节

    // 发送CAN消息
     HAL_CAN_AddTxMessage(&hcan1, &tx_message, gimbal_can_send_data, &send_mail_box);
}

/**
 * @brief 获取指定索引的motor_measure_t结构体指针。
 *
 * @param i 数组索引，范围为0到2。
 * @return 指向指定索引的motor_measure_t结构体的指针。
 */
const motor_measure_t* get_motor_measure_point()
{
    return &dji_motor;
}