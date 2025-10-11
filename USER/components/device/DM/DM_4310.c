#include "DM_4310.h"

extern CAN_HandleTypeDef hcan1;

#define DM_CAN &hcan1

// 静态数组，用于存储达妙电机的测量数据
static dm_motor_measure_t dm_motor_measure[3] = {0};


/**
 * @brief 达妙电机数据处理回调函数
 *
 */
void dm_motor_can_callback(uint32_t can_id, const uint8_t* rx_data)
{
    switch (can_id)
    {
        case JOINT1_8009_RXID:
            dm_motor_measure_parse(&dm_motor_measure[0], rx_data);  // 解析数据并填充到结构体 8009-1 中
            break;
        case JOINT2_8009_RXID:
            dm_motor_measure_parse(&dm_motor_measure[1], rx_data);  // 解析数据并填充到结构体 8009-2 中
            break;
        case JOINT3_4310_RXID:
            dm_motor_measure_parse(&dm_motor_measure[2], rx_data);  // 解析数据并填充到结构体 4310 中
            break;

        default:
            break;
    }
}

/**
 * @brief 解析FDCAN数据并填充到motor_4310_measure_t结构体中。
 *
 * @param ptr 指向motor_4310_measure_t结构体的指针。
 * @param data 指向包含原始FDCAN数据的字节数组的指针。
 */
void dm_motor_measure_parse(dm_motor_measure_t* ptr, const uint8_t* data) {
    ptr->error = (data[0] >> 4);                    // 提取高 4 位作为错误码
    ptr->ID = (data[0] & 0xF);                      // 提取低 4 位作为 ID
    ptr->p_int = (data[1] << 8) | data[2];          // 组合成 16 位位置整数
    ptr->v_int = (data[3] << 4) | (data[4] >> 4);   // 组合成 12 位速度整数
    ptr->t_int = ((data[4] & 0xF) << 8) | data[5];  // 组合成 12 位扭矩整数
    ptr->position = DMmotor_UintToFloat(ptr->p_int, P_MIN, P_MAX, 16); // 转换为浮点数位置，单位：弧度 (rad)
    ptr->velocity = DMmotor_UintToFloat(ptr->v_int, V_MIN, V_MAX, 12); // 转换为浮点数速度，单位：弧度/秒 (rad/s)
    ptr->torque = DMmotor_UintToFloat(ptr->t_int, T_MIN, T_MAX, 12);   // 转换为浮点数扭矩，单位：牛·米 (N·m)
    ptr->T_mos = (float)data[6];                    // MOSFET 温度，单位：摄氏度 (°C)
    ptr->T_motor = (float)data[7];                  // 电机线圈温度，单位：摄氏度 (°C)
}

/**
 * @brief 获取指定索引的motor_4310_measure_t结构体指针。
 *
 * @param i 数组索引，范围为0到2。
 * @return 指向指定索引的motor_4310_measure_t结构体的指针。
 */
const dm_motor_measure_t* get_dm_motor_measure_point(uint8_t i)
{
    return &dm_motor_measure[i];
}

/**
 * @brief 使能电机。
 */
void DM_ALL_MotorEnable(uint8_t index)
{
    switch (index)
    {
        case 0:
            DM_MotorEnable(DM_CAN,  JOINT1_8009_TXID);
            break;
        case 1:
            DM_MotorEnable(DM_CAN, JOINT2_8009_TXID);
            break;
        case 2:
            DM_MotorEnable(DM_CAN,  JOINT3_4310_TXID);
            break;

        default:
            break;
    }
}

//达妙电机串级PID使能
void DM_MotorEnable(CAN_HandleTypeDef* hcan, uint16_t id){
    uint32_t send_mail_box; // 发送邮箱编号
    uint8_t dm_send_data[8] = {0}; // CAN发送数据缓冲区
    CAN_TxHeaderTypeDef tx_message; // CAN消息头

    // 配置CAN消息头
    tx_message.StdId = id; // 标准ID + 偏移量
    tx_message.IDE = CAN_ID_STD;                   // 标准帧
    tx_message.RTR = CAN_RTR_DATA;                // 数据帧
    tx_message.DLC = 0x08;                        // 数据长度为8字节
    tx_message.TransmitGlobalTime = DISABLE;      // 不启用全局时间传输

    // 填充CAN数据
    dm_send_data[0] = 0xFF;
    dm_send_data[1] = 0xFF;
    dm_send_data[2] = 0xFF;
    dm_send_data[3] = 0xFF;
    dm_send_data[4] = 0xFF;
    dm_send_data[5] = 0xFF;
    dm_send_data[6] = 0xFF;
    dm_send_data[7] = 0xFC; // 使能指令

    // 发送CAN消息
   HAL_CAN_AddTxMessage(hcan, &tx_message, dm_send_data, &send_mail_box);
}


/**
 * @brief 将无符号整数转换为浮点数。
 *
 * @param x_int 要转换的无符号整数。
 * @param x_min 目标浮点数的最小值。
 * @param x_max 目标浮点数的最大值。
 * @param bits 无符号整数的位数。
 * @return 转换后的浮点数。
 */
float DMmotor_UintToFloat(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + x_min;
}

/**
 * @brief 将浮点数转换为无符号整数。
 *
 * @param x 要转换的浮点数。
 * @param x_min 浮点数的最小值。
 * @param x_max 浮点数的最大值。
 * @param bits 无符号整数的位数。
 * @return 转换后的无符号整数。
 */
inline int DMmotor_FloatToUint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    return (int)((x - x_min) * ((float)((1 << bits) - 1)) / span);
}

/**
 * @brief  将浮点数转换为无符号整数
 * @param  x     			要转换的浮点数
 * @param  x_min      浮点数的最小值
 * @param  x_max    	浮点数的最大值
 * @param  bits      	无符号整数的位数
 */

int float_to_uint(float x, float x_min, float x_max, int bits){
    /// Converts a float to an unsigned int, given range and number of bits///
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}


/**
 * @brief  MIT模式控下控制帧
 * @param  hcan   CAN的句柄
 * @param  ID     数据帧的ID
 * @param  _pos   位置给定
 * @param  _vel   速度给定
 * @param  _KP    位置比例系数
 * @param  _KD    位置微分系数
 * @param  _torq  转矩给定值
 */
void MIT_CtrlMotor(CAN_HandleTypeDef* hcan, uint16_t id, float _pos, float _vel,
                   float _KP, float _KD, float _torq)
{
    // 定义CAN发送消息头
    CAN_TxHeaderTypeDef Tx_Header;
    uint32_t Tx_Mailbox; // 用于存储发送邮箱编号

    // 配置CAN消息头
    Tx_Header.StdId = id;           // 标准ID
    Tx_Header.ExtId = 0;            // 扩展ID（标准帧时设置为0）
    Tx_Header.IDE = CAN_ID_STD;     // 使用标准ID
    Tx_Header.RTR = CAN_RTR_DATA;   // 数据帧
    Tx_Header.DLC = 8;              // 数据长度为8字节

    // 将浮点数参数转换为整数值
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
    kp_tmp = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
    kd_tmp = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
    tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);

    // 填充CAN数据（按照MIT协议打包）
    uint8_t Tx_Data[8];
    Tx_Data[0] = (pos_tmp >> 8);
    Tx_Data[1] = pos_tmp;
    Tx_Data[2] = (vel_tmp >> 4);
    Tx_Data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    Tx_Data[4] = kp_tmp;
    Tx_Data[5] = (kd_tmp >> 4);
    Tx_Data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    Tx_Data[7] = tor_tmp;

    // 将消息添加到发送邮箱并发送
    HAL_CAN_AddTxMessage(hcan, &Tx_Header, Tx_Data, &Tx_Mailbox);
}


//达妙电机串级PID控制函数
void DM4310_PosSpeed_CtrlMotor(CAN_HandleTypeDef* hcan, uint16_t id, float _pos, float _vel)
{
    // 定义CAN发送消息头
    CAN_TxHeaderTypeDef Tx_Header;
    uint32_t Tx_Mailbox; // 用于存储发送邮箱编号

    // 配置CAN消息头
    Tx_Header.StdId = DM_SERIAL_MODE_OFFSET;          // 标准ID
    Tx_Header.ExtId = 0;           // 扩展ID（标准帧时设置为0）
    Tx_Header.IDE = CAN_ID_STD;    // 使用标准ID
    Tx_Header.RTR = CAN_RTR_DATA;  // 数据帧
    Tx_Header.DLC = 8;             // 数据长度为8字节

    // 将浮点数转换为字节数组
    uint8_t* pbuf = (uint8_t*)&_pos;
    uint8_t* vbuf = (uint8_t*)&_vel;

    // 填充CAN数据
    uint8_t Tx_Data[8] = {
            pbuf[0], pbuf[1], pbuf[2], pbuf[3],
            vbuf[0], vbuf[1], vbuf[2], vbuf[3]
    };

    // 将消息添加到发送邮箱并发送
   HAL_CAN_AddTxMessage(hcan, &Tx_Header, Tx_Data, &Tx_Mailbox);

}

//达妙关节电机控制
void CAN_cmd_DM_joint_control(float posi1,float vel1)
{
    DM4310_PosSpeed_CtrlMotor(&hcan1,1,posi1,vel1);
}

