import serial
import struct
import time
import math
import threading
from datetime import datetime


class GimbalDataSender:
    def __init__(self, port='COM9', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.running = False
        self.thread = None
        # 添加平滑处理相关变量
        self.last_yaw = 0.0
        self.last_pitch = 0.0
        self.last_time = time.time()

    def calculate_crc16(self, data):
        """计算CRC16校验"""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc = crc >> 1
        return crc

    def pack_vision_data(self, mode, yaw, pitch):
        """打包视觉数据"""
        packet_size = 29
        send_buffer = bytearray(packet_size)

        # 填充帧头 (位置0-1)
        send_buffer[0] = ord('S')
        send_buffer[1] = ord('P')

        # 填充模式 (位置2)
        send_buffer[2] = mode

        # 填充角度数据（小端序）
        struct.pack_into('<f', send_buffer, 3, yaw)
        struct.pack_into('<f', send_buffer, 7, 0.0)
        struct.pack_into('<f', send_buffer, 11, 0.0)
        struct.pack_into('<f', send_buffer, 15, pitch)
        struct.pack_into('<f', send_buffer, 19, 0.0)
        struct.pack_into('<f', send_buffer, 23, 0.0)

        # 计算CRC16（不包括CRC自身的2字节）
        crc_data = send_buffer[:27]  # 29-2=27
        crc16 = self.calculate_crc16(crc_data)

        # 填充CRC16（小端序，位置27-28）
        struct.pack_into('<H', send_buffer, 27, crc16)

        return send_buffer

    def smooth_angle_change(self, current_angle, last_angle, max_rate=0.5):
        """平滑角度变化，限制最大变化率"""
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        if dt <= 0:
            dt = 0.01

        # 计算角度变化率
        angle_diff = current_angle - last_angle
        rate = angle_diff / dt

        # 限制最大变化率
        if rate > max_rate:
            smoothed_angle = last_angle + max_rate * dt
        elif rate < -max_rate:
            smoothed_angle = last_angle - max_rate * dt
        else:
            smoothed_angle = current_angle

        return smoothed_angle

    def low_pass_filter(self, current_value, last_value, alpha=0.4):
        """低通滤波器"""
        return alpha * current_value + (1.0 - alpha) * last_value

    def send_sine_wave_data(self, frequency=0.1, amplitude=0.3):
        """发送正弦波数据"""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            self.running = True

            print(f"开始向 {self.port} 发送平滑正弦波数据...")
            print(f"数据包大小: 29字节")
            print("Yaw和Pitch在 [-0.3, 0.3] 范围内平滑变化")
            print("使用低通滤波和速率限制")
            print("按 Ctrl+C 停止")
            print("-" * 70)

            start_time = time.time()
            packet_count = 0
            # 重置平滑变量
            self.last_yaw = 0.0
            self.last_pitch = 0.0
            self.last_time = time.time()

            while self.running:
                current_time = time.time() - start_time

                # 计算原始正弦波角度
                raw_yaw = amplitude * math.sin(2 * math.pi * frequency * current_time)
                raw_pitch = amplitude * math.sin(2 * math.pi * frequency * current_time + math.pi / 2)

                # 方法1：低通滤波（推荐）
                smooth_yaw = self.low_pass_filter(raw_yaw, self.last_yaw, 0.3)
                smooth_pitch = self.low_pass_filter(raw_pitch, self.last_pitch, 0.3)

                # 方法2：速率限制（备选）
                # smooth_yaw = self.smooth_angle_change(raw_yaw, self.last_yaw, 0.3)
                # smooth_pitch = self.smooth_angle_change(raw_pitch, self.last_pitch, 0.3)

                # 更新上一次的角度值
                self.last_yaw = smooth_yaw
                self.last_pitch = smooth_pitch

                # 打包数据
                packet = self.pack_vision_data(mode=1, yaw=smooth_yaw, pitch=smooth_pitch)

                # 验证数据包大小
                if len(packet) != 29:
                    print(f"错误: 数据包大小不正确: {len(packet)} 字节")
                    continue

                # 发送数据
                self.ser.write(packet)
                packet_count += 1

                # 显示发送状态
                if packet_count % 10 == 0:  # 每10个包显示一次，避免刷屏
                    print(f"\r时间: {current_time:6.1f}s | "
                          f"包: {packet_count:5d} | "
                          f"Yaw: {smooth_yaw:7.3f}rad | "
                          f"Pitch: {smooth_pitch:7.3f}rad",
                          end='', flush=True)

                # 控制发送频率（20Hz）
                time.sleep(0.05)

        except KeyboardInterrupt:
            print("\n\n用户中断发送")
        except Exception as e:
            print(f"\n发送错误: {e}")
        finally:
            if self.ser and self.ser.is_open:
                self.ser.close()
            print(f"\n总共发送 {packet_count} 个数据包")

    def start(self, frequency=0.1, amplitude=0.3):
        """启动发送线程"""
        self.thread = threading.Thread(
            target=self.send_sine_wave_data,
            args=(frequency, amplitude)
        )
        self.thread.daemon = True
        self.thread.start()
        print("发送器已启动")

    def stop(self):
        """停止发送"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)
        print("发送器已停止")


# 验证数据包结构的函数
def verify_packet_structure():
    """验证数据包结构"""
    print("=== 数据包结构验证 ===")
    print("VisionToGimbal 结构体:")
    print("  head[2]     : 2字节  (位置: 0-1)")
    print("  mode        : 1字节  (位置: 2)")
    print("  yaw         : 4字节  (位置: 3-6)")
    print("  yaw_vel     : 4字节  (位置: 7-10)")
    print("  yaw_acc     : 4字节  (位置: 11-14)")
    print("  pitch       : 4字节  (位置: 15-18)")
    print("  pitch_vel   : 4字节  (位置: 19-22)")
    print("  pitch_acc   : 4字节  (位置: 23-26)")
    print("  crc16       : 2字节  (位置: 27-28)")
    print("总计: 29字节")
    print("=" * 40)


def main():
    print("云台数据发送器 - 平滑版本")
    print("=" * 50)

    # 验证数据包结构
    verify_packet_structure()

    # 列出可用串口
    try:
        import serial.tools.list_ports
        ports = serial.tools.list_ports.comports()
        print("可用串口:")
        for port in ports:
            print(f"  {port.device} - {port.description}")
    except:
        pass

    print(f"\n使用端口: COM9 (STM32)")
    print("=" * 50)

    # 创建发送器
    sender = GimbalDataSender('COM9', 115200)

    try:
        # 启动发送
        sender.start(frequency=0.1, amplitude=0.8)

        # 主循环
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n正在停止发送器...")
    finally:
        sender.stop()
        print("程序已退出")


if __name__ == "__main__":
    main()