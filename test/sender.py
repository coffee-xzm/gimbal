import serial
import struct
import threading
import time
import json
from datetime import datetime
from dataclasses import dataclass
from typing import Optional, Callable
from collections import deque


@dataclass
class GimbalData:
    head: bytes
    mode: int
    q: tuple  # (w, x, y, z)
    yaw: float
    yaw_vel: float
    pitch: float
    pitch_vel: float
    bullet_speed: float
    bullet_count: int
    crc16: int
    timestamp: float
    raw_data: bytes


class GimbalDataReceiver:
    def __init__(self, port='COM9', baudrate=115200, packet_size=43):
        self.port = port
        self.baudrate = baudrate
        self.packet_size = packet_size
        self.ser = None
        self.running = False
        self.thread = None
        self.buffer = bytearray()
        self.callbacks = []
        self.latest_data = None
        self.data_history = deque(maxlen=1000)  # 保存最近1000个数据包
        self.receive_count = 0
        self.error_count = 0

    def add_callback(self, callback: Callable[[GimbalData], None]):
        """添加数据回调函数"""
        self.callbacks.append(callback)

    def remove_callback(self, callback: Callable[[GimbalData], None]):
        """移除数据回调函数"""
        if callback in self.callbacks:
            self.callbacks.remove(callback)

    def unpack_packet(self, data: bytes) -> Optional[GimbalData]:
        """解包单个数据包"""
        try:
            if len(data) != self.packet_size:
                return None

            # 检查帧头
            if data[0:2] != b'SP':
                return None

            # 解包数据 - 小端序
            unpacked = struct.unpack('<2s B 4f 4f f H H', data)

            return GimbalData(
                head=unpacked[0],
                mode=unpacked[1],
                q=unpacked[2:6],
                yaw=unpacked[6],
                yaw_vel=unpacked[7],
                pitch=unpacked[8],
                pitch_vel=unpacked[9],
                bullet_speed=unpacked[10],
                bullet_count=unpacked[11],
                crc16=unpacked[12],
                timestamp=time.time(),
                raw_data=data
            )
        except Exception as e:
            self.error_count += 1
            print(f"解包错误: {e}")
            return None

    def process_data(self, data: bytes):
        """处理接收到的数据"""
        self.buffer.extend(data)

        # 查找完整的数据包
        while len(self.buffer) >= self.packet_size:
            # 查找帧头
            start_index = -1
            for i in range(len(self.buffer) - 1):
                if self.buffer[i] == ord('S') and self.buffer[i + 1] == ord('P'):
                    start_index = i
                    break

            if start_index == -1:
                # 没有找到帧头，清空缓冲区
                self.buffer.clear()
                break

            # 移除帧头之前的数据
            if start_index > 0:
                self.buffer = self.buffer[start_index:]

            # 检查是否有完整的数据包
            if len(self.buffer) >= self.packet_size:
                packet_data = bytes(self.buffer[:self.packet_size])
                self.buffer = self.buffer[self.packet_size:]

                # 解包数据
                gimbal_data = self.unpack_packet(packet_data)
                if gimbal_data:
                    self.receive_count += 1
                    self.latest_data = gimbal_data
                    self.data_history.append(gimbal_data)

                    # 调用所有回调函数
                    for callback in self.callbacks:
                        try:
                            callback(gimbal_data)
                        except Exception as e:
                            print(f"回调函数错误: {e}")

    def serial_reader(self):
        """串口读取线程"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )

            print(f"已连接到 {self.port}, 波特率 {self.baudrate}")
            print("等待数据...")

            while self.running:
                if self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting)
                    self.process_data(data)
                time.sleep(0.001)  # 短暂休眠

        except Exception as e:
            print(f"串口错误: {e}")
        finally:
            if self.ser and self.ser.is_open:
                self.ser.close()

    def start(self):
        """启动接收器"""
        if self.running:
            print("接收器已在运行")
            return

        self.running = True
        self.thread = threading.Thread(target=self.serial_reader)
        self.thread.daemon = True
        self.thread.start()
        print("接收器已启动")

    def stop(self):
        """停止接收器"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)
        if self.ser and self.ser.is_open:
            self.ser.close()
        print("接收器已停止")

    def get_stats(self):
        """获取统计信息"""
        return {
            'receive_count': self.receive_count,
            'error_count': self.error_count,
            'latest_data': self.latest_data,
            'data_history_count': len(self.data_history)
        }


# 数据处理器类
class DataProcessor:
    def __init__(self):
        self.start_time = time.time()
        self.packet_count = 0

    def print_realtime_data(self, data: GimbalData):
        """实时打印数据"""
        self.packet_count += 1
        current_time = time.time() - self.start_time

        # 清空当前行并打印新数据
        print(f"\r[{current_time:7.2f}s] "
              f"包:{self.packet_count:5d} | "
              f"模式:{data.mode} | "
              f"Yaw:{data.yaw:7.2f}° | "
              f"YawV:{data.yaw_vel:7.2f} | "
              f"Pitch:{data.pitch:7.2f}° | "
              f"PitchV:{data.pitch_vel:7.2f} | "
              f"子弹:{data.bullet_speed:.1f}m/s | "
              f"CRC:0x{data.crc16:04X}",
              end='', flush=True)

    def save_to_file(self, data: GimbalData):
        """保存数据到文件"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        with open('gimbal_data_log.csv', 'a', encoding='utf-8') as f:
            f.write(f"{timestamp},{data.mode},"
                    f"{data.q[0]:.6f},{data.q[1]:.6f},{data.q[2]:.6f},{data.q[3]:.6f},"
                    f"{data.yaw:.4f},{data.yaw_vel:.4f},{data.pitch:.4f},{data.pitch_vel:.4f},"
                    f"{data.bullet_speed:.2f},{data.bullet_count},{data.crc16}\n")

    def print_statistics(self, receiver: GimbalDataReceiver):
        """打印统计信息"""
        stats = receiver.get_stats()
        print(f"\n\n=== 统计信息 ===")
        print(f"接收数据包: {stats['receive_count']}")
        print(f"错误数据包: {stats['error_count']}")
        print(f"历史数据量: {stats['data_history_count']}")
        if stats['latest_data']:
            latest = stats['latest_data']
            print(f"最新数据时间: {datetime.fromtimestamp(latest.timestamp).strftime('%H:%M:%S.%f')[:-3]}")
        print(f"数据保存到: gimbal_data_log.csv")


def main():
    # 创建接收器 - 使用COM9（你的STM32端口）
    receiver = GimbalDataReceiver(port='COM9', baudrate=115200)

    # 创建数据处理器
    processor = DataProcessor()

    # 添加回调函数
    receiver.add_callback(processor.print_realtime_data)
    receiver.add_callback(processor.save_to_file)

    try:
        # 启动接收器
        receiver.start()

        # 主循环
        last_stat_time = time.time()
        while True:
            time.sleep(0.1)

            # 每10秒打印一次统计信息
            if time.time() - last_stat_time > 10:
                print()  # 换行
                processor.print_statistics(receiver)
                last_stat_time = time.time()

    except KeyboardInterrupt:
        print("\n\n正在停止...")
    finally:
        receiver.stop()
        processor.print_statistics(receiver)
        print("程序已退出")


if __name__ == "__main__":
    # 创建数据目录和文件头
    try:
        with open('gimbal_data_log.csv', 'w', encoding='utf-8') as f:
            f.write("时间戳,模式,q_w,q_x,q_y,q_z,yaw,yaw_vel,pitch,pitch_vel,bullet_speed,bullet_count,crc16\n")
        print("数据日志文件已创建: gimbal_data_log.csv")
    except Exception as e:
        print(f"创建日志文件失败: {e}")

    print("云台数据接收器")
    print("=" * 50)
    print("支持的串口:")

    # 列出可用串口
    try:
        import serial.tools.list_ports

        ports = serial.tools.list_ports.comports()
        for port in ports:
            print(f"  {port.device} - {port.description}")
    except:
        print("  无法获取串口列表")

    print(f"\n使用端口: COM9 (STM32)")
    print("按 Ctrl+C 退出程序")
    print("=" * 50)

    main()