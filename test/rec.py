#!/usr/bin/env python3
"""
Gimbal数据接收和可视化脚本
基于GimbalToVision结构体进行数据解析和实时绘图
"""

import serial
import struct
import time
import threading
import queue
import numpy as np
from collections import deque
from typing import NamedTuple, Optional
import argparse

# 检查NumPy版本并处理兼容性
try:
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
except ImportError as e:
    print(f"❌ matplotlib导入失败: {e}")
    print("💡 解决方案:")
    print("   1. 降级NumPy: pip install 'numpy<2'")
    print("   2. 升级matplotlib: pip install --upgrade matplotlib")
    print("   3. 重新安装: pip uninstall matplotlib numpy && pip install matplotlib numpy")
    exit(1)

class GimbalData(NamedTuple):
    """云台数据结构"""
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

class GimbalReceiver:
    """云台数据接收器"""
    
    def __init__(self, port: str, baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.serial_conn: Optional[serial.Serial] = None
        self.data_queue = queue.Queue()
        self.running = False
        self.receive_thread: Optional[threading.Thread] = None
        
        # 数据存储
        self.timestamps = deque(maxlen=1000)
        self.yaw_data = deque(maxlen=1000)
        self.pitch_data = deque(maxlen=1000)
        self.yaw_vel_data = deque(maxlen=1000)
        self.pitch_vel_data = deque(maxlen=1000)
        
    def connect(self) -> bool:
        """连接串口"""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.1,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            print(f"✓ 成功连接到串口: {self.port}")
            return True
        except Exception as e:
            print(f"✗ 串口连接失败: {e}")
            return False
    
    def disconnect(self):
        """断开串口连接"""
        self.running = False
        if self.receive_thread:
            self.receive_thread.join()
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("✓ 串口连接已断开")
    
    def find_frame_header(self, buffer: bytes) -> int:
        """在缓冲区中查找帧头 'SP'"""
        for i in range(len(buffer) - 1):
            if buffer[i] == ord('S') and buffer[i + 1] == ord('P'):
                return i
        return -1
    
    def calculate_crc16(self, data: bytes) -> int:
        """计算CRC16校验"""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc = crc >> 1
        return crc & 0xFFFF
    
    def unpack_gimbal_data(self, data_bytes: bytes) -> Optional[GimbalData]:
        """解包云台数据"""
        try:
            # 检查数据长度 (2字节帧头 + 1字节模式 + 16字节四元数 + 16字节运动数据 + 4字节子弹速度 + 2字节子弹计数 + 2字节CRC)
            expected_size = 43
            if len(data_bytes) != expected_size:
                return None
            
            # 解包数据 (小端序)
            # 格式: 2s B 4f 4f f H H (帧头, 模式, 四元数4个float, yaw+vel+pitch+vel 4个float, 子弹速度, 子弹计数, CRC16)
            unpacked = struct.unpack('<2s B 4f 4f f H H', data_bytes)
            
            # 验证帧头
            if unpacked[0] != b'SP':
                return None
            
            # 验证CRC16
            data_without_crc = data_bytes[:-2]
            calculated_crc = self.calculate_crc16(data_without_crc)
            received_crc = unpacked[12]
            
            if calculated_crc != received_crc:
                print(f"⚠️  CRC校验失败: 计算值={calculated_crc:04X}, 接收值={received_crc:04X}")
                return None
            
            return GimbalData(
                head=unpacked[0],
                mode=unpacked[1],
                q=unpacked[2:6],           # 四元数 (w, x, y, z)
                yaw=unpacked[6],            # yaw
                yaw_vel=unpacked[7],        # yaw_vel
                pitch=unpacked[8],          # pitch
                pitch_vel=unpacked[9],      # pitch_vel
                bullet_speed=unpacked[10],  # bullet_speed
                bullet_count=unpacked[11],   # bullet_count
                crc16=unpacked[12],         # crc16
                timestamp=time.time()
            )
            
        except Exception as e:
            print(f"✗ 数据解包失败: {e}")
            return None
    
    def receive_data(self):
        """接收数据线程"""
        buffer = b''
        frame_size = 43  # GimbalToVision结构体大小
        
        while self.running:
            try:
                if not self.serial_conn or not self.serial_conn.is_open:
                    time.sleep(0.1)
                    continue
                
                # 读取可用数据
                available = self.serial_conn.in_waiting
                if available > 0:
                    new_data = self.serial_conn.read(available)
                    buffer += new_data
                    
                    # 查找完整的数据帧
                    while len(buffer) >= frame_size:
                        # 查找帧头
                        header_pos = self.find_frame_header(buffer)
                        if header_pos == -1:
                            # 没找到帧头，清空缓冲区
                            buffer = b''
                            break
                        
                        # 移除帧头前的数据
                        if header_pos > 0:
                            buffer = buffer[header_pos:]
                        
                        # 检查是否有完整帧
                        if len(buffer) >= frame_size:
                            frame_data = buffer[:frame_size]
                            buffer = buffer[frame_size:]
                            
                            # 解包数据
                            gimbal_data = self.unpack_gimbal_data(frame_data)
                            if gimbal_data:
                                self.data_queue.put(gimbal_data)
                                
                                # 打印16进制数据
                                hex_data = frame_data.hex().upper()
                                print(f"📦 接收数据 (hex): {hex_data}")
                                
                                # 打印解包后的数据
                                self.print_gimbal_data(gimbal_data)
                        else:
                            break
                else:
                    time.sleep(0.001)
                    
            except Exception as e:
                print(f"✗ 接收数据错误: {e}")
                time.sleep(0.1)
    
    def print_gimbal_data(self, data: GimbalData):
        """打印解包数据"""
        mode_names = {0: "空闲", 1: "自瞄", 2: "小符", 3: "大符"}
        mode_name = mode_names.get(data.mode, "未知")
        
        print("=" * 50)
        print(f"🕐 时间戳: {data.timestamp:.3f}")
        print(f"📡 帧头: {data.head.decode('ascii')}")
        print(f"🎯 模式: {data.mode} ({mode_name})")
        print(f"🧭 四元数: [{data.q[0]:.6f}, {data.q[1]:.6f}, {data.q[2]:.6f}, {data.q[3]:.6f}]")
        print(f"↔️  Yaw: {data.yaw:.3f} rad ({np.degrees(data.yaw):.1f}°)")
        print(f"↕️  Pitch: {data.pitch:.3f} rad ({np.degrees(data.pitch):.1f}°)")
        print(f"🔄 Yaw速度: {data.yaw_vel:.3f} rad/s")
        print(f"🔄 Pitch速度: {data.pitch_vel:.3f} rad/s")
        print(f"💨 子弹速度: {data.bullet_speed:.2f} m/s")
        print(f"🔢 子弹计数: {data.bullet_count}")
        print(f"🔍 CRC16: 0x{data.crc16:04X}")
        print("=" * 50)
    
    def start_receiving(self):
        """开始接收数据"""
        if not self.connect():
            return False
        
        self.running = True
        self.receive_thread = threading.Thread(target=self.receive_data, daemon=True)
        self.receive_thread.start()
        print("✓ 开始接收云台数据...")
        return True
    
    def get_latest_data(self) -> Optional[GimbalData]:
        """获取最新数据"""
        try:
            return self.data_queue.get_nowait()
        except queue.Empty:
            return None

class GimbalVisualizer:
    """云台数据可视化器"""
    
    def __init__(self, receiver: GimbalReceiver):
        self.receiver = receiver
        self.fig, self.axes = plt.subplots(2, 2, figsize=(12, 8))
        self.fig.suptitle('Gimbal Data Visualization', fontsize=16)
        
        # 设置子图
        self.ax_yaw = self.axes[0, 0]
        self.ax_pitch = self.axes[0, 1]
        self.ax_yaw_vel = self.axes[1, 0]
        self.ax_pitch_vel = self.axes[1, 1]
        
        # 初始化绘图
        self.setup_plots()
        
    def setup_plots(self):
        """设置绘图"""
        # Yaw角度图
        self.ax_yaw.set_title('Yaw Angle')
        self.ax_yaw.set_ylabel('Yaw (rad)')
        self.ax_yaw.grid(True)
        self.line_yaw, = self.ax_yaw.plot([], [], 'b-', linewidth=2)
        
        # Pitch角度图
        self.ax_pitch.set_title('Pitch Angle')
        self.ax_pitch.set_ylabel('Pitch (rad)')
        self.ax_pitch.grid(True)
        self.line_pitch, = self.ax_pitch.plot([], [], 'r-', linewidth=2)
        
        # Yaw速度图
        self.ax_yaw_vel.set_title('Yaw Velocity')
        self.ax_yaw_vel.set_ylabel('Yaw Vel (rad/s)')
        self.ax_yaw_vel.set_xlabel('Time (s)')
        self.ax_yaw_vel.grid(True)
        self.line_yaw_vel, = self.ax_yaw_vel.plot([], [], 'g-', linewidth=2)
        
        # Pitch速度图
        self.ax_pitch_vel.set_title('Pitch Velocity')
        self.ax_pitch_vel.set_ylabel('Pitch Vel (rad/s)')
        self.ax_pitch_vel.set_xlabel('Time (s)')
        self.ax_pitch_vel.grid(True)
        self.line_pitch_vel, = self.ax_pitch_vel.plot([], [], 'm-', linewidth=2)
        
    def update_plot(self, frame):
        """更新绘图"""
        # 获取最新数据
        data = self.receiver.get_latest_data()
        if data:
            # 更新数据存储
            self.receiver.timestamps.append(data.timestamp)
            self.receiver.yaw_data.append(data.yaw)
            self.receiver.pitch_data.append(data.pitch)
            self.receiver.yaw_vel_data.append(data.yaw_vel)
            self.receiver.pitch_vel_data.append(data.pitch_vel)
            
            # 转换为numpy数组
            times = np.array(self.receiver.timestamps)
            yaw_data = np.array(self.receiver.yaw_data)
            pitch_data = np.array(self.receiver.pitch_data)
            yaw_vel_data = np.array(self.receiver.yaw_vel_data)
            pitch_vel_data = np.array(self.receiver.pitch_vel_data)
            
            # 计算相对时间
            if len(times) > 0:
                rel_times = times - times[0]
                
                # 更新线条数据
                self.line_yaw.set_data(rel_times, yaw_data)
                self.line_pitch.set_data(rel_times, pitch_data)
                self.line_yaw_vel.set_data(rel_times, yaw_vel_data)
                self.line_pitch_vel.set_data(rel_times, pitch_vel_data)
                
                # 自动调整坐标轴
                for ax in [self.ax_yaw, self.ax_pitch, self.ax_yaw_vel, self.ax_pitch_vel]:
                    ax.relim()
                    ax.autoscale_view()
        
        return [self.line_yaw, self.line_pitch, self.line_yaw_vel, self.line_pitch_vel]
    
    def start_visualization(self):
        """开始可视化"""
        # 设置动画
        self.animation = animation.FuncAnimation(
            self.fig, self.update_plot, interval=50, blit=False
        )
        
        # 显示图形
        plt.tight_layout()
        plt.show()

def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='Gimbal数据接收和可视化')
    parser.add_argument('--port', '-p', default='/dev/gimbal', 
                       help='串口设备路径 (默认: /dev/ttyUSB0)')
    parser.add_argument('--baudrate', '-b', type=int, default=115200,
                       help='波特率 (默认: 115200)')
    
    args = parser.parse_args()
    
    print("🚀 启动Gimbal数据接收和可视化程序")
    print(f"📡 串口: {args.port}")
    print(f"⚡ 波特率: {args.baudrate}")
    print("-" * 50)
    
    # 创建接收器
    receiver = GimbalReceiver(args.port, args.baudrate)
    
    try:
        # 开始接收数据
        if not receiver.start_receiving():
            return
        
        # 创建可视化器
        visualizer = GimbalVisualizer(receiver)
        
        # 开始可视化
        visualizer.start_visualization()
        
    except KeyboardInterrupt:
        print("\n⏹️  程序被用户中断")
    except Exception as e:
        print(f"✗ 程序错误: {e}")
    finally:
        # 清理资源
        receiver.disconnect()
        print("✓ 程序已退出")

if __name__ == "__main__":
    main()