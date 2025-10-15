#!/usr/bin/env python3
import serial
import time

# 配置串口
ser = serial.Serial(
    '/dev/gimbal',
    115200,
    timeout=0.1
)

print("开始接收数据，按Ctrl+C停止")
print("=" * 50)

try:
    while True:
        if ser.in_waiting > 0:
            # 读取所有可用数据
            data = ser.read(ser.in_waiting)
            
            # 转换为16进制字符串
            hex_data = data.hex()
            
            # 输出16进制数据
            print(f"接收 {len(data)} 字节: {hex_data}")
            
        time.sleep(0.01)
        
except KeyboardInterrupt:
    print("\n停止接收")
finally:
    ser.close()
