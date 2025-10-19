#!/usr/bin/env python3
import serial
import time
import re

ser = serial.Serial(
    '/dev/ttyACM0',
    115200,
    timeout=0.1,  # 短超时
    write_timeout=0.1,
    inter_byte_timeout=0.1
)

line_count = 0
max_lines = 1000  # 限制最大行数
debug_counter = 0

# 调试信息解析正则表达式
debug_pattern = re.compile(r'Debug: Counter=(\d+), Yaw=([\d.-]+), Pitch=([\d.-]+)')

try:
    buffer = ""
    while line_count < max_lines:
        if ser.in_waiting:
            data = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
            buffer += data
            
            # 按行处理
            while '\n' in buffer:
                line, buffer = buffer.split('\n', 1)
                line = line.strip()
                
                # 解析调试信息
                match = debug_pattern.match(line)
                if match:
                    counter = int(match.group(1))
                    yaw = float(match.group(2))
                    pitch = float(match.group(3))
                    print(f"调试信息 #{counter}: Yaw={yaw}, Pitch={pitch}")
                    debug_counter = counter
                else:
                    print(f"{line_count}: {line}")
                
                line_count += 1
                
        time.sleep(0.01)  # 避免CPU占用过高
        
except KeyboardInterrupt:
    print(f"\n共接收 {line_count} 行数据，其中 {debug_counter} 条调试信息")
finally:
    ser.close()
