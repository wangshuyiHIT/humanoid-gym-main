import math
import time
import numpy as np
import serial
import os
import time
import struct
#====
import threading
import queue
#====
from isaacgym import gymapi
import torch
from tqdm import tqdm
from collections import deque

from humanoid.envs.custom.acuator.imu.imu import IMU
from humanoid.envs.custom.wsybot_config import wsybotCfg as Cfg
# 定义常量
DATASIZE = 44
StartByte = 0xAB
EndByte = 0xBA
#pos limit order=MOTOR 1     2     3     4     5     6     7     8     9     A     B     C
motor_lower_limit  = [-0.15,-1.0, -1.0, -0.4, -1.0, -0.5, -0.7 ,-1.2, -1.0, -1.5, -1.0, -0.5]
motor_higher_limit = [ 0.7,  1.2,  1.0,  1.5,  1.0,  0.5,  0.15, 1.0,  1.0,  0.4,  1.0,  0.5]

def decode_motor_data(data, p_min, p_max, v_min, v_max, bits):
    # 检查数据长度是否正确（假设数据长度固定为50字节，包括帧头和帧尾）
    if len(data) != 50:
        raise ValueError("Data length is incorrect")
    # 验证帧头和帧尾
    if data[0] != 0xAB or data[-1] != 0xBA:
        raise ValueError("Frame header or footer is incorrect")
    # 初始化存储p_float和v_float的列表
    p_float_list = []
    v_float_list = []
    # 解码p_int值并转换为p_float
    for i in range(12):  # 
        # 计算p_int的字节索引（跳过帧头，每个p_int占2个字节）
        byte_index = 1 + 2 * i
        # 提取p_int的高字节和低字节，并组合成一个整数
        p_int = (data[byte_index] << 8) + data[byte_index + 1]
        # 使用uint_to_float函数将p_int转换为p_float
        p_float = uint_to_float(p_int, p_min, p_max, bits)
        p_float_list.append(p_float)
    # 解码v_int值并转换为v_float（与p_int类似）
    for i in range(12):  # 
        # 计算v_int的字节索引（跳过帧头和p_int数据，每个v_int占2个字节）
        byte_index = 25 + 2 * i  # 25是p_int数据结束后的起始索引
        # 提取v_int的高字节和低字节，并组合成一个整数
        v_int = (data[byte_index] << 8) + data[byte_index + 1]
        # 使用uint_to_float函数将v_int转换为v_float（注意这里仍然使用uint_to_float，只是参数不同）
        v_float = uint_to_float(v_int, v_min, v_max, bits)
        v_float_list.append(v_float)
    
    return p_float_list, v_float_list

#uint to float
def uint_to_float(x_int, x_min, x_max, bits):
    span = x_max - x_min
    offset = x_min
    return (x_int * span) / ((1 << bits) - 1) + offset
#float to uint
def float_to_uint(x_float, x_min, x_max, bits):
    span = x_max - x_min
    offset = x_min
    return int((x_float - offset) * ((1 << bits) - 1) / span)
#print to test
def print_array_data(data_array):
    # 定义数据字段的名称
    data_labels = [
        "Motor_Pos  1", "Motor_Pos  2", "Motor_Pos  3", "Motor_Pos  4",
        "Motor_Pos  5", "Motor_Pos  6", "Motor_Pos  7", "Motor_Pos  8",
        "Motor_Pos  9", "Motor_Pos 10", "Motor_Pos 11", "Motor_Pos 12"
    ]
    # 检查输入数组的长度是否正确
    if len(data_array) != len(data_labels):
        raise ValueError(f"Expected {len(data_labels)} elements in the data array, but got {len(data_array)} elements.")
    # 打印数据
    for label, value in zip(data_labels, data_array):
        print(f"{label:<12}: {value:<9.4f}")

os.system('sudo chmod a+rw /dev/ttyACM0')
control_port = "/dev/ttyACM0"
contro_baudrate = 115200
ser=serial.Serial("/dev/ttyACM0",115200, timeout=0)
ser.timeout = 0.005
ser.reset_input_buffer()  # 清空输入缓冲区

motor_queue = queue.Queue(maxsize=1)

def control_motor():
    global motor_queue, ser
    while True:
        try:
            buffer = ser.read(50)
            if len(buffer) == 50:
                p_float, v_float = decode_motor_data(buffer, -12.5, 12.5, -30.0, 30.0, 16)
                if motor_queue.full():
                    # 如果队列满了，直接丢弃旧数据
                    motor_queue.get()
                motor_queue.put((p_float, v_float))
        except Exception as e:
            print(f"Error reading motor data: {e}")
            # time.sleep(0.001)  # 避免频繁报错
        time.sleep(0.001)

motor_thread = threading.Thread(target=control_motor)

def execute(target_q):
    packed_target_q = struct.pack('<12H', *target_q)
    packed_data = bytes([StartByte]) + packed_target_q + bytes([EndByte])
    if not ser.is_open:
        ser.open
    ser.write(packed_data)
#====execution end
target_q_int16 = [0] * 12
target_q = [0.]* 12
for i in range(12):
    target_q_int16[i] = float_to_uint(target_q[i], -12.5, 12.5, 16)
execute(target_q_int16)
time.sleep(2)

count_lowlevel = 0

while True:
    start_time = time.time()

    try:
        p_float, v_float = motor_queue.get_nowait()
    except queue.Empty:
        continue  # 若队列为空，跳过本次循环
    #====
    # time.sleep(0.001)
    flag_time = time.time()
    if flag_time - start_time < 0.015:
        pass
    else:
        continue

    time.sleep(0.002)

    # # MotorOrder:1    2    3    4    5    6    7    8    9    A    B    C
    # target_q =  [0.0, 0.1, 0.0, 0.2, 0.1, 0.0, 0.0,-0.1, 0.0,-0.2,-0.1, 0.0]
    if count_lowlevel < 1000:
        target_q[1] += 0.001
        target_q[3] += 0.002
        target_q[4] += 0.001
        
        target_q[7] -= 0.001
        target_q[9] -= 0.002
        target_q[10]-= 0.001
    target_q = np.clip(target_q, motor_lower_limit, motor_higher_limit)
    
    for i in range(12):
        target_q_int16[i] = float_to_uint(target_q[i], -12.5, 12.5, 16)
    execute(target_q_int16)
        
    count_lowlevel += 1
    execte_end_time = time.time()

    execute_time = execte_end_time - start_time

    if execute_time < 0.0197:
        time.sleep(0.0197 - execute_time)
    else :
        pass
    end_time = time.time()
    print("  SingleFilm:",end_time - start_time)
    