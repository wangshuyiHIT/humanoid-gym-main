import math
import time
import numpy as np
import serial
import gc
import os
import threading
import queue
from collections import deque

from isaacgym import gymapi

import torch

from humanoid.envs.custom.acuator.imu.imu import IMU
from humanoid.envs.custom.wsybot_config import wsybotCfg as Cfg

# 定义常量
DATASIZE = 44
StartByte = 0xAB
EndByte = 0xBA

def deploy(policy):
    pass # function body is omitted

def execute(target_pos):
    pass # function body is omitted

def uint16_to_uint8(uint_16_data):
    pass # function body is omitted

class cmd:
    ...

#[wsy] imu decoding
def quaternion_to_euler(quat):
    # 计算偏航角（yaw）
    pass # function body is omitted

def decode_motor_data(data, p_min, p_max, v_min, v_max, bits):
    # 检查数据长度是否正确（假设数据长度固定为50字节，包括帧头和帧尾）
    pass # function body is omitted

#uint to float
def uint_to_float(x_int, x_min, x_max, bits):
    pass # function body is omitted
#float to uint
def float_to_uint(x_float, x_min, x_max, bits):
    pass # function body is omitted
#print to test
def print_array_data(data_array):
    # 定义数据字段的名称
    pass # function body is omitted

#realease gc
# gc.set_threshold(100000, 100, 100)
os.system('sudo chmod a+rw /dev/ttyUSB0')
imu_port = "/dev/ttyUSB0"
imu_baudrate = 115200
imu = IMU()
os.system('sudo chmod a+rw /dev/ttyACM0')
control_port = "/dev/ttyACM0"
contro_baudrate = 115200
ser = serial.Serial("/dev/ttyACM0", 115200, timeout=0)
ser.timeout = 0.005
ser.reset_input_buffer()  # 清空输入缓冲区

policy_file_path = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "policy_1.pt",
)

policy = torch.jit.load(policy_file_path, map_location=torch.device('cpu'))

# 创建队列用于存储 IMU 数据和电机数据
imu_queue = queue.Queue(maxsize=1)
motor_queue = queue.Queue(maxsize=1)

def read_imu_data():
    pass # function body is omitted

def control_motor():
    pass # function body is omitted

# 创建并启动线程
imu_thread = threading.Thread(target=read_imu_data)
motor_thread = threading.Thread(target=control_motor)

imu_thread.start()
motor_thread.start()

while True:
    start_time = time.time()
    
    # 非阻塞方式获取数据
    quat, gvec = None, None
    p_float, v_float = None, None
    
    try:
        quat, gvec = imu_queue.get_nowait()
    except queue.Empty:
        continue  # 若队列为空，跳过本次循环

    try:
        p_float, v_float = motor_queue.get_nowait()
    except queue.Empty:
        continue  # 若队列为空，跳过本次循环
    
    flag_time = time.time()
    if flag_time - start_time < 0.01:
        pass
    else:
        continue
    
    hist_obs = deque()
    for _ in range(15):
        hist_obs.append(np.zeros([1, 47], dtype=np.double))

    action = np.zeros((12), dtype=np.double)
    
    count_lowlevel = 0
    # 1000hz -> 100hz--->1000/10(decimention)=100
    if count_lowlevel % 10 == 0:
        obs = np.zeros([1, 47], dtype=np.float32)
        pitch, roll, yaw = quaternion_to_euler(quat)
        eu_ang = np.array([pitch, roll, yaw])
        eu_ang[eu_ang > math.pi] -= 2 * math.pi

        obs[0, 0] = math.sin(2 * math.pi * count_lowlevel * 0.01  / 0.64)
        obs[0, 1] = math.cos(2 * math.pi * count_lowlevel * 0.01  / 0.64)
        obs[0, 2] = cmd.vx * 2
        obs[0, 3] = cmd.vy * 2
        obs[0, 4] = cmd.dyaw * 1
        obs[0, 5:17] = np.array(p_float) * 1
        obs[0, 17:29] = np.array(v_float) * 0.05
        obs[0, 29:41] = action
        obs[0, 41:44] = gvec
        obs[0, 44:47] = eu_ang

        obs = np.clip(obs, -18, 18)

        hist_obs.append(obs)
        hist_obs.popleft()
        
        policy_input = np.zeros([1, 705], dtype=np.float32)
        for i in range(15):
            policy_input[0, i * 47 : (i + 1) * 47] = hist_obs[i][0, :]
            action[:] = policy(torch.tensor(policy_input))[0].detach().numpy()
            action = np.clip(action, -18, 18)

            target_q = action * 0.25

        count_lowlevel += 1
    
    execte_end_time = time.time()
    
    time.sleep(0.001)
    
    print_array_data(p_float)
    execute_time = execte_end_time - start_time
    if execute_time < 0.0189:
        time.sleep(0.0189 - execute_time)
