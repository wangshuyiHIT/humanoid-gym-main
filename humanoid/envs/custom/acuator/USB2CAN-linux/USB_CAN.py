import numpy as np
import time
import struct
import serial
import os
os.system(" sudo chmod 777 /dev/ttyACM0")
from can_common import *
ser=serial.Serial("/dev/ttyACM0",921600)
if(ser.isOpen()):
    print("开启1号CDC设备成功")
def can_Baudrate():  #1Mbps
    send_data=np.array([0x55,0x05,0x00,0xAA,0x55],np.uint8)
    return send_data
def can_jump():  #1Mbps
    send_data=np.array([0x55,0x04,0xAA,0x55],np.uint8)
    return send_data
def usart_Baudrate():
    send_data=np.array([0x55,0xAA,0x00,0x00,0x00,0x00,0,0,0,0xAA,0x55],np.uint8)
    Baudrate=list(struct.pack("<i",921600))
    print(Baudrate)
    for i in range(4):
        send_data[2+i]=Baudrate[i]
    
    return send_data

# ser.write(bytes(usart_Baudrate().T))  
# count=ser.inWaiting()
# if count>0:
#     data=ser.read(16)
#     print(list(data))  
# ser.write(bytes(can_Baudrate().T))  
# count=ser.inWaiting()
# if count>0:
#     data=ser.read(16)
#     print(list(data))
# #ser.write(bytes(can_jump().T))  
MotorControl_Start(1)
# MotorControl_Start(3)
# MotorControl_Start(9)
# MotorControl_Start(12)
count=ser.inWaiting()
if count>0:
    data=ser.read(16*1)
    print(list(data))
while 1:
    s=0
    CanComm_SendControlPara(0,2,0,1,0,1)