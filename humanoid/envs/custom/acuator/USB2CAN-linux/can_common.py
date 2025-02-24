import numpy as np
from socket import *
import threading
import serial



ser=serial.Serial("/dev/ttyACM0",921600)
if(ser.isOpen()):
    print("开启1号CDC设备成功")

# HOST='192.168.4.101'
# port=8881
# port1=8882
# BUFSZIE=4*1024
# ADDR=(HOST,port)
# ADDR1=(HOST,port1)
# tcpSvrSock=socket(AF_INET,SOCK_STREAM)
# tcpSvrSock.connect(ADDR)


sendControl_flag=0
sendCommed_flag=0
data1=[]
CMD_MOTOR_MODE=0x01
CMD_RESET_MODE=0x02
CMD_ZERO_POSITION=0x03
CMD_START_MODE=0x04
send_counter=0
send_datas=np.zeros((54,1),np.uint8)
control_datas=np.zeros((58,1),np.uint8)
P_MIN=-95.5   
P_MAX= 95.5       
V_MIN=-45.0  
V_MAX= 45.0
KP_MIN=0.0    
KP_MAX=500.0
KD_MIN=0.0     
KD_MAX=5.0
T_MIN=-13.0
T_MAX=13.0
HOST='192.168.4.10'
HOST1='192.168.4.101'
port=8881
port1=8882
BUFSZIE=4*1024
ADDR_1=(HOST1,port)
ADDR_2=(HOST1,port1)
ADDR1=(HOST,8888)
listen_socket = socket(AF_INET, SOCK_DGRAM)
listen_socket.bind(ADDR1)
A_MIN=0.0
A_MAX=20.0 #额定电流为5A 堵转电流为20A   扭矩常数=0.28   扭矩=扭矩常数*A
motor_feedback=np.zeros((12,4))

s_flag=0



CAN1_buf=np.zeros((9,1),np.uint8)
CAN1_buf1=np.zeros((13,1),np.uint8)
CAN1_buf2=np.zeros((13,1),np.uint8)
CAN1_buf3=np.zeros((13,1),np.uint8)

CAN2_buf=np.zeros((9,1),np.uint8)
CAN2_buf1=np.zeros((13,1),np.uint8)
CAN2_buf2=np.zeros((13,1),np.uint8)
CAN2_buf3=np.zeros((13,1),np.uint8)

CAN1_buf1[0]=CAN1_buf2[0]=CAN1_buf3[0]=8
CAN1_buf1[1]=CAN1_buf2[1]=CAN1_buf3[1]=0
CAN1_buf1[2]=CAN1_buf2[2]=CAN1_buf3[2]=0
CAN1_buf1[3]=CAN1_buf2[3]=CAN1_buf3[3]=0


CAN2_buf1[0]=CAN2_buf2[0]=CAN2_buf3[0]=8
CAN2_buf1[1]=CAN2_buf2[1]=CAN2_buf3[1]=0
CAN2_buf1[2]=CAN2_buf2[2]=CAN2_buf3[2]=0
CAN2_buf1[3]=CAN2_buf2[3]=CAN2_buf3[3]=0
def GetModbusCRC16_Cal(data,len):
     temp=0
     wcrc=0xffff
     for i in range(len):
          temp=data[i]&0x00ff
          wcrc^=temp
          for j in range(8):
               if wcrc&0x0001:
                    wcrc>>=1
                    wcrc^=0xA001
               else:
                    wcrc>>=1
     return (wcrc<<8)|(wcrc>>8)



def float_to_uint(x,x_min, x_max,bits):
    span=x_max-x_min
    offset=x_min
    return np.uint16((x-offset)*((1<<bits)-1)/span)

def uint_to_float(x_int,x_min, x_max,bits):
    span=x_max-x_min
    offset=x_min
    return  np.float(x_int*span/((1<<bits)-1)+offset)
def LIMIT_MIN_MAX(x,min,max):
    if x<=min:
        x=min
    elif x>max:
        x=max

# def SPI_TransmitReceive(send_data):
   
#     if send_data[2]==1 or send_data[2]==2 or send_data[2]==3 or send_data[2]==10 or send_data[2]==11 or send_data[2]==12:
#         Chip_ID=1
#     elif send_data[2]==4 or send_data[2]==5 or send_data[2]==6 or send_data[2]==7 or send_data[2]==8 or send_data[2]==9:
#         Chip_ID=2
#     if Chip_ID==1:
#          wiringpi.digitalWrite(2,0)  
#          len1,receive1=wiringpi.wiringPiSPIDataRW(SPIchannel,bytes(send_data))
#          wiringpi.digitalWrite(2,1) 
#          receive_data1=list(receive1)
#          print(receive_data1)
#          if receive_data1[0]==89 and receive_data1[1]==88:
#             temp_value=(np.uint16(receive_data1[4])<<8)|np.uint16(receive_data1[5])
#             CurPosition=uint_to_float(temp_value, P_MIN, P_MAX, 16)
#             temp_value1=(np.uint16(receive_data1[6])<<4)|((receive_data1[7]&0xf0)>>4)
#             CurVelocity=uint_to_float(temp_value1, V_MIN, V_MAX, 12)
#             temp_value2=((np.uint16(receive_data1[7])&0x000f)<<8)|np.uint16(receive_data1[8])
#             CurTorque=0.28*uint_to_float(temp_value2, A_MIN, A_MAX, 12)
#             for i in range(1,13):
#                 if receive_data1[3]==i:
#                     motor_feedback[i-1,0]=receive_data1[3]
#                     motor_feedback[i-1,1]=CurPosition
#                     motor_feedback[i-1,2]=CurVelocity
#                     motor_feedback[i-1,3]=CurTorque
         
#     elif Chip_ID==2:
#          wiringpi.digitalWrite(3,0)  
#          len1,receive2=wiringpi.wiringPiSPIDataRW(SPIchannel,bytes(send_data))
#          wiringpi.digitalWrite(3,1) 
#          receive_data2=list(receive2)
#          if receive_data2[0]==89 and receive_data2[1]==88:
#             temp_value=(np.uint16(receive_data2[4])<<8)|np.uint16(receive_data2[5])
#             CurPosition=uint_to_float(temp_value, P_MIN, P_MAX, 16)
#             temp_value1=(np.uint16(receive_data2[6])<<4)|((receive_data2[7]&0xf0)>>4)
#             CurVelocity=uint_to_float(temp_value1, V_MIN, V_MAX, 12)
#             temp_value2=((np.uint16(receive_data2[7])&0x000f)<<8)|np.uint16(receive_data2[8])
#             CurTorque=0.28*uint_to_float(temp_value2, A_MIN, A_MAX, 12)
#             for i in range(1,13):
#                 if receive_data2[3]==i:
#                     motor_feedback[i-1,0]=receive_data2[3]
#                     motor_feedback[i-1,1]=CurPosition
#                     motor_feedback[i-1,2]=CurVelocity
#                     motor_feedback[i-1,3]=CurTorque
#     return  motor_feedback 

receive_data=np.zeros((6,8),np.uint8)
def reveice_process(data):
    global receive_data
    if(data[0]==100 and data[1]==99):
        receive_data[0]=data[2:10]
        receive_data[1]=data[10:18]
        receive_data[2]=data[18:26]
        receive_data[3]=data[26:34]
        receive_data[4]=data[34:42]
        receive_data[5]=data[42:50]
        for i in range(6):
            temp_value=(np.uint16(receive_data[i][1])<<8)|np.uint16(receive_data[i][2])
            CurPosition=uint_to_float(temp_value, P_MIN, P_MAX, 16)
            temp_value1=(np.uint16(receive_data[i][3])<<4)|((receive_data[i][4]&0xf0)>>4)
            CurVelocity=uint_to_float(temp_value1, V_MIN, V_MAX, 12)
            temp_value2=((np.uint16(receive_data[i][4])&0x000f)<<8)|np.uint16(receive_data[i][5])
            CurTorque=0.28*uint_to_float(temp_value2, A_MIN, A_MAX, 12)
            motor_feedback[receive_data[i][0]-1,0]=receive_data[i][0]
            motor_feedback[receive_data[i][0]-1,1]=CurPosition
            motor_feedback[receive_data[i][0]-1,2]=CurVelocity
            motor_feedback[receive_data[i][0]-1,3]=CurTorque   
        #print(motor_feedback)         
       
# 数据头  数据头    motor_ID   Pd Vd  Kp Kd Tq(8个字节)  CRC_H  CRC_L

#1号腿  motor_ID 1 2 3     Slave_ID 20 21 22    1号芯片控制    1号CAN口    can1   大腿  小腿  侧摆
#2号腿  motor_ID 4 5 6     Slave_ID 23 24 25    2号芯片控制    2号CAN口    can2
#3号腿  motor_ID 7 8 9     Slave_ID 26 27 28    2号芯片控制    1号CAN口    can3
#4号腿  motor_ID 10 11 12  Slave_ID 29 30 31    1号芯片控制    2号CAN口    can4
buf=np.zeros((9,1),np.uint8)
buf1=np.zeros((11,1),np.uint8)
def CanComm_SendControlPara(f_p,f_v,f_kp,f_kd,f_t,GM8115_ID):
    global sendControl_flag,pre_time,s_flag,CAN1_buf1
    send_data=np.array([0x55,0xAA,0x1e,0x01,0x01,0x00,0x00,0x00,0x0a,0x00,0x00,0x00,0x00,0,0,0,0,0x00,0x08,0x00,0x00,0,0,0,0,0,0,0,0,0x88],np.uint8)

    LIMIT_MIN_MAX(f_p,  P_MIN,  P_MAX)
    LIMIT_MIN_MAX(f_v,  V_MIN,  V_MAX)
    LIMIT_MIN_MAX(f_kp, KP_MIN, KP_MAX)
    LIMIT_MIN_MAX(f_kd, KD_MIN, KD_MAX)
    LIMIT_MIN_MAX(f_t,  T_MIN,  T_MAX)
    p = float_to_uint(f_p,      P_MIN,  P_MAX,  16)            
    v = float_to_uint(f_v,      V_MIN,  V_MAX,  12)
    kp = float_to_uint(f_kp,    KP_MIN, KP_MAX, 12)
    kd = float_to_uint(f_kd,    KD_MIN, KD_MAX, 12)
    t = float_to_uint(f_t,      T_MIN,  T_MAX,  12)
    buf[0]=GM8115_ID
    buf[1] = p>>8
    buf[2] = p&0xFF
    buf[3] = v>>4
    buf[4] = ((v&0xF)<<4)|(kp>>8)
    buf[5] = kp&0xFF
    buf[6] = kd>>4
    buf[7] = ((kd&0xF)<<4)|(t>>8)
    buf[8] = t&0xff
    send_data[13]=GM8115_ID
    send_data[21:29]=buf[1:9,0]
    print(send_data)
    ser.write(bytes(send_data.T))
    # CAN1_buf1[4:13]=buf
    # #CAN1_buf2[4:13,0]=buf
    # listen_socket.sendto(bytes(CAN1_buf1.T),ADDR_1)
   # listen_socket.sendto(bytes(CAN1_buf2.T),ADDR_2)
    # buf1[0]=77
    # buf1[1]=67
    # for i in range(2,11):
    #     buf1[i]=buf[i-2]
    # SPI_TransmitReceive(buf1)    
    # s_flag=s_flag+1
    # if(wiringpi.millis() -pre_time>1000):
    #    counts=s_flag
    #    print("Linux端USB总线1s接收次数为:%d\r\n"%counts)
    #    s_flag=0
    #    pre_time=wiringpi.millis()    
    
    #motor_feedbacks=np.zeros((12,4))
    #motor_feedbacks=SPI_TransmitReceive(bytes(buf))
    
    #for z in range(1,13):
        #print("第%d个电机反馈的位置、速度、扭矩为: %10.3f %10.3f %10.3f\n"%(z,motor_feedbacks[z-1,1],motor_feedbacks[z-1,2],motor_feedbacks[z-1,3]))


def CanComm_ControlCmd(cmd,GM8115_ID):
    global CAN1_buf1,CAN1_buf2
    global sendCommed_flag
    send_data=np.array([0x55,0xAA,0x1e,0x01,0x01,0x00,0x00,0x00,0x0a,0x00,0x00,0x00,0x00,0,0,0,0,0x00,0x08,0x00,0x00,0,0,0,0,0,0,0,0,0x88],np.uint8)
    buf=np.array([GM8115_ID,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0x00],np.uint8)
    if cmd==CMD_MOTOR_MODE:
        buf[8]=0xFC
    elif cmd==CMD_RESET_MODE:
        buf[8]=0xFD
    elif cmd==CMD_ZERO_POSITION:
        buf[8]=0xFE
    else:
        print("等待电机启动命令")
    send_data[13]=GM8115_ID
    send_data[21:29]=buf[1:9]
    print(send_data)
    ser.write(bytes(send_data.T))
    # CAN1_buf1[4:13,0]=buf
    # CAN1_buf1[4:13,0]=buf
    # #CAN1_buf2[4:13,0]=buf
    # listen_socket.sendto(bytes(CAN1_buf1.T),ADDR_1)
    #listen_socket.sendto(bytes(CAN1_buf2.T),ADDR_2)
    #print(buf1)
    #SPI_TransmitReceive(buf1)    
   
    
    #SPI_TransmitReceive(bytes(buf))
  

def  ZeroPosition(GM8115_ID):
     CanComm_ControlCmd(CMD_MOTOR_MODE,GM8115_ID)
     CanComm_SendControlPara(0,0,0,0,0,GM8115_ID)

def  MotorControl_Start( GM8115_ID):
    ZeroPosition(GM8115_ID)
    CanComm_ControlCmd(CMD_ZERO_POSITION,GM8115_ID)

def MotorControl_Stop(GM8115_ID):
    CanComm_ControlCmd(CMD_RESET_MODE,GM8115_ID)

def MotorControl_startSend(GM8115_ID):
    motor_commed=np.zeros((10,1),np.uint8)
    motor_commed[0]=88      
    motor_commed[1]=87
    for i in range(2,8):
        motor_commed[i]=GM8115_ID[i-2]
    S_CRC_result=GetModbusCRC16_Cal(motor_commed,8)
    motor_commed[8]=(S_CRC_result&0xff00)>>8
    motor_commed[9]=(S_CRC_result&0x00ff)
    
    #print(data1)
def MotorControl_stopSend(GM8115_ID):
    motor_commedStop=np.zeros((10,1),np.uint8)
    motor_commedStop[0]=78      
    motor_commedStop[1]=77
    for i in range(2,8):
        motor_commedStop[i]=GM8115_ID[i-2]
    S_CRC_result=GetModbusCRC16_Cal(motor_commedStop,8)
    motor_commedStop[8]=(S_CRC_result&0xff00)>>8
    motor_commedStop[9]=(S_CRC_result&0x00ff)
  
    #print(data1)
    
