import time
import serial
import numpy as np
from .python.parsers.hipnuc_serial_parser import hipnuc_parser

class IMU:
    def __init__(self):
        self.quat_data = None
        self.gvec_data = None

    def _parse_hipnuc_frame(self, frame):
        """
        解析单个hipnuc帧，提取四元数和陀螺仪数据
        """
        if frame.frame_type is not None:
            # 提取并转换四元数数据
            quat_data = np.array(frame.quat, dtype=np.double)
            self.quat_data = quat_data

            # 提取并转换陀螺仪数据（单位：弧度）
            gvec_data = np.array(frame.gyr, dtype=np.double) * (np.pi / 180)
            self.gvec_data = gvec_data

    def cmd_read(self, port, baudrate):
        """
        从串口读取IMU数据
        """
        serial_parser = hipnuc_parser()
        latest_hipnuc_frame = None
        read_count = 0  # 记录已读取的次数
        max_read_count = 1  # 设置最大读取次数

        try:
            with serial.Serial(port, int(baudrate), timeout=1) as ser:
                while read_count < max_read_count:
                    if ser.in_waiting:
                        data = ser.read(ser.in_waiting)
                        hipnuc_frames = serial_parser.parse(data)
                        if hipnuc_frames:
                            latest_hipnuc_frame = hipnuc_frames[-1]
                            self._parse_hipnuc_frame(latest_hipnuc_frame)
                            read_count += 1
        except serial.SerialException as e:
            print(f"Serial port error: {e}")
        except Exception as e:
            print(f"Unexpected error: {e}")

        return self.quat_data, self.gvec_data