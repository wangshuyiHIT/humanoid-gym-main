%% ���� IMU matlab ���ճ���
%% ��� www.hipnuc.com

clear;
clc;
close all;
format short;

global CH_HDR_SIZE;
global MAXRAWLEN;

CH_HDR_SIZE = 6;                % ֡ͷ��С
MAXRAWLEN = 512;                % ���buff size


imu_tempate.id = 0;                 % user defined ID
imu_tempate.acc = [0 0 0];          % acceleration
imu_tempate.gyr = [0 0 0];          %  angular velocity
imu_tempate.mag = [0 0 0];          % magnetic field
imu_tempate.eul = [0 0 0];          % attitude: eular angle
imu_tempate.quat = [0 0 0 0];       % attitude: quaternion
imu_tempate.pressure = 0;     	    %  air pressure
imu_tempate.timestamp = 0;


global raw;
raw.nbyte = 0;                            %   /* number of bytes in message buffer */
raw.len = 0;                                % /* message length (bytes) */
raw.imu= imu_tempate;             %   /* imu data list, if (HI226/HI229/CH100/CH110, use imu[0]) */
raw.buf = [0 0];


%% Ĭ������, ��Ҫ����PC�ʹ������õĲ������޸�
BAUD = 115200;
PORT = 'COM4';

%% ����ѡ��
if length(serialportlist) >=1 %���ֶ������
    fprintf("���ô���:%s\n", serialportlist);
    fprintf("��ѡ�񴮿ڣ����ĳ����е�PORT����\n");
end

if length(serialportlist) == 1 %ֻ��һ������
    PORT = serialportlist;
end

if isempty(serialportlist) == true %û�д���
    fprintf("�޿��ô���\n");
end

%% ��ʾ
fprintf('��ʹ��matlab 2020b �����ϰ汾!!!\n');
fprintf('���� clear s  ���� ctrl+c ������ֹ���ڴ��䲢�˳�ѭ��\n');
fprintf('��ǰѡ����%s, ������:%d\n', PORT, BAUD);
x = input("���س�����...\n");

%% �򿪴���
s = serialport(PORT, BAUD); %��������
%configureCallback(s,"byte",100,@callbackFcn)  %�����¼��ص�����

while true
    if  s.NumBytesAvailable > 0
        data = read(s, s.NumBytesAvailable,"uint8"); %��ȡ����������
        
        for ii = 1: length(data)
            
            [new_data_rdy]  =  ch_serial_input(data(ii));
            if new_data_rdy == 1
                fprintf("���ٶ�:%.3f %.3f %.3f\n", raw.imu.acc);
                fprintf("���ٶ�:%.3f %.3f %.3f\n",  raw.imu.gyr);
                fprintf("ŷ����: Roll:%.2f Pitch:%.2f Yaw:%.2f\n", raw.imu.eul(1), raw.imu.eul(2), raw.imu.eul(3));
                new_data_rdy = 0;
            end
        end
        pause(0.01);
    end
end



%% ͬ��֡ͷ�� 1:ͬ��  0��δͬ��
function ret = sync_ch(data)
global raw;
raw.buf(1) = raw.buf(2);
raw.buf(2) = data;
if (raw.buf(1) == 0x5A && raw.buf(2) == 0xA5);  ret = 1; else; ret = 0; end
end

%% ��֡
function new_data_rdy = decode_ch()
global raw;
new_data_rdy = 0;

crc1 = raw.buf(5) + raw.buf(6)*256;
crc_text = raw.buf;
crc_text(5:6) = [];

%����CRC У��ɹ�����ý������ݺ���
crc2 = crc16(double(crc_text));

if crc1 == crc2
    parse_data();
    new_data_rdy = 1;
else
    fprintf("CRC err\n");
end

end

%% ����������dataΪһ��uint8��������
function  [new_data_rdy] = ch_serial_input(data)

global raw;
global CH_HDR_SIZE;
global MAXRAWLEN;

new_data_rdy = 0;

if (raw.nbyte == 0)
    if(sync_ch(data) == 0);  return;      end
    
    raw.nbyte = 3;
    return;
end

raw.buf(raw.nbyte) = data;
raw.nbyte = raw.nbyte + 1;

if (raw.nbyte == CH_HDR_SIZE)
    raw.len = raw.buf(3) + raw.buf(4)*256;
    if(raw.len > (MAXRAWLEN - CH_HDR_SIZE));   fprintf("ch length error: len=%d\n",raw.len); raw.nbyte = 0;  return; end
end

if raw.nbyte < (raw.len + CH_HDR_SIZE+1);    return; end;
raw.nbyte  = 0;
new_data_rdy = decode_ch();

end

%% ����֡��������
function parse_data()
global raw;

data = raw.buf;
data(1:6) = [];
len = length(data); %�����򳤶�


offset = 1;
while offset < len
    byte = data(offset);
    switch byte
        case 0x90 % ID��ǩ
            raw.imu.id = data(offset+1);
            offset = offset + 2;
        case 0xA0 %���ٶ�
            tmp = typecast(uint8(data(offset+1:offset+6)), 'int16');
            raw.imu.acc = double(tmp) / 1000;
            offset = offset + 7;
        case 0xB0 %���ٶ�
            tmp = typecast(uint8(data(offset+1:offset+6)), 'int16');
            raw.imu.gyr = double(tmp) / 10;
            offset = offset + 7;
        case 0xC0 %�ش�
            tmp = typecast(uint8(data(offset+1:offset+6)), 'int16');
            raw.imu.mag = double(tmp) / 10;
            offset = offset + 7;
        case 0xD0 %ŷ����
            tmp = typecast(uint8(data(offset+1:offset+6)), 'int16');
            raw.imu.eul(1) = double(tmp(1)) / 100;
            raw.imu.eul(2) = double(tmp(2)) / 100;
            raw.imu.eul(3) = double(tmp(3)) / 10;
            offset = offset + 7;
        case 0xF0 % ��ѹ
            offset = offset + 5;
        case 0x91 % 0x91���ݰ�, ��λ����û��ֲ�
            raw.imu.id = data(offset+1);
            raw.imu.acc = double(typecast(uint8(data(offset+12:offset+23)), 'single'));
            raw.imu.gyr = double(typecast(uint8(data(offset+24:offset+35)), 'single'));
            raw.imu.mag = double(typecast(uint8(data(offset+36:offset+47)), 'single'));
            raw.imu.eul = double(typecast(uint8(data(offset+48:offset+59)), 'single'));
            raw.imu.quat = double(typecast(uint8(data(offset+60:offset+75)), 'single'));
            offset = offset + 76;
        otherwise
            offset = offset + 1;
    end
end

end




% data = "5A A5 4C 00 6C 51 91 00 A0 3B 01 A8 02 97 BD BB 04 00 9C A0 65 3E A2 26 45 3F 5C E7 30 3F E2 D4 5A C2 E5 9D A0 C1 EB 23 EE C2 78 77 99 41 AB AA D1 C1 AB 2A 0A C2 8D E1 42 42 8F 1D A8 C1 1E 0C 36 C2 E6 E5 5A 3F C1 94 9E 3E B8 C0 9E BE BE DF 8D BE";
% data = sscanf(data,'%2x');
%
%
% for ii = 1: length(data)
%
%     [new_data_rdy]  =  ch_serial_input(data(ii));
%     if new_data_rdy == 1
%         fprintf("���ٶ�:%.3f %.3f %.3f\n", raw.imu.acc);
%           fprintf("���ٶ�:%.3f %.3f %.3f\n",  raw.imu.gyr);
%         fprintf("ŷ����: Roll:%.2f Pitch:%.2f Yaw:%.2f\n", raw.imu.eul(1), raw.imu.eul(2), raw.imu.eul(3));
%         new_data_rdy = 0;
%
%     end
% end

