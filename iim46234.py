import serial
import signal
import struct
import math
import time
import sys
from typing import Optional
import logging
from typing import List
import psutil
import os
from datetime import datetime
import time

# Constants
BYTE_HEADER_REP = 0x23
SIZE_PACKET_FULL_DATA = 46
BYTE_HEADER_CMD = 0x24
BYTE_HEADER_REP = 0x23
BYTE_RESERVED = 0x00
BYTE_FOOTER_1 = 0x0D
BYTE_FOOTER_2 = 0x0A
BYTE_PADDING = 0x00

CMD_TYPE_GET_VERSION = 0x20
CMD_TYPE_GET_SERIAL_NUM = 0x26
CMD_TYPE_READ_REG = 0x11
CMD_TYPE_WRITE_REG = 0x12
CMD_TYPE_SELF_TEST = 0x2B
CMD_TYPE_SET_UTC_TIME = 0x2D
CMD_TYPE_SELECT_INTF = 0x30
CMD_TYPE_START_STREAMING = 0x27
CMD_TYPE_STOP_STREAMING = 0x28

SIZE_CMD_COMMON = 8
SIZE_CMD_SET_UTC_TIME = 15
SIZE_CMD_SELECT_INTF = 9
SIZE_CMD_READ_REGS = 12
SIZE_CMD_WRITE_REGS_BASE = 12

SIZE_RESP_ACK = 10
SIZE_RESP_GET_SERIAL_NUM = 26
SIZE_RESP_GET_VERSION = 20
SIZE_RESP_IMU_SELF_TEST = 16
SIZE_RESP_READ_REGS_BASE = 16

SIZE_PACKET_CMD = 20
SIZE_PACKET_FULL_DATA = 46
SIZE_BUFF_RESP = 26

SELF_TEST_RESULT_PASS = 0x03
ERR_CODE_SUCCESS = 0x00

BIT_SELECT_OUT_DATA_VEL = 0x10
BIT_SELECT_OUT_DATA_ANG = 0x08
BIT_SELECT_OUT_DATA_TEMP = 0x04
BIT_SELECT_OUT_DATA_GYRO = 0x02
BIT_SELECT_OUT_DATA_ACC = 0x01

BIT_SAVE_ALL_CONFIG_CMD = 0x50
BIT_SAVE_ALL_CONFIG_RESULT_IN_PROGRESS = 0x00
BIT_SAVE_ALL_CONFIG_RESULT_SUCCESS = 0x01
BIT_SAVE_ALL_CONFIG_RESULT_NOT_SAVED = 0x02

ACC_FSR_16G = 0x00
ACC_FSR_8G = 0x20
ACC_FSR_4G = 0x40
ACC_FSR_2G = 0x60
GYRO_FSR_2000DPS = 0x00
GYRO_FSR_1000DPS = 0x20
GYRO_FSR_500DPS = 0x40
GYRO_FSR_480DPS = 0x40
GYRO_FSR_250DPS = 0x60
ACC_LPF_BW4 = 0x40
ACC_LPF_BW5 = 0x50
ACC_LPF_BW6 = 0x60
ACC_LPF_BW7 = 0x70
GYRO_LPF_BW4 = 0x4
GYRO_LPF_BW5 = 0x5
GYRO_LPF_BW6 = 0x6
GYRO_LPF_BW7 = 0x7

dict_acc_fsr = {
    ACC_FSR_16G: '16g',
    ACC_FSR_8G: '8g',
    ACC_FSR_4G: '4g',
    ACC_FSR_2G: '2g'
}
dict_gyr_fsr = {
    GYRO_FSR_2000DPS: '2000dps',
    GYRO_FSR_1000DPS: '1000dps',
    GYRO_FSR_500DPS: '500dps',
    GYRO_FSR_480DPS: '480dps',
    GYRO_FSR_250DPS: '250dps'
}
dict_acc_bw = {
    ACC_LPF_BW4: 'BW4',
    ACC_LPF_BW5: 'BW5',
    ACC_LPF_BW6: 'BW6',
    ACC_LPF_BW7: 'BW7'
}
dict_gyr_bw = {
    GYRO_LPF_BW4: 'BW4',
    GYRO_LPF_BW5: 'BW5',
    GYRO_LPF_BW6: 'BW6',
    GYRO_LPF_BW7: 'BW7'
}

IIM4623x_GRAVITY = 9.8
global FORMAT, accel_scale, gyro_scale, temp_scale, temp_offset
global lpf_bw, accel_fsr, gyro_fsr
lpf_bw = ACC_LPF_BW4 | GYRO_LPF_BW4
accel_fsr = ACC_FSR_4G | 0x06
gyro_fsr = GYRO_FSR_480DPS | 0x06
class Reg:
    def __init__(self, first_addr, length, page_id):
        self.first_addr = first_addr
        self.length = length
        self.page_id = page_id

WHO_AM_I = Reg(0x00, 1, 0)
SERIAL_NUM = Reg(0x01, 16, 0)
FIRMWARE_REV = Reg(0x11, 2, 0)
BOOTLOADER_REV = Reg(0x13, 2, 0)
FLASH_ENDURANCE = Reg(0x15, 4, 0)
OUT_DATA_FORM = Reg(0x19, 1, 0)
SELECT_OUT_DATA = Reg(0x1C, 1, 0)
BW_CONFIG = Reg(0x30, 1, 0)
ACCEL_CONFIG0 = Reg(0x33, 1, 0)
GYRO_CONFIG0 = Reg(0x34, 1, 0)
SAMPLE_RATE_DIV = Reg (0x1A, 2 , 0)

def calc_checksum(buff):
    return sum(buff) & 0xFFFF

def IIM46234_SetCMD_ReadRegister(user_reg):
    cmd_packet = [0x00] * SIZE_PACKET_CMD
    cmd_packet[0] = BYTE_HEADER_CMD
    cmd_packet[1] = BYTE_HEADER_CMD
    cmd_packet[2] = SIZE_CMD_READ_REGS
    cmd_packet[3] = CMD_TYPE_READ_REG
    cmd_packet[4] = BYTE_RESERVED
    cmd_packet[5] = user_reg.length
    cmd_packet[6] = user_reg.first_addr
    cmd_packet[7] = user_reg.page_id
    checksum = calc_checksum(cmd_packet[3:8])
    cmd_packet[8] = (checksum >> 8) & 0xFF
    cmd_packet[9] = checksum & 0x00FF
    cmd_packet[10] = BYTE_FOOTER_1
    cmd_packet[11] = BYTE_FOOTER_2
    return cmd_packet


def IIM46234_SetCMD_WriteRegister(user_reg, value):
    cmd_packet = [0x00] * SIZE_PACKET_CMD
    cmd_packet[0] = BYTE_HEADER_CMD
    cmd_packet[1] = BYTE_HEADER_CMD
    cmd_packet[2] = SIZE_CMD_WRITE_REGS_BASE + user_reg.length
    cmd_packet[3] = CMD_TYPE_WRITE_REG
    cmd_packet[4] = BYTE_RESERVED
    cmd_packet[5] = user_reg.length
    cmd_packet[6] = user_reg.first_addr
    cmd_packet[7] = user_reg.page_id

    if user_reg.length == 1:
        cmd_packet[8] = value
    elif user_reg.length == 2:
        cmd_packet[8] = (value >> 8) & 0xFF
        cmd_packet[9] = value & 0xFF
    elif user_reg.length == 4:
        cmd_packet[8] = (value >> 24) & 0xFF
        cmd_packet[9] = (value >> 16) & 0xFF
        cmd_packet[10] = (value >> 8) & 0xFF
        cmd_packet[11] = value & 0xFF
    else:
        print("Does not support this length")
        return None

    checksum = calc_checksum(cmd_packet[3:8 + user_reg.length])

    cmd_packet[8 + user_reg.length] = (checksum >> 8) & 0xFF
    cmd_packet[9 + user_reg.length] = checksum & 0xFF
    cmd_packet[10 + user_reg.length] = BYTE_FOOTER_1
    cmd_packet[11 + user_reg.length] = BYTE_FOOTER_2

    return cmd_packet

def IIM46234_SetCMD_Common(cmd_type):
    cmd_packet = [0x00] * SIZE_PACKET_CMD
    cmd_packet[0] = BYTE_HEADER_CMD
    cmd_packet[1] = BYTE_HEADER_CMD
    cmd_packet[2] = SIZE_CMD_COMMON
    cmd_packet[3] = cmd_type
    checksum = calc_checksum(cmd_packet[3:4])
    cmd_packet[4] = (checksum >> 8) & 0xFF
    cmd_packet[5] = checksum & 0x00FF
    cmd_packet[6] = BYTE_FOOTER_1
    cmd_packet[7] = BYTE_FOOTER_2

    return cmd_packet


def IIM46234_Read_WhoAmI():
    cmd_packet = IIM46234_SetCMD_ReadRegister(WHO_AM_I)
    ser.write(bytearray(cmd_packet))
    whoami = ser.readline()
    print('WHO_AM_I:', whoami[12])

def IIM46234_Get_Version():
    cmd_packet = IIM46234_SetCMD_Common(CMD_TYPE_GET_VERSION)
    ser.write(bytearray(cmd_packet))
    version_bytes = ser.read(SIZE_RESP_GET_VERSION)
    if (version_bytes[3] == CMD_TYPE_GET_VERSION):

        major_version = version_bytes[6]
        minor_version = version_bytes[7]

        version_str = f"{major_version}.{minor_version}"
        print('version #:',version_str)


def IIM46234_Get_SerialNum():
    cmd_packet = IIM46234_SetCMD_Common(CMD_TYPE_GET_SERIAL_NUM)
    ser.write(bytearray(cmd_packet))
    SerialNum = ser.read(SIZE_RESP_GET_SERIAL_NUM)
    if SerialNum[3] == CMD_TYPE_GET_SERIAL_NUM:
        serial_num = int.from_bytes(SerialNum[18:22] + SerialNum[14:18] + SerialNum[10:14] + SerialNum[6:10], byteorder= 'big')
        print('serial #:', serial_num)

    # if SerialNum[3] == CMD_TYPE_GET_SERIAL_NUM:
    #     W3 = struct.unpack(">I", bytes(SerialNum[6:10]))[0]
    #     W2 = struct.unpack(">I", bytes(SerialNum[10:14]))[0]
    #     W1 = struct.unpack(">I", bytes(SerialNum[14:18]))[0]
    #     W0 = struct.unpack(">I", bytes(SerialNum[18:22]))[0]
    #
    #     print('serial #:', W0*pow(2,96)+W1*pow(2,64)+W2*pow(2,32)+W3)



def IIM46234_Start_Streaming():
    cmd_packet = IIM46234_SetCMD_Common(CMD_TYPE_START_STREAMING)
    ser.write(bytearray(cmd_packet))
    print('started streaming')
def IIM46234_Stop_Streaming():
    cmd_packet = IIM46234_SetCMD_Common(CMD_TYPE_STOP_STREAMING)
    ser.write(bytearray(cmd_packet))
    print('stopped streaming')


def IIM46234_Set_SelectOutData(select_out_data):
    IIM46234_SetCMD_WriteRegister(SELECT_OUT_DATA, select_out_data)
def convert_to_float(fixed_value, scale_factor):
    if fixed_value > 2**31:  # check if the value exceeds 2^31
        fixed_value = fixed_value - 2**32  # convert to negative
    return fixed_value * scale_factor


def IIM46234_Set_OutDataForm(out_data_form):
    global FORMAT, accel_scale, gyro_scale, temp_scale, temp_offset
    IIM46234_SetCMD_WriteRegister(OUT_DATA_FORM, out_data_form)
    if out_data_form == 0:
        FORMAT = '>HBBBB8s7fHH'
        accel_scale = 1
        gyro_scale = 1
        temp_scale = 1
        temp_offset = 0
    else:
        FORMAT = '>HBBBB8s7LHH'
        accel_scale = 8 / 2 ** 31
        gyro_scale = 480 / 2 ** 31
        temp_scale = 126.8
        temp_offset = 25

def IIM46234_Read_AccelConfig():
    cmd_packet = IIM46234_SetCMD_ReadRegister(ACCEL_CONFIG0)
    ser.write(bytearray(cmd_packet))
    fsr = ser.readline()
    print('ACCEL_CONFIG:', dict_acc_fsr[fsr[12] ^ 0x06])

def IIM46234_Set_AccelConfig(value):
    global accel_fsr
    accel_fsr &= 0x1F
    accel_fsr |= value
    cmd_packet = IIM46234_SetCMD_WriteRegister(ACCEL_CONFIG0, accel_fsr)
    ser.write(bytearray(cmd_packet))

def IIM46234_Read_GyroConfig():
    cmd_packet = IIM46234_SetCMD_ReadRegister(GYRO_CONFIG0)
    ser.write(bytearray(cmd_packet))
    fsr = ser.readline()
    print('GYRO_CONFIG:', dict_gyr_fsr[fsr[12] ^ 0x06])

def IIM46234_Set_GyroConfig(value):
    global gyro_fsr
    gyro_fsr &= 0x1F
    gyro_fsr |= value
    cmd_packet = IIM46234_SetCMD_WriteRegister(GYRO_CONFIG0, gyro_fsr)
    ser.write(bytearray(cmd_packet))

def IIM46234_Read_BWConfig_Accel():
    cmd_packet = IIM46234_SetCMD_ReadRegister(BW_CONFIG)
    ser.write(bytearray(cmd_packet))
    bw = ser.readline()
    print('ACCEL_BW:', dict_acc_bw[bw[12] & 0xF0])

def IIM46234_Set_BWConfig_Accel(acc_bw):
    global lpf_bw
    lpf_bw &= 0x0F
    lpf_bw |= acc_bw
    cmd_packet = IIM46234_SetCMD_WriteRegister(BW_CONFIG, lpf_bw)
    ser.write(bytearray(cmd_packet))

def IIM46234_Read_BWConfig_Gyro():
    cmd_packet = IIM46234_SetCMD_ReadRegister(BW_CONFIG)
    ser.write(bytearray(cmd_packet))
    bw = ser.readline()
    print('Gyro_BW:', dict_gyr_bw[bw[12] & 0x0F])

def IIM46234_Set_BWConfig_Gyro(gyr_bw):
    global lpf_bw
    lpf_bw &= 0xF0
    lpf_bw |= gyr_bw
    cmd_packet = IIM46234_SetCMD_WriteRegister(BW_CONFIG, lpf_bw)
    ser.write(bytearray(cmd_packet))
# enum IIM4623x_SampleRateDiv {
#     ODR_1KHZ = 1,
#     ODR_500HZ = 2,
#     ODR_250HZ = 4,
#     ODR_200HZ = 5,
#     ODR_125HZ = 8,
#     ODR_100HZ = 10,
#     ODR_50HZ = 20,
#     ODR_25HZ = 40,
#     ODR_20HZ = 50,
#     ODR_10HZ = 100 // 0x64
# };


def IIM46234_Set_SampleRateDiv(divisor):
    cmd_packet = IIM46234_SetCMD_WriteRegister(SAMPLE_RATE_DIV, divisor)
    ser.write(bytearray(cmd_packet))




class IIM4623xData:
    # Define the format for struct.unpack based on the structure layout


    def __init__(self, buffer):
        (
            self.hdr,
            self.length,
            self.type,
            self.status,
            self.sample_ctr,
            self.timestamp,
            self.ax,
            self.ay,
            self.az,
            self.gx,
            self.gy,
            self.gz,
            self.temp,
            self.checksum,
            self.footer
        ) = struct.unpack(FORMAT, buffer)

def read_sensor(serial_port):
    buffer = bytearray()
    while True:
        try:
            # Read a large chunk of data from the serial port
            data = serial_port.read(1024)  # Read 1024 bytes at a time
            buffer.extend(data)
            
            # Process buffer to extract and process complete packets
            while len(buffer) >= SIZE_PACKET_FULL_DATA:
                # Find the start of a packet
                start_index = buffer.find(BYTE_HEADER_REP.to_bytes(1, 'big'))
                if start_index == -1:
                    # No valid header found, clear buffer
                    buffer.clear()
                    break
                
                # Ensure we have a full packet starting from start_index
                if len(buffer) - start_index < SIZE_PACKET_FULL_DATA:
                    # Not enough data for a full packet, wait for more data
                    break
                
                # Extract a packet
                packet = buffer[start_index:start_index + SIZE_PACKET_FULL_DATA]
                buffer = buffer[start_index + SIZE_PACKET_FULL_DATA:]
                
                # Verify the packet header
                if packet[0] != BYTE_HEADER_REP or packet[1] != BYTE_HEADER_REP:
                    print(f"Wrong data stream header (0x{packet[0]:02x} 0x{packet[1]:02x})")
                    continue

                # Verify the packet type
                if packet[3] != 0xAB:
                    print(f"Wrong data stream type (0x{packet[3]:02x})")
                    continue

                # Verify the checksum
                checksum_read = (packet[SIZE_PACKET_FULL_DATA - 4] << 8) | packet[SIZE_PACKET_FULL_DATA - 3]
                checksum = calc_checksum(packet[3:SIZE_PACKET_FULL_DATA - 4])
                if checksum != checksum_read:
                    print(f"Incorrect checksum (read data) {checksum_read} {checksum}")
                    continue

                # Process the valid packet
                data = IIM4623xData(packet)
                data.ax = convert_to_float(data.ax, accel_scale)
                data.ay = convert_to_float(data.ay, accel_scale)
                data.az = convert_to_float(data.az, accel_scale)
                data.gx = convert_to_float(data.gx, gyro_scale)
                data.gy = convert_to_float(data.gy, gyro_scale)
                data.gz = convert_to_float(data.gz, gyro_scale)
                data.temp = data.temp * temp_scale + temp_offset

                print("ax: %.6f, ay: %.6f, az: %.6f, gx: %.6f, gy: %.6f, gz: %.6f, temp: %.6f" %
                      (data.ax, data.ay, data.az, data.gx, data.gy, data.gz, data.temp))
        
        except serial.SerialException as e:
            print(f"Serial exception: {e}")
        except Exception as e:
            print(f"Unexpected error: {e}")

def write_sensor(serial_port, duration=10):
    # open file
    cur_time = datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
    filename = cur_time + '.csv'
    with open(filename, 'w') as f:
        f.write('ax,ay,az,gx,gy,gz,temp\n') # header

        start_time = time.time()

        # read sensor
        buffer = bytearray()
        while time.time() - start_time < duration:
            try:
                # Read a large chunk of data from the serial port
                data = serial_port.read(1024)  # Read 1024 bytes at a time
                buffer.extend(data)
                
                # Process buffer to extract and process complete packets
                while len(buffer) >= SIZE_PACKET_FULL_DATA:
                    # Find the start of a packet
                    start_index = buffer.find(BYTE_HEADER_REP.to_bytes(1, 'big'))
                    if start_index == -1:
                        # No valid header found, clear buffer
                        buffer.clear()
                        break
                    
                    # Ensure we have a full packet starting from start_index
                    if len(buffer) - start_index < SIZE_PACKET_FULL_DATA:
                        # Not enough data for a full packet, wait for more data
                        break
                    
                    # Extract a packet
                    packet = buffer[start_index:start_index + SIZE_PACKET_FULL_DATA]
                    buffer = buffer[start_index + SIZE_PACKET_FULL_DATA:]
                    
                    # Verify the packet header
                    if packet[0] != BYTE_HEADER_REP or packet[1] != BYTE_HEADER_REP:
                        print(f"Wrong data stream header (0x{packet[0]:02x} 0x{packet[1]:02x})")
                        continue

                    # Verify the packet type
                    if packet[3] != 0xAB:
                        print(f"Wrong data stream type (0x{packet[3]:02x})")
                        continue

                    # Verify the checksum
                    checksum_read = (packet[SIZE_PACKET_FULL_DATA - 4] << 8) | packet[SIZE_PACKET_FULL_DATA - 3]
                    checksum = calc_checksum(packet[3:SIZE_PACKET_FULL_DATA - 4])
                    if checksum != checksum_read:
                        print(f"Incorrect checksum (read data) {checksum_read} {checksum}")
                        continue

                    # Process the valid packet
                    data = IIM4623xData(packet)
                    data.ax = convert_to_float(data.ax, accel_scale)
                    data.ay = convert_to_float(data.ay, accel_scale)
                    data.az = convert_to_float(data.az, accel_scale)
                    data.gx = convert_to_float(data.gx, gyro_scale)
                    data.gy = convert_to_float(data.gy, gyro_scale)
                    data.gz = convert_to_float(data.gz, gyro_scale)
                    data.temp = data.temp * temp_scale + temp_offset
                    
                    f.write('{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.2f}\n'.format(data.ax, data.ay, data.az, data.gx, data.gy, data.gz, data.temp))
                    # f.write('{:.6f},{:.6f},{:.6f}\n'.format(data.ax, data.ay, data.az))
            
            except serial.SerialException as e:
                print(f"Serial exception: {e}")
            except Exception as e:
                print(f"Unexpected error: {e}")



def IIM4623_flush_data(serial_port):
    try:
        for i in range(10):
            # Clear the input and output buffers
            serial_port.reset_input_buffer()
            serial_port.reset_output_buffer()
            # print("UART buffer flushed successfully")
            time.sleep(0.01)  # Sleep for 10 milliseconds
    except serial.SerialException as e:
        print(f"Error flushing UART buffer: {e}")


def find_port() -> Optional[str]:
    from serial.tools.list_ports import comports

    vid = [0x403, 0x04b4]
    pid = [0x6001, 0x0003]
    for info in comports():
        if info.vid and info.vid in vid and info.pid in pid:
            return info.device
        if 'VID_04B4' in info.hwid and 'PID_0003' in info.hwid:
            return info.device
    return None

def sig_handler(sig, frame):
    global ser
    logging.info(f'Program terminated by signal #{sig}')
    cleanup()
    sys.exit(0)

def cleanup():
    global ser
    try:
        if ser.is_open:
            IIM46234_Stop_Streaming()
            ser.close()
            logging.info("Serial connection closed.")
    except Exception as e:
        logging.error(f"Error during cleanup: {e}")
def set_high_priority():
    p = psutil.Process(os.getpid())
    try:
        p.nice(psutil.REALTIME_PRIORITY_CLASS)
        print("Process priority set to real-time.")
    except Exception as e:
        print(f"Failed to set process priority: {e}")

signal.signal(signal.SIGINT, sig_handler)

def main():
    set_high_priority()
    com_port = find_port()
    if com_port is None:
        print("No suitable COM port found.")
        return

    global ser
    ser = serial.Serial(com_port, baudrate=921600, timeout=0.1)
    ser.set_buffer_size(rx_size=1048576, tx_size=512)

    IIM46234_Stop_Streaming()
    IIM4623_flush_data(ser)
    IIM46234_Read_WhoAmI()
    IIM46234_Get_Version()
    IIM46234_Set_SelectOutData(BIT_SELECT_OUT_DATA_ACC | BIT_SELECT_OUT_DATA_GYRO | BIT_SELECT_OUT_DATA_TEMP)
    IIM46234_Get_SerialNum()
    IIM46234_Set_OutDataForm(0)
    IIM46234_Read_AccelConfig()
    IIM46234_Read_GyroConfig()
    IIM46234_Read_BWConfig_Accel()
    IIM46234_Read_BWConfig_Gyro()

    #  ODR_1KHZ = 1, 
    IIM46234_Set_SampleRateDiv(1)
    IIM4623_flush_data(ser)
    IIM46234_Set_BWConfig_Accel(ACC_LPF_BW4)
    IIM4623_flush_data(ser)
    IIM46234_Set_BWConfig_Gyro(GYRO_LPF_BW4)
    IIM4623_flush_data(ser)
    IIM46234_Set_AccelConfig(ACC_FSR_2G)
    IIM4623_flush_data(ser)
    IIM46234_Set_GyroConfig(GYRO_FSR_480DPS)
    IIM4623_flush_data(ser)

    IIM46234_Read_AccelConfig()
    IIM46234_Read_GyroConfig()
    IIM46234_Read_BWConfig_Accel()
    IIM46234_Read_BWConfig_Gyro()
    IIM4623_flush_data(ser)
    IIM46234_Start_Streaming()
    # read_sensor(ser)
    write_sensor(ser)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        IIM46234_Stop_Streaming()
        ser.close()



