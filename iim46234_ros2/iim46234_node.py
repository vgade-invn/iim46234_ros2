
import rclpy

from sensor_msgs.msg import Imu, Temperature
import numpy as np
import serial
import serial
import struct
import math
from tf2_ros import TransformBroadcaster
import tf_transformations
from rclpy.qos import QoSProfile


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


IIM4623x_GRAVITY = 9.8
global FORMAT, accel_scale, gyro_scale, temp_scale, temp_offset
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
SAMPLE_RATE_DIV = Reg(0x1A, 2, 0)
# Initialize the serial port
ser = serial.Serial('/dev/ttyACM0', 921600)

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
    else:
        print("Does not support this length")

    # Assuming calc_checksum is a function that's already defined in Python
    checksum = calc_checksum(cmd_packet[3:8 + user_reg.length])

    cmd_packet[8 + user_reg.length] = (checksum >> 8) & 0xFF
    cmd_packet[9 + user_reg.length] = checksum & 0x00FF
    cmd_packet[10 + user_reg.length] = BYTE_FOOTER_1
    cmd_packet[11 + user_reg.length] = BYTE_FOOTER_2

    return cmd_packet  # Added this line to return the cmd_packet, since the original C code doesn't explicitly return anything


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

# enum IIM46234_SampleRateDiv {
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


def IIM46234_Start_Streaming():
    cmd_packet = IIM46234_SetCMD_Common(CMD_TYPE_START_STREAMING)
    ser.write(bytearray(cmd_packet))
    print('started streaming')
def IIM46234_Stop_Streaming():
    cmd_packet = IIM46234_SetCMD_Common(CMD_TYPE_STOP_STREAMING)
    ser.write(bytearray(cmd_packet))
    print('stopped streaming')


def IIM46234_Set_SelectOutData(select_out_data):
    cmd_packet = IIM46234_SetCMD_WriteRegister(SELECT_OUT_DATA, select_out_data)
    ser.write(bytearray(cmd_packet))
   

def convert_to_float(fixed_value, scale_factor):
    if fixed_value > 2**31:  # check if the value exceeds 2^31
        fixed_value = fixed_value - 2**32  # convert to negative
    return fixed_value * scale_factor


def IIM46234_Set_OutDataForm(out_data_form):
    global FORMAT, accel_scale, gyro_scale, temp_scale, temp_offset
    cmd_packet = IIM46234_SetCMD_WriteRegister(OUT_DATA_FORM, out_data_form)
    # ser.write(bytearray(cmd_packet))
    if out_data_form == 0:
        FORMAT = '>HBBBB8s7fHH'
        accel_scale = 1
        gyro_scale = math.pi / 180
        temp_scale = 1
        temp_offset = 0
    else:
        FORMAT = '>HBBBB8s7LHH'
        accel_scale = 8 / 2 ** 31
        gyro_scale_dps = 480 / 2 ** 31
        gyro_scale= gyro_scale_dps * math.pi / 180
        temp_scale = 126.8
        temp_offset = 25

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

def accel_to_quaternion(accel):
    ref = np.array([0, 0, 1])
    acceln = accel / np.linalg.norm(accel)
    axis = np.cross(acceln, ref)
    angle = np.arccos(np.dot(acceln, ref))
    
    q = tf_transformations.quaternion_about_axis(angle, axis)

    return q

def read_sensor(node, imu_pub):
    while True:
        rbuf_data = [0] * SIZE_PACKET_FULL_DATA
        rbuf_data = ser.read(SIZE_PACKET_FULL_DATA)

        checksum_read = (rbuf_data[SIZE_PACKET_FULL_DATA - 4] << 8) | rbuf_data[SIZE_PACKET_FULL_DATA - 3]
        checksum = calc_checksum(rbuf_data[3:SIZE_PACKET_FULL_DATA - 4])

        if rbuf_data[0] != BYTE_HEADER_REP or rbuf_data[1] != BYTE_HEADER_REP:
            print(f"Wrong data stream header (0x{rbuf_data[0]:02x} 0x{rbuf_data[1]:02x})")
        if rbuf_data[3] != 0xAB:
            print(f"Wrong data stream type (0x{rbuf_data[3]:02x})")

        if checksum != checksum_read:
            print(f"Incorrect checksum (read data) {checksum_read} {checksum}")


        data = IIM4623xData(rbuf_data)



        data.ax = convert_to_float(data.ax, accel_scale)
        data.ay = convert_to_float(data.ay, accel_scale)
        data.az = convert_to_float(data.az, accel_scale)

        data.gx = convert_to_float(data.gx, gyro_scale)
        data.gy = convert_to_float(data.gy, gyro_scale)
        data.gz = convert_to_float(data.gz, gyro_scale)
        data.temp = data.temp*temp_scale+temp_offset



        # print(f"ax: {data.ax}, ay: {data.ay}, az: {data.az} gx: {data.gx}, gy: {data.gy}, gz: {data.gz} temp: {data.temp}")
        imu_msg = Imu()
        imu_msg.header.frame_id = "iim46234"
        imu_msg.header.stamp = node.get_clock().now().to_msg()
        imu_msg.linear_acceleration.x = data.ax
        imu_msg.linear_acceleration.y = data.ay
        imu_msg.linear_acceleration.z = data.az
        imu_msg.angular_velocity.x = data.gx
        imu_msg.angular_velocity.y = data.gy
        imu_msg.angular_velocity.z = data.gz

        # Calculate the orientation using the acceleration data
        orientation_quat = accel_to_quaternion(np.array([data.ax, data.ay, data.az]))
        imu_msg.orientation.x = orientation_quat[1]
        imu_msg.orientation.y = orientation_quat[2]
        imu_msg.orientation.z = orientation_quat[3]
        imu_msg.orientation.w = orientation_quat[0]
        
        

        # imu_msg.orientation_covariance = [-1.0] * 9  # Set orientation covariance to indicate no orientation data


        # Publish the Imu message
        imu_pub.publish(imu_msg)







def main():
    rclpy.init()
    node = rclpy.create_node('iim46234_publisher')

 
    qos_profile = rclpy.qos.qos_profile_sensor_data

    imu_pub = node.create_publisher(Imu, '/imu/data_raw', qos_profile=qos_profile)


    IIM46234_Read_WhoAmI()
    IIM46234_Get_Version()
    IIM46234_Get_SerialNum()
    IIM46234_Set_OutDataForm(0)
    IIM46234_Start_Streaming()
    
    timer_frequency = 100  # 100Hz
    timer_period = 1.0 / timer_frequency
    timer = node.create_timer(timer_period, lambda: read_sensor(node, imu_pub))


    try:
        while rclpy.ok():
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        IIM46234_Stop_Streaming()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()



