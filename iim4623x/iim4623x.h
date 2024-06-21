/*
 * __________________________________________________________________
 *
 * Copyright (C) [2023] by InvenSense, Inc.
 * 
 * Permission to use, copy, modify, and/or distribute this software
 * for any purpose with or without fee is hereby granted.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY  AND FITNESS. IN NO EVENT SHALL
 * THE AUTHOR BE LIABLE  FOR ANY SPECIAL, DIRECT, INDIRECT, OR
 * CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM
 * LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 * __________________________________________________________________
 */

#ifndef _INV_IIM4623X_H_
#define _INV_IIM4623X_H_

#define BYTE_HEADER_CMD	0x24
#define BYTE_HEADER_REP	0x23
#define BYTE_RESERVED	0x00
#define BYTE_FOOTER_1	0x0D
#define BYTE_FOOTER_2	0x0A
#define BYTE_PADDING	0x00

#define CMD_TYPE_SWITCH_TO_BOOTLOADER  0xF4
#define CMD_TYPE_UPGRADE_FIRMWARE      0xF5
#define CMD_TYPE_CLEAR_UPGRADE_FLAG    0xF6
#define CMD_TYPE_LENGTH_INFO           0xDA
#define CMD_TYPE_IMAGE_DATA            0xDA

#define CMD_TYPE_GET_VERSION		0x20
#define CMD_TYPE_GET_SERIAL_NUM		0x26
#define CMD_TYPE_READ_REG			0x11
#define CMD_TYPE_WRITE_REG			0x12
#define CMD_TYPE_SELF_TEST			0x2B
#define CMD_TYPE_SET_UTC_TIME		0x2D
#define CMD_TYPE_START_STREAMING	0x27
#define CMD_TYPE_STOP_STREAMING		0x28
#define CMD_TYPE_ENABLE_SENSORFT	0x2E
#define CMD_TYPE_DISABLE_SENSORFT	0x2F

#define SIZE_CMD_COMMON				8
#define SIZE_CMD_LENGTH_INFO        16
#define SIZE_CMD_SET_UTC_TIME		15
#define SIZE_CMD_READ_REGS			12
#define SIZE_CMD_WRITE_REGS_BASE	12

#define SIZE_RESP_ACK				10
#define SIZE_RESP_GET_SERIAL_NUM	26
#define SIZE_RESP_GET_VERSION		20
#define SIZE_RESP_IMU_SELF_TEST		16
#define SIZE_RESP_READ_REGS_BASE	16

#define SIZE_PACKET_CMD			20
#define SIZE_PACKET_CMD_FW		20
#define SIZE_PACKET_DATA_FW		523
#define SIZE_PACKET_BASE_DATA	18
#define SIZE_PACKET_FULL_DATA	70
#define SIZE_BUFF_RESP			26

#define SELF_TEST_RESULT_PASS	0x03
#define ERR_CODE_SUCCESS	0x00

typedef struct _reg{
	uint8_t first_addr;
	uint8_t length;
	uint8_t page_id;
} reg;

#define BIT_SELECT_OUT_DATA_VEL		(0x10)
#define BIT_SELECT_OUT_DATA_ANG		(0x08)
#define BIT_SELECT_OUT_DATA_TEMP	(0x04)
#define BIT_SELECT_OUT_DATA_GYRO	(0x02)
#define BIT_SELECT_OUT_DATA_ACC		(0x01)

#define BIT_SAVE_ALL_CONFIG_CMD		(0x50)
#define BIT_SAVE_ALL_CONFIG_RESULT_IN_PROGRESS	(0x00)
#define BIT_SAVE_ALL_CONFIG_RESULT_SUCCESS		(0x01)
#define BIT_SAVE_ALL_CONFIG_RESULT_NOT_SAVED	(0x02)

#define BIT_ENABLE_EXT_ACCEL_BIAS		(0x01)
#define BIT_ENABLE_EXT_GYRO_BIAS		(0x02)
#define BIT_ENABLE_EXT_ACCEL_SENS		(0x04)
#define BIT_ENABLE_EXT_GYRO_SENS		(0x08)

#define IIM46230_WHO_AM_I	0xE6
#define IIM46234_WHO_AM_I	0xEA

typedef struct _utc{
	uint16_t year;
	uint16_t month;
	uint16_t day;
	uint16_t hh;
	uint16_t mm;
	uint16_t ss;
} utc;

typedef enum IIM4623x_PartNum {
    IIM46230 = 0,
    IIM46234
} PartNum;

enum IIM4623x_Intf {
    INTF_UART = 1,
    INTF_SPI
};

enum IIM4623x_Mode {
    COMMAND = 0,
    STREAMING
};

enum IIM4623x_OutDataForm {
    FLOATING = 0,	// 32-Bit IEEE 754 single-precision floating point (default)
    FIXED			// 32-Bit Fixed point 2's Complement representation
};

enum IIM4623x_DataOutPut {
    ACCEL = 0,
    GYRO,
    TEMP,
    DELTA_VEL,
    DELTA_ANG
};

enum IIM4623x_SampleRateDiv {
    ODR_1KHZ = 1,
    ODR_500HZ = 2,
    ODR_250HZ = 4,
    ODR_200HZ = 5,
    ODR_125HZ = 8,
    ODR_100HZ = 10,
    ODR_50HZ = 20,
    ODR_25HZ = 40,
    ODR_20HZ = 50,
    ODR_10HZ = 100 // 0x64
};

enum IIM4623x_UartBaudRate {
    BAUD_921600 = 0,
    BAUD_1500000 = 1,
    BAUD_3000000 = 3
};

enum IIM4623x_SyncConfig {
    DISABLE_SYNC = 0,
    SYNC_WITH_PPS = 1
};

enum IIM4623x_AccBwConfig {
    ACC_LPF_BW4 = 0x40,
	ACC_LPF_BW5 = 0x50,
	ACC_LPF_BW6 = 0x60,
	ACC_LPF_BW7 = 0x70
};

enum IIM4623x_GyroBwConfig {
    GYRO_LPF_BW4 = 0x4,
	GYRO_LPF_BW5 = 0x5,
	GYRO_LPF_BW6 = 0x6,
	GYRO_LPF_BW7 = 0x7
};

enum IIM4623x_AccelConfig0 {
	ACC_FSR_16G = 0x00,
	ACC_FSR_8G = 0x20,
	ACC_FSR_4G = 0x40,
	ACC_FSR_2G = 0x60
};

enum IIM4623x_GyroConfig0 {
    GYRO_FSR_2000DPS = 0x00,
	GYRO_FSR_1000DPS = 0x20,
    GYRO_FSR_500DPS = 0x40,
    GYRO_FSR_480DPS = 0x40,
	GYRO_FSR_250DPS = 0x60
};

enum IIM4623x_Axis {
    ACCEL_X = 0,
	ACCEL_Y,
	ACCEL_Z,
	GYRO_X,
	GYRO_Y,
	GYRO_Z
};

enum IIM4623x_CalibConfig {
    ACCEL_BIAS = 0,
    GYRO_BIAS,
    ACCEL_SENS,
    GYRO_SENS
};

enum IIM4623x_Mat_Index {
    X_1 = 0,
	X_2,
	X_3,
	Y_1,
	Y_2,
	Y_3,
	Z_1,
	Z_2,
	Z_3
};

typedef union{
    //uint8_t u8[4]; // Raw bytes received from SPI/UART
    float val; //Floating point value       
    uint32_t u32; //Fixed point value
}float_uint_t;

struct IIM4623x_State {
    uint32_t  (*serif_rx)(uint8_t * buf, uint32_t len);
    uint32_t  (*serif_tx)(uint8_t * buf, uint32_t len);
	uint32_t  (*serif_tx_rx)(uint8_t * wbuf, uint32_t wlen, uint8_t * rbuf, uint32_t rlen);
	void (*reset)(void);

	uint32_t fw_size;
	uint32_t fw_sent_data;
	bool fw_sending_done;

	enum IIM4623x_Mode mode;
	uint8_t fw_cmd_packet[SIZE_PACKET_DATA_FW];
	uint8_t cmd_packet[SIZE_PACKET_CMD];
	bool stp_cmd_in_streaming;
	bool utc_cmd_in_streaming;
	bool sensorft_just_toggled;
	bool sensorft_enabled;
	utc utc_time;
	PartNum part_num;
	enum IIM4623x_Intf streaming_intf;
	enum IIM4623x_OutDataForm data_form;
	enum IIM4623x_UartBaudRate baud_rate;
	enum IIM4623x_SyncConfig sync_config;
	uint8_t data_out;
	enum IIM4623x_SampleRateDiv rate;
	uint8_t lpf_bw;
	uint8_t accel_fsr;
	uint8_t gyro_fsr;
	uint8_t calib_config;
	float_uint_t acc_x;
	float_uint_t acc_y;
	float_uint_t acc_z;
	float_uint_t gyro_x;
	float_uint_t gyro_y;
	float_uint_t gyro_z;
	float_uint_t temp;
	float_uint_t d_vel_x;
	float_uint_t d_vel_y;
	float_uint_t d_vel_z;
	float_uint_t d_ang_x;
	float_uint_t d_ang_y;
	float_uint_t d_ang_z;	
} state;

void IIM4623x_set_serif(uint32_t (*read)(uint8_t *, uint32_t),
                            uint32_t (*write)(uint8_t *, uint32_t),
                            uint32_t (*write_read)(uint8_t *, uint32_t, uint8_t *, uint32_t));
uint32_t IIM4623x_serif_rx(uint8_t *buf, uint32_t len);
uint32_t IIM4623x_serif_tx(uint8_t *buf, uint32_t len);
uint32_t IIM4623x_serif_tx_rx(uint8_t *wBuf, uint32_t wlen,  uint8_t *rBuf, uint32_t rlen);
void IIM4623x_set_reset(void (*reset)(void));
PartNum IIM4623x_Get_PartNum(void);
uint16_t calc_checksum(uint8_t *buff, uint32_t length);
uint32_t calc_checksum_32(uint8_t *buff, uint32_t length);
void IIM4623x_Reset(void);
void IIM4623x_Initialize(void);
void IIM4623x_SetCMD_Firmware(uint8_t cmd_type);
void IIM4623x_Switch_to_Bootloader(void);
void IIM4623x_Upgrade_Firmware(void);
void IIM4623x_Send_Length_Info(void);
void IIM4623x_Send_Image_Data(void);
void IIM4623x_Clear_Upgrade_Flag(void);
void IIM4623x_SetCMD_Common(uint8_t cmd_type);
void IIM4623x_SetCMD_ReadRegister(reg user_reg);
void IIM4623x_SetCMD_WriteRegister(reg user_reg, uint8_t *value);
void IIM4623x_Set_UtcTime(utc time);
void IIM4623x_Read_WhoAmI(void);
void IIM4623x_Get_SerialNum(void);
void IIM4623x_Get_Version(void);
void IIM4623x_Imu_SelfTest(void);
void IIM4623x_Set_OutDataForm(enum IIM4623x_OutDataForm form);
void IIM4623x_Set_SampleRateDiv(enum IIM4623x_SampleRateDiv divisor);
void IIM4623x_Toggle_DataOutput(enum IIM4623x_DataOutPut output);
void IIM4623x_Set_UartIfConfig(enum IIM4623x_UartBaudRate baud);
void IIM4623x_Set_SyncConfig(enum IIM4623x_SyncConfig sync_mode);
void IIM4623x_Set_BWConfig_Accel(enum IIM4623x_AccBwConfig bw);
void IIM4623x_Set_BWConfig_Gyro(enum IIM4623x_GyroBwConfig bw);
void IIM4623x_Set_AccelConfig0(enum IIM4623x_AccelConfig0 fsr);
void IIM4623x_Set_GyroConfig0(enum IIM4623x_GyroConfig0 fsr);
void IIM4623x_Toggle_ExtCalibConfig(enum IIM4623x_CalibConfig config);
void IIM4623x_Write_Bias(enum IIM4623x_Axis axis, float bias_val);
void IIM4623x_Read_Bias(enum IIM4623x_Axis axis);
void IIM4623x_Write_Sens_Mat_Accel(enum IIM4623x_Mat_Index indx, float value);
void IIM4623x_Write_Sens_Mat_Gyro(enum IIM4623x_Mat_Index indx, float value);
void IIM4623x_Read_Sens_Mat_Accel(enum IIM4623x_Mat_Index indx);
void IIM4623x_Read_Sens_Mat_Gyro(enum IIM4623x_Mat_Index indx);
void set_read_length(uint32_t *length);
void IIM4623x_Start_Streaming(void);
uint32_t IIM4623x_Stop_Streaming(void);
uint32_t read_u32_data(uint8_t *p_buf);
uint32_t IIM4623x_Read_DataOutput(void);
bool Is_Command_Mode(void);
void IIM4623x_Set_StartStreaming(void);
void IIM4623x_Set_StopStreaming(void);
void IIM4623x_Set_SensorFT(bool on_off);
void IIM4623x_Set_UtcTimeInStreaming(void);
uint32_t response_handler(void);

#endif
