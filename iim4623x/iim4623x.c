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

#include <string.h>
#include "iim4623x.h"

#define DBOX_STYLE_DATA_LOGGING    1
#define DEBUG_PHASE                0

uint8_t fw_image[] = {
#include "iim4623x_fw_img_3_10.h"
};

uint32_t count_data_out = 0;
extern uint32_t enable_log;

extern volatile int drdy_interrupt;
extern volatile uint64_t timestamp;
extern int bus_type;

uint8_t rbuf_resp[SIZE_BUFF_RESP];
uint8_t rbuf_data[SIZE_PACKET_FULL_DATA];

/* Register Information */
/* Name	= {Address, Length, Page ID} */
reg WHO_AM_I			= {0x00, 1 , 0};
reg SERIAL_NUM 			= {0x01, 16, 0};
reg FIRMWARE_REV		= {0x11, 2 , 0};
reg BOOTLOADER_REV		= {0x13, 2 , 0};
reg FLASH_ENDURANCE		= {0x15, 4 , 0};
reg OUT_DATA_FORM		= {0x19, 1 , 0};
reg SAMPLE_RATE_DIV		= {0x1A, 2 , 0};
reg SELECT_OUT_DATA		= {0x1C, 1 , 0};
reg UART_IF_CONFIG		= {0x1D, 1 , 0};
reg SYNC_CONFIG			= {0x1E, 1 , 0};
reg USER_SCRATCH1		= {0x1F, 8 , 0};
reg USER_SCRATCH2		= {0x27, 8 , 0};
reg SAVE_ALL_CONFIG		= {0x2F, 1 , 0};
reg BW_CONFIG			= {0x30, 1 , 0};
reg ACCEL_CONFIG0       = {0x33, 1 , 0};
reg GYRO_CONFIG0        = {0x34, 1 , 0};
reg EXT_CALIB_CONFIG    = {0x3F, 1 , 0};
reg EXT_ACCEL_X_BIAS    = {0x40, 4 , 0};
reg EXT_ACCEL_Y_BIAS    = {0x44, 4 , 0};
reg EXT_ACCEL_Z_BIAS    = {0x48, 4 , 0};
reg EXT_GYRO_X_BIAS     = {0x4C, 4 , 0};
reg EXT_GYRO_Y_BIAS     = {0x50, 4 , 0};
reg EXT_GYRO_Z_BIAS     = {0x54, 4 , 0};

reg EXT_ACC_SENS_MAT11  = {0x58, 4 , 0};
reg EXT_ACC_SENS_MAT12  = {0x5C, 4 , 0};
reg EXT_ACC_SENS_MAT13  = {0x60, 4 , 0};
reg EXT_ACC_SENS_MAT21  = {0x64, 4 , 0};
reg EXT_ACC_SENS_MAT22  = {0x68, 4 , 0};
reg EXT_ACC_SENS_MAT23  = {0x6C, 4 , 0};
reg EXT_ACC_SENS_MAT31  = {0x70, 4 , 0};
reg EXT_ACC_SENS_MAT32  = {0x74, 4 , 0};
reg EXT_ACC_SENS_MAT33  = {0x78, 4 , 0};

reg EXT_GYR_SENS_MAT11  = {0x7C, 4 , 0};
reg EXT_GYR_SENS_MAT12  = {0x80, 4 , 0};
reg EXT_GYR_SENS_MAT13  = {0x84, 4 , 0};
reg EXT_GYR_SENS_MAT21  = {0x88, 4 , 0};
reg EXT_GYR_SENS_MAT22  = {0x8C, 4 , 0};
reg EXT_GYR_SENS_MAT23  = {0x90, 4 , 0};
reg EXT_GYR_SENS_MAT31  = {0x94, 4 , 0};
reg EXT_GYR_SENS_MAT32  = {0x98, 4 , 0};
reg EXT_GYR_SENS_MAT33  = {0x9C, 4 , 0};

reg DATA_READY_STATUS	= {0x00, 1 , 1};
reg TIMESTAMP_OUT		= {0x03, 8 , 1};
reg ACCEL_X_OUTPUT		= {0x0B, 4 , 1};
reg ACCEL_Y_OUTPUT		= {0x0F, 4 , 1};
reg ACCEL_Z_OUTPUT		= {0x13, 4 , 1};
reg GYRO_X_OUTPUT		= {0x17, 4 , 1};
reg GYRO_Y_OUTPUT		= {0x1B, 4 , 1};
reg GYRO_Z_OUTPUT		= {0x1F, 4 , 1};
reg TEMPERA_OUTPUT		= {0x23, 4 , 1};
reg DELTA_VEL_X			= {0x27, 4 , 1};
reg DELTA_VEL_Y			= {0x2B, 4 , 1};
reg DELTA_VEL_Z			= {0x2F, 4 , 1};
reg DELTA_ANGLE_X		= {0x33, 4 , 1};
reg DELTA_ANGLE_Y		= {0x37, 4 , 1};
reg DELTA_ANGLE_Z		= {0x3B, 4 , 1};

void IIM4623x_set_serif(uint32_t (*rx)(uint8_t *, uint32_t),
                            uint32_t (*tx)(uint8_t *, uint32_t),
                            uint32_t (*tx_rx)(uint8_t *, uint32_t, uint8_t *, uint32_t))
{
    if (rx == NULL || tx == NULL || tx_rx == NULL) {
        printf("Read/Write API NULL POINTER!! \n\r");
        exit(-1);
    }
    state.serif_rx = rx;
    state.serif_tx = tx;
	state.serif_tx_rx = tx_rx;
}

void IIM4623x_set_reset(void (*reset)(void))
{
    if (reset == NULL ) {
        printf("Reset API NULL POINTER!! \n\r");
        exit(-1);
    }
    state.reset = reset;
}

uint32_t IIM4623x_serif_rx(uint8_t *buf, uint32_t len)
{
	return state.serif_rx(buf, len);
}

uint32_t IIM4623x_serif_tx(uint8_t *buf, uint32_t len)
{
	return state.serif_tx(buf, len);
}

uint32_t IIM4623x_serif_tx_rx(uint8_t *wBuf, uint32_t wlen,  uint8_t *rBuf, uint32_t rlen)
{
	if (bus_type == 0) {
		if (wlen > rlen) {
			printf("Write size should not be larger than read size !! \n\r");
			return 1;
		}
	}

	return state.serif_tx_rx(wBuf, wlen, rBuf, rlen);;
}

void IIM4623x_Reset(void)
{
	return state.reset();
}

PartNum IIM4623x_Get_PartNum(void)
{
	return state.part_num;
}

void IIM4623x_Initialize(void)
{
 	state.fw_size = sizeof(fw_image)/sizeof(fw_image[0]);
	state.fw_sent_data = 0;
	state.fw_sending_done = false;

	state.mode = COMMAND;
	state.stp_cmd_in_streaming = false;
	state.utc_cmd_in_streaming = false;
	state.utc_time.year = 2020;
	state.utc_time.month = 1;
	state.utc_time.day = 1;
	state.utc_time.hh = 00;
	state.utc_time.mm = 00;
	state.utc_time.ss = 00;
	if (bus_type == 0)
		state.streaming_intf = INTF_SPI;
	else if (bus_type == 1)
		state.streaming_intf = INTF_UART;
	else
		printf("Initial interface setting error \n\r");
	state.data_form = FLOATING;
	state.baud_rate = BAUD_3000000;
	state.sync_config = DISABLE_SYNC;
	state.data_out = BIT_SELECT_OUT_DATA_ACC | BIT_SELECT_OUT_DATA_GYRO | BIT_SELECT_OUT_DATA_TEMP;
	state.rate = ODR_1KHZ;
	state.lpf_bw = ACC_LPF_BW4 | GYRO_LPF_BW4;
	state.accel_fsr = ACC_FSR_8G | 0x06;
	state.gyro_fsr = GYRO_FSR_480DPS | 0x06;
	state.calib_config = 0;
	state.sensorft_enabled = true;
	state.sensorft_just_toggled = false;

	IIM4623x_Reset();
	printf("Resetting device... \n\r");
	delay_ms(200);

	uint32_t rSize = SIZE_RESP_READ_REGS_BASE+1;
	uint16_t checksum_read;
	uint16_t checksum;
	
	IIM4623x_Read_WhoAmI();
	while(1) {
		if (drdy_interrupt == 1) {
			if (IIM4623x_serif_rx(rbuf_resp, rSize)) {
		  		printf("Read error \n\r");
		  		return;
			}

			drdy_interrupt = 0;
		  
			checksum_read = rbuf_resp[rSize-4]<<8 | rbuf_resp[rSize-3];
			checksum = calc_checksum(&rbuf_resp[3], (rSize-7)); 
			if (checksum != checksum_read) {
				printf("Incorrect checksum (read register) %d %d \n\r", checksum_read, checksum);
			}

			if (rbuf_resp[3] == CMD_TYPE_READ_REG) {
				if (rbuf_resp[9] == state.cmd_packet[5]) {
					if (rbuf_resp[12] == IIM46230_WHO_AM_I) state.part_num = IIM46230;
					else if (rbuf_resp[12] == IIM46234_WHO_AM_I) state.part_num = IIM46234;

					if (state.part_num == IIM46230)
						printf("IIM46230 is present \n\r");
					else if (state.part_num == IIM46234)
						printf("IIM46234 is present \n\r");					
				}
				else {
					printf("Read length does not match !!! (Read length Info. in CMD: %d) \n\r", state.cmd_packet[5]);
					return;
				}
			}
			else {
				printf("Type of response does not match !!! (CMD Type: 0x%x) \n\r", CMD_TYPE_READ_REG);
				return;
			}
			
			break;
		}
	}
}

uint16_t calc_checksum(uint8_t *buff, uint32_t length)
{
	uint16_t sum = 0;
	for (uint32_t i = 0; i < length; i++) sum += (uint16_t)buff[i];
	return sum;
}

uint32_t calc_checksum_32(uint8_t *buff, uint32_t length)
{
	uint32_t sum = 0;
	for (uint32_t i = 0; i < length; i++) sum += (uint32_t)buff[i];
	return sum;
}

void IIM4623x_SetCMD_Firmware(uint8_t cmd_type)
{
	for (uint32_t i = 0; i < SIZE_PACKET_CMD_FW; i++) state.cmd_packet[i] = 0x00;
	state.cmd_packet[0] = BYTE_HEADER_CMD;
	state.cmd_packet[1] = BYTE_HEADER_CMD;
	state.cmd_packet[2] = SIZE_CMD_COMMON;
	state.cmd_packet[3] = cmd_type;
	uint16_t checksum = calc_checksum(&state.cmd_packet[3], 1);
	state.cmd_packet[4] = (uint8_t)(checksum >> 8);
	state.cmd_packet[5] = (uint8_t)(checksum &= 0x00FF);
	state.cmd_packet[6] = BYTE_FOOTER_1;
	state.cmd_packet[7] = BYTE_FOOTER_2;
}

void IIM4623x_Switch_to_Bootloader(void)
{
	IIM4623x_SetCMD_Firmware(CMD_TYPE_SWITCH_TO_BOOTLOADER);

	if (IIM4623x_serif_tx(state.cmd_packet, SIZE_PACKET_CMD_FW)) {
		printf("Write error \n\r");
	}
}

void IIM4623x_Upgrade_Firmware(void)
{
	IIM4623x_SetCMD_Firmware(CMD_TYPE_UPGRADE_FIRMWARE);

	if (IIM4623x_serif_tx(state.cmd_packet, SIZE_PACKET_CMD_FW)) {
		printf("Write error \n\r");
	}
}

void IIM4623x_Send_Length_Info(void)
{
    if (state.fw_sending_done == true) {
        printf("FW update was already done \n\r");
		return;
    }

	uint32_t fw_chsum = calc_checksum_32(fw_image, state.fw_size);

	for (uint32_t i = 0; i < SIZE_PACKET_CMD_FW; i++) state.cmd_packet[i] = 0x00;

	state.cmd_packet[0] = BYTE_HEADER_CMD;
	state.cmd_packet[1] = BYTE_HEADER_CMD;
	state.cmd_packet[2] = SIZE_CMD_LENGTH_INFO;
	state.cmd_packet[3] = CMD_TYPE_LENGTH_INFO;
	state.cmd_packet[4] = (uint8_t)(state.fw_size & 0xFF);
	state.cmd_packet[5] = (uint8_t)((state.fw_size >> 8) & 0xFF);
	state.cmd_packet[6] = (uint8_t)((state.fw_size >> 16) & 0xFF);
	state.cmd_packet[7] = (uint8_t)((state.fw_size >> 24) & 0xFF);
	state.cmd_packet[8] = (uint8_t)(fw_chsum & 0xFF);
	state.cmd_packet[9] = (uint8_t)((fw_chsum >> 8) & 0xFF);
	state.cmd_packet[10] = (uint8_t)((fw_chsum >> 16) & 0xFF);
	state.cmd_packet[11] = (uint8_t)((fw_chsum >> 24) & 0xFF);
	uint16_t checksum = calc_checksum(&state.cmd_packet[3], 9);
	state.cmd_packet[12] = (uint8_t)(checksum >> 8);
	state.cmd_packet[13] = (uint8_t)(checksum &= 0xFF);
	state.cmd_packet[14] = BYTE_FOOTER_1;
	state.cmd_packet[15] = BYTE_FOOTER_2;

	if (IIM4623x_serif_tx(state.cmd_packet, SIZE_PACKET_CMD_FW)) {
		printf("Write error \n\r");
	}
}

void IIM4623x_Send_Image_Data(void)
{
    if (state.fw_sending_done == true) {
        printf("Already sent FW image data \n\r");
		return;
    }

    uint32_t remaining = state.fw_size - state.fw_sent_data;
	uint32_t data_len = (remaining/512 >= 1)? 512: remaining;

    for (uint32_t i = 0; i < (data_len + 11); i++) state.fw_cmd_packet[i] = 0x00;
	state.fw_cmd_packet[0] = BYTE_HEADER_CMD;
	state.fw_cmd_packet[1] = BYTE_HEADER_CMD;
	state.fw_cmd_packet[2] = (uint8_t)(((data_len + 11) >> 8) & 0xFF);
	state.fw_cmd_packet[3] = (uint8_t)((data_len + 11) & 0xFF);
	state.fw_cmd_packet[4] = CMD_TYPE_IMAGE_DATA;
	for (uint32_t j = 0; j < data_len; j++) state.fw_cmd_packet[j+5] = fw_image[state.fw_sent_data + j];
	uint32_t checksum = calc_checksum_32(&state.fw_cmd_packet[4], data_len+1);
	state.fw_cmd_packet[data_len+5] = (uint8_t)((checksum >> 24) & 0xFF);
	state.fw_cmd_packet[data_len+6] = (uint8_t)((checksum >> 16) & 0xFF);
	state.fw_cmd_packet[data_len+7] = (uint8_t)((checksum >> 8) & 0xFF);
	state.fw_cmd_packet[data_len+8] = (uint8_t)(checksum & 0xFF);
	state.fw_cmd_packet[data_len+9] = BYTE_FOOTER_1;
	state.fw_cmd_packet[data_len+10] = BYTE_FOOTER_2;

    state.fw_sent_data += data_len;
    if (state.fw_sent_data == state.fw_size) {
		state.fw_sending_done = true;
		printf("Sending FW image data completed \n\r");
    }
	
	if (IIM4623x_serif_tx(state.fw_cmd_packet, data_len + 11)) {
		printf("Write error \n\r");
	}
}

void IIM4623x_Clear_Upgrade_Flag(void)
{
	IIM4623x_SetCMD_Firmware(CMD_TYPE_CLEAR_UPGRADE_FLAG);

	if (IIM4623x_serif_tx(state.cmd_packet, SIZE_PACKET_CMD_FW)) {
		printf("Write error \n\r");
	}
}

void IIM4623x_SetCMD_Common(uint8_t cmd_type)
{
	for (uint32_t i = 0; i < SIZE_PACKET_CMD; i++) state.cmd_packet[i] = 0x00;
	state.cmd_packet[0] = BYTE_HEADER_CMD;
	state.cmd_packet[1] = BYTE_HEADER_CMD;
	state.cmd_packet[2] = SIZE_CMD_COMMON;
	state.cmd_packet[3] = cmd_type;
	uint16_t checksum = calc_checksum(&state.cmd_packet[3], 1);
	state.cmd_packet[4] = (uint8_t)(checksum >> 8);
	state.cmd_packet[5] = (uint8_t)(checksum &= 0x00FF);
	state.cmd_packet[6] = BYTE_FOOTER_1;
	state.cmd_packet[7] = BYTE_FOOTER_2;
}

void IIM4623x_SetCMD_ReadRegister(reg user_reg)
{
	for (uint32_t i = 0; i < SIZE_PACKET_CMD; i++) state.cmd_packet[i] = 0x00;
	state.cmd_packet[0] = BYTE_HEADER_CMD;
	state.cmd_packet[1] = BYTE_HEADER_CMD;
	state.cmd_packet[2] = SIZE_CMD_READ_REGS;
	state.cmd_packet[3] = CMD_TYPE_READ_REG;
	state.cmd_packet[4] = BYTE_RESERVED;
	state.cmd_packet[5] = user_reg.length;
	state.cmd_packet[6] = user_reg.first_addr;
	state.cmd_packet[7] = user_reg.page_id;
	uint16_t checksum = calc_checksum(&state.cmd_packet[3], 5);
	state.cmd_packet[8] = (uint8_t)(checksum >> 8);
	state.cmd_packet[9] = (uint8_t)(checksum &= 0x00FF);
	state.cmd_packet[10] = BYTE_FOOTER_1;
	state.cmd_packet[11] = BYTE_FOOTER_2;
}

void IIM4623x_SetCMD_WriteRegister(reg user_reg, uint8_t *value)
{
	for (uint32_t i = 0; i < SIZE_PACKET_CMD; i++) state.cmd_packet[i] = 0x00;
	state.cmd_packet[0] = BYTE_HEADER_CMD;
	state.cmd_packet[1] = BYTE_HEADER_CMD;
	state.cmd_packet[2] = SIZE_CMD_WRITE_REGS_BASE + user_reg.length;
	state.cmd_packet[3] = CMD_TYPE_WRITE_REG;
	state.cmd_packet[4] = BYTE_RESERVED;
	state.cmd_packet[5] = user_reg.length;
	state.cmd_packet[6] = user_reg.first_addr;
	state.cmd_packet[7] = user_reg.page_id;
	if (user_reg.length == 1)
		state.cmd_packet[8] = *value;
	else if (user_reg.length == 2) {
		state.cmd_packet[8] = (uint8_t)((*value) >> 8);
		state.cmd_packet[9] = (uint8_t)((*value) &= 0x00FF);
	}
	else if (user_reg.length == 4) {
		state.cmd_packet[8] = *value;
		state.cmd_packet[9] = *(value+1);
		state.cmd_packet[10] = *(value+2);
		state.cmd_packet[11] = *(value+3);	
	}	
	else
		printf("Does not support this length \n\r");
	uint16_t checksum = calc_checksum(&state.cmd_packet[3], 5+user_reg.length);
	state.cmd_packet[8+user_reg.length] = (uint8_t)(checksum >> 8);
	state.cmd_packet[9+user_reg.length] = (uint8_t)(checksum &= 0x00FF);	
	state.cmd_packet[10+user_reg.length] = BYTE_FOOTER_1;
	state.cmd_packet[11+user_reg.length] = BYTE_FOOTER_2;
}

void IIM4623x_Set_UtcTime(utc time)
{
	if (state.sync_config == SYNC_WITH_PPS) {
		for (uint32_t i = 0; i < SIZE_PACKET_CMD; i++) state.cmd_packet[i] = 0x00;
		state.cmd_packet[0] = BYTE_HEADER_CMD;
		state.cmd_packet[1] = BYTE_HEADER_CMD;
		state.cmd_packet[2] = SIZE_CMD_SET_UTC_TIME;
		state.cmd_packet[3] = CMD_TYPE_SET_UTC_TIME;
		state.cmd_packet[4] = (uint8_t)(time.year >> 8);
		state.cmd_packet[5] = (uint8_t)(time.year &= 0x00FF);
		state.cmd_packet[6] = time.month;
		state.cmd_packet[7] = time.day;
		state.cmd_packet[8] = time.hh;
		state.cmd_packet[9] = time.mm;
		state.cmd_packet[10] = time.ss;
		uint16_t checksum = calc_checksum(&state.cmd_packet[3], 8);
		state.cmd_packet[11] = (uint8_t)(checksum >> 8);
		state.cmd_packet[12] = (uint8_t)(checksum &= 0x00FF);
		state.cmd_packet[13] = BYTE_FOOTER_1;
		state.cmd_packet[14] = BYTE_FOOTER_2;

		if (state.mode == COMMAND) {
			if (IIM4623x_serif_tx(state.cmd_packet, SIZE_PACKET_CMD))
				printf("Write error \n\r");
			else {
				state.utc_time = time;
				printf("UTC time was set \n\r");
			}
		}
		else {
			uint32_t read_length = 0;
			set_read_length(&read_length);

			if (IIM4623x_serif_tx_rx(state.cmd_packet, SIZE_PACKET_CMD, rbuf_data, read_length))
				printf("Write & Read error \n\r");
			else
				printf("UTC time was set in streaming mode \n\r");
		}
	}
	else // DISABLE_SYNC
		printf("PPS is not enabled \n\r");
}

void IIM4623x_Read_WhoAmI(void)
{
	IIM4623x_SetCMD_ReadRegister(WHO_AM_I);

	if (IIM4623x_serif_tx(state.cmd_packet, SIZE_PACKET_CMD)) {
		printf("Write error \n\r");
	}
}

void IIM4623x_Get_SerialNum(void)
{
	IIM4623x_SetCMD_Common(CMD_TYPE_GET_SERIAL_NUM);

	if (IIM4623x_serif_tx(state.cmd_packet, SIZE_PACKET_CMD)) {
		printf("Write error \n\r");
	}
}

void IIM4623x_Get_Version(void)
{
	IIM4623x_SetCMD_Common(CMD_TYPE_GET_VERSION);

	if (IIM4623x_serif_tx(state.cmd_packet, SIZE_PACKET_CMD)) {
		printf("Write error \n\r");
	}
}

void IIM4623x_Imu_SelfTest(void)
{
	IIM4623x_SetCMD_Common(CMD_TYPE_SELF_TEST);

	if (IIM4623x_serif_tx(state.cmd_packet, SIZE_PACKET_CMD)) {
		printf("Write error \n\r");
	}
}

void IIM4623x_Set_OutDataForm(enum IIM4623x_OutDataForm form)
{
	IIM4623x_SetCMD_WriteRegister(OUT_DATA_FORM, (uint8_t *)&form);

	if (IIM4623x_serif_tx(state.cmd_packet, SIZE_PACKET_CMD))
		printf("Write error \n\r");
	else
		state.data_form = form;
}

void IIM4623x_Set_SampleRateDiv(enum IIM4623x_SampleRateDiv divisor)
{
	IIM4623x_SetCMD_WriteRegister(SAMPLE_RATE_DIV, (uint8_t *)&divisor);

	if (IIM4623x_serif_tx(state.cmd_packet, SIZE_PACKET_CMD))
		printf("Write error \n\r");
	else
		state.rate = divisor;
}

void IIM4623x_Toggle_DataOutput(enum IIM4623x_DataOutPut output)
{
	switch (output) {
	case ACCEL:
		state.data_out ^= BIT_SELECT_OUT_DATA_ACC;
		break;
	case GYRO:
		state.data_out ^= BIT_SELECT_OUT_DATA_GYRO;
		break;
	case TEMP:
		state.data_out ^= BIT_SELECT_OUT_DATA_TEMP;
		break;
	case DELTA_VEL:
		state.data_out ^= BIT_SELECT_OUT_DATA_VEL;
		break;
	case DELTA_ANG:
		state.data_out ^= BIT_SELECT_OUT_DATA_ANG;
		break;
	default:
		break;
	}

	IIM4623x_SetCMD_WriteRegister(SELECT_OUT_DATA, (uint8_t *)&state.data_out);

	if (IIM4623x_serif_tx(state.cmd_packet, SIZE_PACKET_CMD)) {
		printf("Write error \n\r");
	}
}

void IIM4623x_Set_UartIfConfig(enum IIM4623x_UartBaudRate baud)
{
	IIM4623x_SetCMD_WriteRegister(UART_IF_CONFIG, (uint8_t *)&baud);

	if (IIM4623x_serif_tx(state.cmd_packet, SIZE_PACKET_CMD))
		printf("Write error \n\r");
	else
		state.baud_rate = baud;
}

void IIM4623x_Set_SyncConfig(enum IIM4623x_SyncConfig sync_mode)
{
	IIM4623x_SetCMD_WriteRegister(SYNC_CONFIG, (uint8_t *)&sync_mode);

	if (IIM4623x_serif_tx(state.cmd_packet, SIZE_PACKET_CMD))
		printf("Write error \n\r");
	else
		state.sync_config = sync_mode;	
}

void IIM4623x_Set_BWConfig_Accel(enum IIM4623x_AccBwConfig bw)
{
	state.lpf_bw &= 0x0f;
	state.lpf_bw |= bw;
	IIM4623x_SetCMD_WriteRegister(BW_CONFIG, (uint8_t *)&state.lpf_bw);

	if (IIM4623x_serif_tx(state.cmd_packet, SIZE_PACKET_CMD)) {
		printf("Write error \n\r");
	}
}

void IIM4623x_Set_BWConfig_Gyro(enum IIM4623x_GyroBwConfig bw)
{
	state.lpf_bw &= 0xf0;
	state.lpf_bw |= bw;

	IIM4623x_SetCMD_WriteRegister(BW_CONFIG, (uint8_t *)&state.lpf_bw);

	if (IIM4623x_serif_tx(state.cmd_packet, SIZE_PACKET_CMD)) {
		printf("Write error \n\r");
	}
}

void IIM4623x_Set_AccelConfig0(enum IIM4623x_AccelConfig0 fsr)
{
	if (state.part_num == IIM46234) {
		if (fsr == ACC_FSR_16G) {
			printf("Does not support this FSR \n\r");
			return;
		}
	}

	state.accel_fsr &= 0x1f;
	state.accel_fsr |= fsr;
	IIM4623x_SetCMD_WriteRegister(ACCEL_CONFIG0, (uint8_t *)&state.accel_fsr);

	if (IIM4623x_serif_tx(state.cmd_packet, SIZE_PACKET_CMD)) {
		printf("Write error \n\r");
	}
}

void IIM4623x_Set_GyroConfig0(enum IIM4623x_GyroConfig0 fsr)
{
	if (state.part_num == IIM46234) {
		if ((fsr == GYRO_FSR_2000DPS) || (fsr == GYRO_FSR_2000DPS)) {
			printf("Does not support this FSR \n\r");
			return;
		}
	}

	state.gyro_fsr &= 0x1f;
	state.gyro_fsr |= fsr;
	IIM4623x_SetCMD_WriteRegister(GYRO_CONFIG0, (uint8_t *)&state.gyro_fsr);

	if (IIM4623x_serif_tx(state.cmd_packet, SIZE_PACKET_CMD)) {
		printf("Write error \n\r");
	}
}

void IIM4623x_Toggle_ExtCalibConfig(enum IIM4623x_CalibConfig config)
{
	switch (config) {
	case ACCEL_BIAS:
		state.calib_config ^= BIT_ENABLE_EXT_ACCEL_BIAS;
		break;
	case GYRO_BIAS:
		state.calib_config ^= BIT_ENABLE_EXT_GYRO_BIAS;
		break;
	case ACCEL_SENS:
		state.calib_config ^= BIT_ENABLE_EXT_ACCEL_SENS;
		break;
	case GYRO_SENS:
		state.calib_config ^= BIT_ENABLE_EXT_GYRO_SENS;
		break;
	default:
		break;
	}

	if (state.calib_config & BIT_ENABLE_EXT_ACCEL_BIAS)
		printf("Accel bias calib. is enabled \n\r");
	if (state.calib_config & BIT_ENABLE_EXT_GYRO_BIAS)
		printf("Gyro bias calib. is enabled \n\r");
	if (state.calib_config & BIT_ENABLE_EXT_ACCEL_SENS)
		printf("Accel Sens. Mat. calib. is enabled \n\r");
	if (state.calib_config & BIT_ENABLE_EXT_GYRO_SENS)
		printf("Gyro Sens. Mat. calib. is enabled \n\r");

	IIM4623x_SetCMD_WriteRegister(EXT_CALIB_CONFIG, (uint8_t *)&state.calib_config);

	if (IIM4623x_serif_tx(state.cmd_packet, SIZE_PACKET_CMD)) {
		printf("Write error \n\r");
	}

}

void IIM4623x_Write_Bias(enum IIM4623x_Axis axis, float bias_val)
{
	reg bias_reg;
	uint32_t ul_bias_val;
	uint8_t c_bias_val[4];

	ul_bias_val = *((uint32_t*)&bias_val);
	
	c_bias_val[0] = (uint8_t)((ul_bias_val&0xff000000)>>24);
	c_bias_val[1] = (uint8_t)((ul_bias_val&0x00ff0000)>>16);
	c_bias_val[2] = (uint8_t)((ul_bias_val&0x0000ff00)>>8);
	c_bias_val[3] = (uint8_t)(ul_bias_val&0x000000ff);
	
	switch (axis) {
	case ACCEL_X:
		bias_reg = EXT_ACCEL_X_BIAS;
		break;
	case ACCEL_Y:
		bias_reg = EXT_ACCEL_Y_BIAS;
		break;
	case ACCEL_Z:
		bias_reg = EXT_ACCEL_Z_BIAS;
		break;
	case GYRO_X:
		bias_reg = EXT_GYRO_X_BIAS;
		break;
	case GYRO_Y:
		bias_reg = EXT_GYRO_Y_BIAS;
		break;
	case GYRO_Z:
		bias_reg = EXT_GYRO_Z_BIAS;
		break;	
	default:
		break;
	}

	IIM4623x_SetCMD_WriteRegister(bias_reg, &c_bias_val[0]);

	if (IIM4623x_serif_tx(state.cmd_packet, SIZE_PACKET_CMD)) {
		printf("Write error \n\r");
	}	
}

void IIM4623x_Read_Bias(enum IIM4623x_Axis axis)
{
	reg bias_reg;

	switch (axis) {
	case ACCEL_X:
		bias_reg = EXT_ACCEL_X_BIAS;
		break;
	case ACCEL_Y:
		bias_reg = EXT_ACCEL_Y_BIAS;
		break;
	case ACCEL_Z:
		bias_reg = EXT_ACCEL_Z_BIAS;
		break;
	case GYRO_X:
		bias_reg = EXT_GYRO_X_BIAS;
		break;
	case GYRO_Y:
		bias_reg = EXT_GYRO_Y_BIAS;
		break;
	case GYRO_Z:
		bias_reg = EXT_GYRO_Z_BIAS;
		break;	
	default:
		break;
	}

	IIM4623x_SetCMD_ReadRegister(bias_reg);

	if (IIM4623x_serif_tx(state.cmd_packet, SIZE_PACKET_CMD)) {
		printf("Write error \n\r");
	}	
}

void IIM4623x_Write_Sens_Mat_Accel(enum IIM4623x_Mat_Index indx, float value)
{
	reg sens_mat_reg;

	uint32_t ul_val;
	uint8_t c_val[4];

	ul_val = *((uint32_t*)&value);
	
	c_val[0] = (uint8_t)((ul_val&0xff000000)>>24);
	c_val[1] = (uint8_t)((ul_val&0x00ff0000)>>16);
	c_val[2] = (uint8_t)((ul_val&0x0000ff00)>>8);
	c_val[3] = (uint8_t)(ul_val&0x000000ff);

	switch (indx) {
	case X_1:
		sens_mat_reg = EXT_ACC_SENS_MAT11;
		break;
	case X_2:
		sens_mat_reg = EXT_ACC_SENS_MAT12;
		break;
	case X_3:
		sens_mat_reg = EXT_ACC_SENS_MAT13;
		break;
	case Y_1:
		sens_mat_reg = EXT_ACC_SENS_MAT21;
		break;
	case Y_2:
		sens_mat_reg = EXT_ACC_SENS_MAT22;
		break;
	case Y_3:
		sens_mat_reg = EXT_ACC_SENS_MAT23;
		break;
	case Z_1:
		sens_mat_reg = EXT_ACC_SENS_MAT31;
		break;
	case Z_2:
		sens_mat_reg = EXT_ACC_SENS_MAT32;
		break;
	case Z_3:
		sens_mat_reg = EXT_ACC_SENS_MAT33;
		break;
	default:
		break;
	}

	IIM4623x_SetCMD_WriteRegister(sens_mat_reg, c_val);

	if (IIM4623x_serif_tx(state.cmd_packet, SIZE_PACKET_CMD)) {
		printf("Write error \n\r");
	}	
}

void IIM4623x_Write_Sens_Mat_Gyro(enum IIM4623x_Mat_Index indx, float value)
{
	reg sens_mat_reg;

	uint32_t ul_val;
	uint8_t c_val[4];

	ul_val = *((uint32_t*)&value);
	
	c_val[0] = (uint8_t)((ul_val&0xff000000)>>24);
	c_val[1] = (uint8_t)((ul_val&0x00ff0000)>>16);
	c_val[2] = (uint8_t)((ul_val&0x0000ff00)>>8);
	c_val[3] = (uint8_t)(ul_val&0x000000ff);

	switch (indx) {
	case X_1:
		sens_mat_reg = EXT_GYR_SENS_MAT11;
		break;
	case X_2:
		sens_mat_reg = EXT_GYR_SENS_MAT12;
		break;
	case X_3:
		sens_mat_reg = EXT_GYR_SENS_MAT13;
		break;
	case Y_1:
		sens_mat_reg = EXT_GYR_SENS_MAT21;
		break;
	case Y_2:
		sens_mat_reg = EXT_GYR_SENS_MAT22;
		break;
	case Y_3:
		sens_mat_reg = EXT_GYR_SENS_MAT23;
		break;
	case Z_1:
		sens_mat_reg = EXT_GYR_SENS_MAT31;
		break;
	case Z_2:
		sens_mat_reg = EXT_GYR_SENS_MAT32;
		break;
	case Z_3:
		sens_mat_reg = EXT_GYR_SENS_MAT33;
		break;
	default:
		break;
	}

	IIM4623x_SetCMD_WriteRegister(sens_mat_reg, c_val);

	if (IIM4623x_serif_tx(state.cmd_packet, SIZE_PACKET_CMD)) {
		printf("Write error \n\r");
	}	
}

void IIM4623x_Read_Sens_Mat_Accel(enum IIM4623x_Mat_Index indx)
{
	reg sens_mat_reg;

	switch (indx) {
	case X_1:
		sens_mat_reg = EXT_ACC_SENS_MAT11;
		break;
	case X_2:
		sens_mat_reg = EXT_ACC_SENS_MAT12;
		break;
	case X_3:
		sens_mat_reg = EXT_ACC_SENS_MAT13;
		break;
	case Y_1:
		sens_mat_reg = EXT_ACC_SENS_MAT21;
		break;
	case Y_2:
		sens_mat_reg = EXT_ACC_SENS_MAT22;
		break;
	case Y_3:
		sens_mat_reg = EXT_ACC_SENS_MAT23;
		break;
	case Z_1:
		sens_mat_reg = EXT_ACC_SENS_MAT31;
		break;
	case Z_2:
		sens_mat_reg = EXT_ACC_SENS_MAT32;
		break;
	case Z_3:
		sens_mat_reg = EXT_ACC_SENS_MAT33;
		break;
	default:
		break;
	}

	IIM4623x_SetCMD_ReadRegister(sens_mat_reg);

	if (IIM4623x_serif_tx(state.cmd_packet, SIZE_PACKET_CMD)) {
		printf("Write error \n\r");
	}	
}

void IIM4623x_Read_Sens_Mat_Gyro(enum IIM4623x_Mat_Index indx)
{
	reg sens_mat_reg;

	switch (indx) {
	case X_1:
		sens_mat_reg = EXT_GYR_SENS_MAT11;
		break;
	case X_2:
		sens_mat_reg = EXT_GYR_SENS_MAT12;
		break;
	case X_3:
		sens_mat_reg = EXT_GYR_SENS_MAT13;
		break;
	case Y_1:
		sens_mat_reg = EXT_GYR_SENS_MAT21;
		break;
	case Y_2:
		sens_mat_reg = EXT_GYR_SENS_MAT22;
		break;
	case Y_3:
		sens_mat_reg = EXT_GYR_SENS_MAT23;
		break;
	case Z_1:
		sens_mat_reg = EXT_GYR_SENS_MAT31;
		break;
	case Z_2:
		sens_mat_reg = EXT_GYR_SENS_MAT32;
		break;
	case Z_3:
		sens_mat_reg = EXT_GYR_SENS_MAT33;
		break;
	default:
		break;
	}

	IIM4623x_SetCMD_ReadRegister(sens_mat_reg);

	if (IIM4623x_serif_tx(state.cmd_packet, SIZE_PACKET_CMD)) {
		printf("Write error \n\r");
	}	
}

float_uint_t* u32_sample[13];
uint32_t g_ul_out_num = 0;

void set_read_length(uint32_t *length)
{
	*length = SIZE_PACKET_BASE_DATA + 4*g_ul_out_num;
}

void IIM4623x_Start_Streaming(void)
{
	if (state.data_out == 0) {
		printf("At least one output should be selected \n\r");
	}
	else {
		if (state.data_out & BIT_SELECT_OUT_DATA_ACC) {
			u32_sample[g_ul_out_num] = &state.acc_x;
			g_ul_out_num++;
			u32_sample[g_ul_out_num] = &state.acc_y;
			g_ul_out_num++;
			u32_sample[g_ul_out_num] = &state.acc_z;
			g_ul_out_num++;			
		}

		if (state.data_out & BIT_SELECT_OUT_DATA_GYRO) {
			u32_sample[g_ul_out_num] = &state.gyro_x;
			g_ul_out_num++;
			u32_sample[g_ul_out_num] = &state.gyro_y;
			g_ul_out_num++;
			u32_sample[g_ul_out_num] = &state.gyro_z;
			g_ul_out_num++;			
		}
		
		if (state.data_out & BIT_SELECT_OUT_DATA_TEMP) {
			u32_sample[g_ul_out_num] = &state.temp;
			g_ul_out_num++;
		}

		if (state.data_out & BIT_SELECT_OUT_DATA_VEL) {
			u32_sample[g_ul_out_num] = &state.d_vel_x;
			g_ul_out_num++;
			u32_sample[g_ul_out_num] = &state.d_vel_y;
			g_ul_out_num++;
			u32_sample[g_ul_out_num] = &state.d_vel_z;
			g_ul_out_num++;			
		}
		
		if (state.data_out & BIT_SELECT_OUT_DATA_ANG) {
			u32_sample[g_ul_out_num] = &state.d_ang_x;
			g_ul_out_num++;
			u32_sample[g_ul_out_num] = &state.d_ang_y;
			g_ul_out_num++;
			u32_sample[g_ul_out_num] = &state.d_ang_z;
			g_ul_out_num++;			
		}

		if (state.sensorft_just_toggled == true) {
			state.sensorft_just_toggled = false;
			delay_ms(500);
		}

		IIM4623x_SetCMD_Common(CMD_TYPE_START_STREAMING);

		if (IIM4623x_serif_tx(state.cmd_packet, SIZE_PACKET_CMD))
			printf("Write error \n\r");
		else {
			state.mode = STREAMING;
			printf("Sent command to start streaming \n\r");
		}
	}
}

uint32_t IIM4623x_Stop_Streaming(void)
{
	uint32_t read_length = 0;

	IIM4623x_SetCMD_Common(CMD_TYPE_STOP_STREAMING);

	set_read_length(&read_length);

	if (IIM4623x_serif_tx_rx(state.cmd_packet, SIZE_PACKET_CMD, rbuf_data, read_length)) {
		printf("Write & read error \n\r");
		return 1;
	}
	else {
		state.mode = COMMAND;
#if DEBUG_PHASE
		for (uint32_t i=0; i<read_length; i++)
			printf("%x ", rbuf_data[i]);
		printf("\n\r");
#endif			
		g_ul_out_num = 0;

		if (enable_log == 0)
			count_data_out = 0;
	}

	return 0;
}

void IIM4623x_Set_SensorFT(bool on_off)
{
	if (state.sensorft_enabled == on_off) {
		if (state.sensorft_enabled == true)
			printf("SensorFT was already enabled \n\r");
		else // state.sensorft_enabled == false
			printf("SensorFT was already disabled \n\r");
	}
	else {
		uint8_t cmd_type;

	    cmd_type =
	      (on_off == true)? CMD_TYPE_ENABLE_SENSORFT : CMD_TYPE_DISABLE_SENSORFT;

		IIM4623x_SetCMD_Common(cmd_type);
		
		if (IIM4623x_serif_tx(state.cmd_packet, SIZE_PACKET_CMD)) {
			printf("Write error \n\r");
		}

		state.sensorft_enabled = (on_off == true)? true : false;
		state.sensorft_just_toggled = true;
	}
}

inline uint32_t read_u32_data(uint8_t *p_buf)
{
	uint32_t value;
	value = ((uint32_t)(*p_buf)<<24 | 
		(uint32_t)(*(p_buf+1))<<16 | 
		(uint32_t)(*(p_buf+2))<<8 | 
		(uint32_t)(*(p_buf+3)));

	return value;
}

uint32_t IIM4623x_Read_DataOutput(void)
{
	uint32_t read_length = 0;
	uint8_t sensor_status = 0;	
	uint8_t sample_counter = 0;
	uint64_t ts_device = 0;

	set_read_length(&read_length);

	if (IIM4623x_serif_rx(rbuf_data, read_length)) {
		printf("Read error \n\r");
		return 1;		
	}

#if DEBUG_PHASE
	for (uint32_t i=0; i<read_length; i++)
		printf("%x ", rbuf_data[i]);
	printf("\n\r");
#endif

	uint16_t checksum_read = rbuf_data[read_length-4]<<8 | rbuf_data[read_length-3];
	uint16_t checksum = calc_checksum(&rbuf_data[3], (read_length - 7));	
	if (checksum != checksum_read) {
		printf("Incorrect checksum (read data) %d %d \n\r", checksum_read, checksum);
		return 1;
	}

	sensor_status = rbuf_data[4];
	sample_counter = rbuf_data[5];
	ts_device = ((uint64_t)rbuf_data[6]<<56 |
		(uint64_t)rbuf_data[7]<<48 |
		(uint64_t)rbuf_data[8]<<40 |
		(uint64_t)rbuf_data[9]<<32 |
		(uint64_t)rbuf_data[10]<<24 |
		(uint64_t)rbuf_data[11]<<16 |
		(uint64_t)rbuf_data[12]<<8 |
		(uint64_t)rbuf_data[13]);

	if (enable_log == 1)
#if DBOX_STYLE_DATA_LOGGING
		printf("%d, %lld, ", sample_counter, ts_device);
#else
		printf("[%lld] ts %lld # %d ", timestamp, ts_device, sample_counter);
#endif
		
	if (state.data_form == FLOATING) {
		uint32_t sample_val;
		for (uint32_t i=0; i<g_ul_out_num; i++) {
			sample_val = read_u32_data(&rbuf_data[14+4*i]);
			u32_sample[i]->val = *((float*)&sample_val);
		}

		if (enable_log == 1) {
#if DBOX_STYLE_DATA_LOGGING
			if (state.data_out & BIT_SELECT_OUT_DATA_ACC)
				printf("%f, %f, %f, ", state.acc_x.val, state.acc_y.val, state.acc_z.val);
			if (state.data_out & BIT_SELECT_OUT_DATA_GYRO)
				printf("%f, %f, %f, ", state.gyro_x.val, state.gyro_y.val, state.gyro_z.val);
			if (state.data_out & BIT_SELECT_OUT_DATA_TEMP)
				printf("%f, ", state.temp.val);
			if (state.data_out & BIT_SELECT_OUT_DATA_VEL)
				printf("%f, %f, %f, ", state.d_vel_x.val, state.d_vel_y.val, state.d_vel_z.val);
			if (state.data_out & BIT_SELECT_OUT_DATA_ANG)
				printf("%f, %f, %f, ", state.d_ang_x.val, state.d_ang_y.val, state.d_ang_z.val);
#else
			if (state.data_out & BIT_SELECT_OUT_DATA_ACC)
				printf("(%d) a_x: %f a_y: %f a_z: %f ", (sensor_status&0xE0)>>5, state.acc_x.val, state.acc_y.val, state.acc_z.val);
			if (state.data_out & BIT_SELECT_OUT_DATA_GYRO)
				printf("(%d) g_x: %f g_y: %f g_z: %f ", (sensor_status&0x1F), state.gyro_x.val, state.gyro_y.val, state.gyro_z.val);
			if (state.data_out & BIT_SELECT_OUT_DATA_TEMP)
				printf("temp: %f ", state.temp.val);
			if (state.data_out & BIT_SELECT_OUT_DATA_VEL)
				printf("d_vel_x: %f d_vel_y: %f d_vel_z: %f ", state.d_vel_x.val, state.d_vel_y.val, state.d_vel_z.val);
			if (state.data_out & BIT_SELECT_OUT_DATA_ANG)
				printf("d_ang_x: %f d_ang_y: %f d_ang_z: %f ", state.d_ang_x.val, state.d_ang_y.val, state.d_ang_z.val);
#endif
		}
	}
	else if (state.data_form == FIXED) {
		for (uint32_t i=0; i<g_ul_out_num; i++)
			u32_sample[i]->u32 = read_u32_data(&rbuf_data[14+4*i]);

		if (enable_log == 1) {
#if DBOX_STYLE_DATA_LOGGING
			if (state.data_out & BIT_SELECT_OUT_DATA_ACC)
				printf("%ld, %ld, %ld, ", state.acc_x.u32, state.acc_y.u32, state.acc_z.u32);
			if (state.data_out & BIT_SELECT_OUT_DATA_GYRO)
				printf("%ld, %ld, %ld, ", state.gyro_x.u32, state.gyro_y.u32, state.gyro_z.u32);
			if (state.data_out & BIT_SELECT_OUT_DATA_TEMP)
				printf("%ld, ", state.temp.u32);
			if (state.data_out & BIT_SELECT_OUT_DATA_VEL)
				printf("%ld, %ld, %ld, ", state.d_vel_x.u32, state.d_vel_y.u32, state.d_vel_z.u32);
			if (state.data_out & BIT_SELECT_OUT_DATA_ANG)
				printf("%ld, %ld, %ld, ", state.d_ang_x.u32, state.d_ang_y.u32, state.d_ang_z.u32);
#else
			if (state.data_out & BIT_SELECT_OUT_DATA_ACC)
				printf("(%d) a_x: %ld a_y: %ld a_z: %ld ", (sensor_status&0xE0)>>5, state.acc_x.u32, state.acc_y.u32, state.acc_z.u32);
			if (state.data_out & BIT_SELECT_OUT_DATA_GYRO)
				printf("(%d) g_x: %ld g_y: %ld g_z: %ld ", (sensor_status&0x1F), state.gyro_x.u32, state.gyro_y.u32, state.gyro_z.u32);
			if (state.data_out & BIT_SELECT_OUT_DATA_TEMP)
				printf("temp: %ld ", state.temp.u32);
			if (state.data_out & BIT_SELECT_OUT_DATA_VEL)
				printf("d_vel_x: %ld d_vel_y: %ld d_vel_z: %ld ", state.d_vel_x.u32, state.d_vel_y.u32, state.d_vel_z.u32);
			if (state.data_out & BIT_SELECT_OUT_DATA_ANG)
				printf("d_ang_x: %ld d_ang_y: %ld d_ang_z: %ld ", state.d_ang_x.u32, state.d_ang_y.u32, state.d_ang_z.u32);
#endif
		}
	}
	else {
		printf("Does not support this type of data form \n\r");
		return 1;
	}	

	if (enable_log == 1)
#if DBOX_STYLE_DATA_LOGGING
		printf("%d\n\r", sensor_status);
#else
		printf("\n\r");
#endif
	
	if (enable_log == 0) {
		count_data_out++;
		if ((count_data_out%1000) == 0)
			printf("[%lld] ts: %lld count: %ld \n\r", timestamp, ts_device, count_data_out);
	}

	return 0;
}

bool Is_Command_Mode(void)
{
	if (state.mode == COMMAND)
		return true;
	else
		return false;
}

void IIM4623x_Set_StopStreaming(void)
{
	state.stp_cmd_in_streaming = true;
}

void IIM4623x_Set_StartStreaming(void)
{
	state.stp_cmd_in_streaming = false;
}

void IIM4623x_Set_UtcTimeInStreaming(void)
{
	printf("\r\n");
	printf("Setting UTC time begins.. \r\n");
	
	printf("Enter year (yyyy) and then press 'Enter': ");
	scanf("%hu", &state.utc_time.year);
	printf("%hu \r\n", state.utc_time.year);
	
	printf("Enter month (mm) and then press 'Enter': ");
	scanf("%hu", &state.utc_time.month);
	printf("%hu \r\n", state.utc_time.month);
	
	printf("Enter day (dd) and then press 'Enter': ");
	scanf("%hu", &state.utc_time.day);
	printf("%hu \r\n", state.utc_time.day);
	
	printf("Enter hour (hh) and then press 'Enter': ");
	scanf("%hu", &state.utc_time.hh);
	printf("%hu \r\n", state.utc_time.hh);
	
	printf("Enter minute (mm) and then press 'Enter': ");
	scanf("%hu", &state.utc_time.mm);
	printf("%hu \r\n", state.utc_time.mm);
	
	printf("Enter second (ss) and then press 'Enter': ");
	scanf("%hu", &state.utc_time.ss);
	printf("%hu \r\n", state.utc_time.ss);
	printf("\r\n");	

	state.utc_cmd_in_streaming = true;
}

uint32_t response_handler(void)
{
	uint32_t rSize = 0;
	uint32_t i;
	uint8_t cmd_type;
	uint16_t checksum_read;
	uint16_t checksum;

	cmd_type = state.cmd_packet[3];
	if (Is_Command_Mode()) {
  		switch(cmd_type) {
		case CMD_TYPE_SWITCH_TO_BOOTLOADER:
			rSize = SIZE_RESP_ACK;
  		   
			if (IIM4623x_serif_rx(rbuf_resp, rSize)) {
				printf("Read error \n\r");
				return 1;
			}
  
			checksum_read = rbuf_resp[rSize-4]<<8 | rbuf_resp[rSize-3];
			checksum = calc_checksum(&rbuf_resp[3], (rSize-7)); 
			if (checksum != checksum_read) {
				printf("Incorrect checksum (write register) %d %d \n\r", checksum_read, checksum);
				return 1;
			}
  		   
			if (rbuf_resp[3] == CMD_TYPE_SWITCH_TO_BOOTLOADER) {
				if (rbuf_resp[4] == ERR_CODE_SUCCESS) {
					printf("Switched to bootloader successfully \n\r");
					delay_ms(1000);
				    IIM4623x_Upgrade_Firmware();
				}
				else {
					printf("Switching to bootloader was not successful \n\r");
					return 1;
				}
			}
			else {
				printf("Type of response does not match (CMD Type: 0x%x) \n\r", CMD_TYPE_WRITE_REG);
			}
			break;
		case CMD_TYPE_UPGRADE_FIRMWARE:
			rSize = SIZE_RESP_ACK;
  		   
			if (IIM4623x_serif_rx(rbuf_resp, rSize)) {
				printf("Read error \n\r");
				return 1;
			}
  
			checksum_read = rbuf_resp[rSize-4]<<8 | rbuf_resp[rSize-3];
			checksum = calc_checksum(&rbuf_resp[3], (rSize-7)); 
			if (checksum != checksum_read) {
				printf("Incorrect checksum (write register) %d %d \n\r", checksum_read, checksum);
				return 1;
			}
  		   
			if (rbuf_resp[3] == CMD_TYPE_UPGRADE_FIRMWARE) {
				if (rbuf_resp[4] == ERR_CODE_SUCCESS) {
					printf("UPGRADE_FIRMWARE was received (Bootloader Version: 0x%x) \n\r", rbuf_resp[5]);
					
					if (state.fw_sending_done == true) {
						printf("FW was already updated \n\r");
						IIM4623x_Clear_Upgrade_Flag();
					}
					else {
						delay_ms(10);
						IIM4623x_Send_Length_Info();
					}
				}
				else {
					printf("UPGRADE_FIRMWARE was not received well \n\r");
					return 1;
				}
			}
			else {
				printf("Type of response does not match (CMD Type: 0x%x) \n\r", CMD_TYPE_WRITE_REG);
			}
			break;
		case CMD_TYPE_LENGTH_INFO:
			rSize = SIZE_RESP_ACK;
  		   
			if (IIM4623x_serif_rx(rbuf_resp, rSize)) {
				printf("Read error \n\r");
				return 1;
			}

			checksum_read = rbuf_resp[rSize-4]<<8 | rbuf_resp[rSize-3];
			checksum = calc_checksum(&rbuf_resp[3], (rSize-7)); 
			if (checksum != checksum_read) {
				printf("Incorrect checksum (write register) %d %d \n\r", checksum_read, checksum);
				return 1;
			}
  		   
			if (rbuf_resp[3] == CMD_TYPE_LENGTH_INFO) {
				if (rbuf_resp[4] == ERR_CODE_SUCCESS) {
					if (state.fw_sent_data == 0)
						printf("Bootloader received Lenth Info successfully \n\r");
					else
						printf("Bootloader received image data packet successfully (%d%%) \n\r", (int)(state.fw_sent_data*100/state.fw_size));
					
				    if (state.fw_sending_done == false) {
				        delay_ms(10);
				        IIM4623x_Send_Image_Data();
				    }
				}
				else {
					if (state.fw_sent_data == 0)
					    printf("Length Info was not received well by Bootloader \n\r");
					else
						printf("Image data packet was not received well by Bootloader \n\r");
					return 1;
				}
			}
			else {
				printf("Type of response does not match (CMD Type: 0x%x) \n\r", CMD_TYPE_WRITE_REG);
			}
			break;
		case CMD_TYPE_CLEAR_UPGRADE_FLAG:
			rSize = SIZE_RESP_ACK;
  		   
			if (IIM4623x_serif_rx(rbuf_resp, rSize)) {
				printf("Read error \n\r");
				return 1;
			}
  
			checksum_read = rbuf_resp[rSize-4]<<8 | rbuf_resp[rSize-3];
			checksum = calc_checksum(&rbuf_resp[3], (rSize-7)); 
			if (checksum != checksum_read) {
				printf("Incorrect checksum (write register) %d %d \n\r", checksum_read, checksum);
				return 1;
			}
  		   
			if (rbuf_resp[3] == CMD_TYPE_CLEAR_UPGRADE_FLAG) {
				if (rbuf_resp[4] == ERR_CODE_SUCCESS) {
					printf("Upgrade flag was cleared successfully \n\r");
				}
				else {
					printf("Upgrade flag was not cleared \n\r");
					return 1;
				}
			}
			else {
				printf("Type of response does not match (CMD Type: 0x%x) \n\r", CMD_TYPE_WRITE_REG);
			}
			break;
  		case CMD_TYPE_GET_SERIAL_NUM:
			rSize = SIZE_RESP_GET_SERIAL_NUM;

			if (IIM4623x_serif_rx(rbuf_resp, rSize)) {
				printf("Read error \n\r");
				return 1;
			}
  		   
			checksum_read = rbuf_resp[rSize-4]<<8 | rbuf_resp[rSize-3];
			checksum = calc_checksum(&rbuf_resp[3], (rSize-7)); 
			if (checksum != checksum_read) {
				printf("Incorrect checksum (get serial num) %d %d \n\r", checksum_read, checksum);
			}
  		   
			if (rbuf_resp[3] == CMD_TYPE_GET_SERIAL_NUM) {
				printf("Device Serial Number: 0x%x%x%x%x%x%x%x%x%x%x%x%x%x%x%x%x \n\r",
					rbuf_resp[6], rbuf_resp[7], rbuf_resp[8], rbuf_resp[9],
					rbuf_resp[10],rbuf_resp[11],rbuf_resp[12],rbuf_resp[13],
					rbuf_resp[14],rbuf_resp[15],rbuf_resp[16],rbuf_resp[17],
					rbuf_resp[18],rbuf_resp[19],rbuf_resp[20],rbuf_resp[21]);
			}
			else {
				printf("Type of response does not match (CMD Type: 0x%x) \n\r", CMD_TYPE_GET_SERIAL_NUM);
			}
			break;
		case CMD_TYPE_GET_VERSION:
			rSize = SIZE_RESP_GET_VERSION;
  	   
			if (IIM4623x_serif_rx(rbuf_resp, rSize)) {
				printf("Read error \n\r");
				return 1;
			}
  
			checksum_read = rbuf_resp[rSize-4]<<8 | rbuf_resp[rSize-3];
			checksum = calc_checksum(&rbuf_resp[3], (rSize-7)); 
			if (checksum != checksum_read) {
				printf("Incorrect checksum (get version) %d %d \n\r", checksum_read, checksum);
			}
  		   
			if (rbuf_resp[3] == CMD_TYPE_GET_VERSION) {
				printf("Version Info ... Major: %d Minor: %d\n\r", rbuf_resp[6], rbuf_resp[7]);
			}
			else {
				printf("Type of response does not match (CMD Type: 0x%x) \n\r", CMD_TYPE_GET_VERSION);
			}
			break;
		case CMD_TYPE_SELF_TEST:
			rSize = SIZE_RESP_IMU_SELF_TEST;

			if (IIM4623x_serif_rx(rbuf_resp, rSize)) {
				printf("Read error \n\r");
				return 1;
			}
  
			checksum_read = rbuf_resp[rSize-4]<<8 | rbuf_resp[rSize-3];
			checksum = calc_checksum(&rbuf_resp[3], (rSize-7)); 
			if (checksum != checksum_read) {
				printf("Incorrect checksum (self-test) %d %d \n\r", checksum_read, checksum);
			}
  		   
			if (rbuf_resp[3] == CMD_TYPE_SELF_TEST) {
				if ((rbuf_resp[6]  == SELF_TEST_RESULT_PASS) &&
					(rbuf_resp[7]  == SELF_TEST_RESULT_PASS) &&
					(rbuf_resp[8]  == SELF_TEST_RESULT_PASS) &&
					(rbuf_resp[9]  == SELF_TEST_RESULT_PASS) &&
					(rbuf_resp[10] == SELF_TEST_RESULT_PASS) &&
					(rbuf_resp[11] == SELF_TEST_RESULT_PASS)) {
					printf("Passed IMU Self-Test \n\r");
				}
				else {
					printf("Failed IMU Self-Test (Result: %d %d %d %d %d %d) \n\r",
						rbuf_resp[6], rbuf_resp[7], rbuf_resp[8], rbuf_resp[9], rbuf_resp[10], rbuf_resp[11]);
					return 1;
				}
			}
			else {
				printf("Type of response does not match (CMD Type: 0x%x) \n\r", CMD_TYPE_SELF_TEST);
			}
			break;
		case CMD_TYPE_SET_UTC_TIME:
			rSize = SIZE_RESP_ACK;
			printf("Note that RESPONE or ACK packets are not sent for Set UTC Time command \n\r");
			break;
		case CMD_TYPE_READ_REG:
			/* Length of base command + length of register */
			rSize = SIZE_RESP_READ_REGS_BASE + state.cmd_packet[5];
  	   
			if (IIM4623x_serif_rx(rbuf_resp, rSize)) {
  				printf("Read error \n\r");
  				return 1;
			}
  
			checksum_read = rbuf_resp[rSize-4]<<8 | rbuf_resp[rSize-3];
			checksum = calc_checksum(&rbuf_resp[3], (rSize-7)); 
			if (checksum != checksum_read) {
				printf("Incorrect checksum (read register) %d %d \n\r", checksum_read, checksum);
			}

			if (rbuf_resp[3] == CMD_TYPE_READ_REG) {
				if (rbuf_resp[9] == state.cmd_packet[5]) {
					printf("Contents of User Registers: ");
					for (i = 0; i < rbuf_resp[9]; i++) printf("0x%x ", rbuf_resp[i+12]);
					printf("\n\r");
				}
				else {
					printf("Read length does not match (Read length Info. in CMD: %d) \n\r", state.cmd_packet[5]);
				}
			}
			else {
				printf("Type of response does not match (CMD Type: 0x%x) \n\r", CMD_TYPE_READ_REG);
			}
			break;
		case CMD_TYPE_WRITE_REG:
			rSize = SIZE_RESP_ACK;
  		   
			if (IIM4623x_serif_rx(rbuf_resp, rSize)) {
				printf("Read error \n\r");
				return 1;
			}
  
			checksum_read = rbuf_resp[rSize-4]<<8 | rbuf_resp[rSize-3];
			checksum = calc_checksum(&rbuf_resp[3], (rSize-7)); 
			if (checksum != checksum_read) {
				printf("Incorrect checksum (write register) %d %d \n\r", checksum_read, checksum);
				return 1;
			}
  		   
			if (rbuf_resp[3] == CMD_TYPE_WRITE_REG) {
				if (rbuf_resp[4] == ERR_CODE_SUCCESS) {
					printf("Writing registers was successful \n\r");
				}
				else {
					printf("Writing registers was not successful \n\r");
					return 1;
				}
			}
			else {
				printf("Type of response does not match (CMD Type: 0x%x) \n\r", CMD_TYPE_WRITE_REG);
			}

			if (bus_type == 1) {
				if (state.cmd_packet[6] == 0x1D) {
					switch (state.baud_rate) {
					case BAUD_921600:
						reconfigure_uart_baudrate(921600);
						printf("Changed UART Baud rate of the host board (921600) \n\r");		
						break;
					case BAUD_1500000:
						reconfigure_uart_baudrate(1500000);
						printf("Changed UART Baud rate of the host board (1500000) \n\r");
						break;
					case BAUD_3000000:
						reconfigure_uart_baudrate(3000000);
						printf("Changed UART Baud rate of the host board (3000000) \n\r");		
						break;
					default:
						break;
					}
				}
  			}
			break;
		case CMD_TYPE_ENABLE_SENSORFT:
			rSize = SIZE_RESP_ACK;
  		   
			if (IIM4623x_serif_rx(rbuf_resp, rSize)) {
				printf("Read error \n\r");
				return 1;
			}
  
			checksum_read = rbuf_resp[rSize-4]<<8 | rbuf_resp[rSize-3];
			checksum = calc_checksum(&rbuf_resp[3], (rSize-7)); 
			if (checksum != checksum_read) {
				printf("Incorrect checksum (write register) %d %d \n\r", checksum_read, checksum);
				return 1;
			}

			if (rbuf_resp[3] == CMD_TYPE_ENABLE_SENSORFT) {
				if (rbuf_resp[4] == ERR_CODE_SUCCESS) {
					printf("Enabling SensorFT was successful \n\r");
				}
				else {
					printf("Enabling SensorFT was not successful \n\r");
					return 1;
				}
			}
			else {
				printf("Type of response does not match (CMD Type: 0x%x) \n\r", CMD_TYPE_ENABLE_SENSORFT);
			}
			break;
		case CMD_TYPE_DISABLE_SENSORFT:
			rSize = SIZE_RESP_ACK;
  		   
			if (IIM4623x_serif_rx(rbuf_resp, rSize)) {
				printf("Read error \n\r");
				return 1;
			}
  
			checksum_read = rbuf_resp[rSize-4]<<8 | rbuf_resp[rSize-3];
			checksum = calc_checksum(&rbuf_resp[3], (rSize-7)); 
			if (checksum != checksum_read) {
				printf("Incorrect checksum (write register) %d %d \n\r", checksum_read, checksum);
				return 1;
			}

			if (rbuf_resp[3] == CMD_TYPE_DISABLE_SENSORFT) {
				if (rbuf_resp[4] == ERR_CODE_SUCCESS) {
					printf("Disabling SensorFT was successful \n\r");
				}
				else {
					printf("Disabling SensorFT was not successful \n\r");
					return 1;
				}
			}
			else {
				printf("Type of response does not match (CMD Type: 0x%x) \n\r", CMD_TYPE_DISABLE_SENSORFT);
			}
			break;
		default:
			break;
		}

#if DEBUG_PHASE
		for (i = 0; i < rSize; i++) {
			printf("IIM4623x response packet data (command mode) ... rbuf[%ld]: 0x%x \n\r", i, rbuf_resp[i]);
		}
#endif
	}
	else {
		if (state.stp_cmd_in_streaming == false) {
			if (state.utc_cmd_in_streaming == true) {
				IIM4623x_Set_UtcTime(state.utc_time);
				state.utc_cmd_in_streaming = false;
			}
			else {
				if (IIM4623x_Read_DataOutput()) {
 					printf("Data read error \n\r");
 					return 1;
				}				
			}
		}
		else { // state.stp_cmd_in_streaming == true
			if (!IIM4623x_Stop_Streaming()) {
				state.mode = COMMAND;
				printf("IIM4623x ... Sent command to stop streaming \n\r");
			}
			else {
				printf("Stop streaming error \n\r");
				return 1;
			}
		}
	}
 
	return 0;
}

