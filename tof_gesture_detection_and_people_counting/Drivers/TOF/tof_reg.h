/*
 * tof_reg.h
 *
 *  Created on: 06-Dec-2022
 *      Author: sukhdeep
 */

#ifndef TOF_TOF_REG_H_
#define TOF_TOF_REG_H_

#include "main.h"
#include "i2c.h"

extern uint8_t read_val, write_val;
extern uint32_t counter;
extern uint8_t result_buff[132];
/*I2C specific maros*/
#define TOF_I2C_ADDR	0x41 << 1
#define TOF_I2C_Handle	&hi2c2

/*Always available Register Definitions*/
#define TOF_APPID		0x00
#define TOF_MINOR		0x01
#define TOF_ENABLE		0xE0
#define TOF_INT_STATUS	0xE1
#define TOF_INT_ENAB	0xE2
#define TOF_ID			0xE3
#define TOF_REVID		0xE4


//appid=0x80 – Bootloader Registers
#define TOF_BL_CMD_STAT		0x08
#define TOF_BL_SIZE			0x09
#define TOF_BL_DATA			0x0A 	//Address 0x0A-0x8A  [bl_data0 ...bl_data127] can transfer 128 bytes at a time
#define TOF_BL_CSUM						//Address after bl_data* , If there is no databyte, BL_SUM address is 0x0A.

//Bootloader Commands
#define RAMREMAP_RESET		0x11		//Remap RAM to Address 0 and Reset
#define DOWNLOAD_INIT		0x14		//Initialize for RAM download from host to TMF8820/21/28
#define RAM_BIST			0x2A		//Build in self test of RAM (pattern test)
#define I2C_BIST			0x2C		//Build in self test of I 2 C RAM (pattern test)
#define W_RAM				0x41		//Write RAM Region (Plain = not encoded into e.g. Intel Hex Records)
#define ADDR_RAM			0x43		//Set the read/write RAM pointer to a given address

//appid=0x03, Main Application Registers
#define TOF_PATCH				0x02
#define TOF_BUILD_TYPE			0x03
#define TOF_APPLICATION_STATUS	0x04
#define TOF_MEASURE_STATUS		0x05
#define TOF_ALGORITHM_STATUS	0x06
#define TOF_CALIBRATION_STATUS	0x07
#define TOF_CMD_STAT			0x08
#define TOF_PREV_CMD			0x09
#define TOF_MODE				0x10
#define TOF_LIVE_BEAT			0x0A
#define TOF_LIVE_GPIO			0x0B
#define TOF_SERIAL_NUMBER_0		0x1C
#define TOF_SERIAL_NUMBER_1		0x1D
#define TOF_SERIAL_NUMBER_2		0x1E
#define TOF_SERIAL_NUMBER_3		0x1F
#define TOF_CONFIG_RESULT		0x20
#define TOF_TID					0x21
#define TOF_SIZE_LSB			0x22
#define TOF_SIZE_MSB			0x23

//appid=0x03, cid_rid=0x10 – Measurement Results Registers
#define RESULT_NUMBER			0x24
#define TEMPERATURE				0x25
#define NUMBER_VALID_RESULTS	0x26
#define AMBIENT_LIGHT_0			0x28
#define AMBIENT_LIGHT_1			0x29
#define AMBIENT_LIGHT_2			0x2A
#define AMBIENT_LIGHT_3			0x2B
#define PHOTON_COUNT_0			0x2C
#define PHOTON_COUNT_1			0x2D
#define PHOTON_COUNT_2			0x2E
#define PHOTON_COUNT_3			0x2F
#define REFERENCE_COUNT_0		0x30
#define REFERENCE_COUNT_1		0x31
#define REFERENCE_COUNT_2		0x32
#define REFERENCE_COUNT_3		0x33
#define SYS_TICK_0				0x34
#define SYS_TICK_1				0x35
#define SYS_TICK_2				0x36
#define SYS_TICK_3				0x37
#define RES_CONFIDENCE_0		0x38
#define RES_DISTANCE_0_LSB		0x39
#define RES_DISTANCE_0_MSB		0x3A
#define RES_CONFIDENCE_1		0x3B
#define RES_DISTANCE_1_LSB		0x3C
#define RES_DISTANCE_1_MSB		0x3D
#define RES_CONFIDENCE_2		0x3E
#define RES_DISTANCE_2_LSB		0x3F
#define RES_DISTANCE_2_MSB		0x40
#define RES_CONFIDENCE_3		0x41
#define RES_DISTANCE_3_LSB		0x42
#define RES_DISTANCE_3_MSB		0x43
#define RES_CONFIDENCE_4		0x44
#define RES_DISTANCE_4_LSB		0x45
#define RES_DISTANCE_4_MSB		0x46
#define RES_CONFIDENCE_5		0x47
#define RES_DISTANCE_5_LSB		0x48
#define RES_DISTANCE_5_MSB		0x49
#define RES_CONFIDENCE_6		0x4A
#define RES_DISTANCE_6_LSB		0x4B
#define RES_DISTANCE_6_MSB		0x4C
#define RES_CONFIDENCE_7		0x4D
#define RES_DISTANCE_7_LSB		0x4E
#define RES_DISTANCE_7_MSB		0x4F
#define RES_CONFIDENCE_8		0x50
#define RES_DISTANCE_8_LSB		0x51
#define RES_DISTANCE_8_MSB		0x52
#define RES_CONFIDENCE_9		0x53
#define RES_DISTANCE_9_LSB		0x54
#define RES_DISTANCE_9_MSB		0x55
#define RES_CONFIDENCE_10		0x56
#define RES_DISTANCE_10_LSB		0x57
#define RES_DISTANCE_10_MSB		0x58
#define RES_CONFIDENCE_11		0x59
#define RES_DISTANCE_11_LSB		0x5A
#define RES_DISTANCE_11_MSB		0x5B
#define RES_CONFIDENCE_12		0x5C
#define RES_DISTANCE_12_LSB		0x5D
#define RES_DISTANCE_12_MSB		0x5E
#define RES_CONFIDENCE_13		0x5F
#define RES_DISTANCE_13_LSB		0x60
#define RES_DISTANCE_13_MSB		0x61
#define RES_CONFIDENCE_14		0x62
#define RES_DISTANCE_14_LSB		0x63
#define RES_DISTANCE_14_MSB		0x64
#define RES_CONFIDENCE_15		0x65
#define RES_DISTANCE_15_LSB		0x66
#define RES_DISTANCE_15_MSB		0x67
#define RES_CONFIDENCE_16		0x68
#define RES_DISTANCE_16_LSB		0x69
#define RES_DISTANCE_16_MSB		0x6A
#define RES_CONFIDENCE_17		0x6B
#define RES_DISTANCE_17_LSB		0x6C
#define RES_DISTANCE_17_MSB		0x6D
#define RES_CONFIDENCE_18		0x6E
#define RES_DISTANCE_18_LSB		0x6F
#define RES_DISTANCE_18_MSB		0x70
#define RES_CONFIDENCE_19		0x71
#define RES_DISTANCE_19_LSB		0x72
#define RES_DISTANCE_19_MSB		0x73
#define RES_CONFIDENCE_20		0x74
#define RES_DISTANCE_20_LSB		0x75
#define RES_DISTANCE_20_MSB		0x76
#define RES_CONFIDENCE_21		0x77
#define RES_DISTANCE_21_LSB		0x78
#define RES_DISTANCE_21_MSB		0x79
#define RES_CONFIDENCE_22		0x7A
#define RES_DISTANCE_22_LSB		0x7B
#define RES_DISTANCE_22_MSB		0x7C
#define RES_CONFIDENCE_23		0x7D
#define RES_DISTANCE_23_LSB		0x7E
#define RES_DISTANCE_23_MSB		0x7F
#define RES_CONFIDENCE_24		0x80
#define RES_DISTANCE_24_LSB		0x81
#define RES_DISTANCE_24_MSB		0x82
#define RES_CONFIDENCE_25		0x83
#define RES_DISTANCE_25_LSB		0x84
#define RES_DISTANCE_25_MSB		0x85
#define RES_CONFIDENCE_26		0x86
#define RES_DISTANCE_26_LSB		0x87
#define RES_DISTANCE_26_MSB		0x88
#define RES_CONFIDENCE_27		0x89
#define RES_DISTANCE_27_LSB		0x8A
#define RES_DISTANCE_27_MSB		0x8B
#define RES_CONFIDENCE_28		0x8C
#define RES_DISTANCE_28_LSB		0x8D
#define RES_DISTANCE_28_MSB		0x8E
#define RES_CONFIDENCE_29		0x8F
#define RES_DISTANCE_29_LSB		0x90
#define RES_DISTANCE_29_MSB		0x91
#define RES_CONFIDENCE_30		0x92
#define RES_DISTANCE_30_LSB		0x93
#define RES_DISTANCE_30_MSB		0x94
#define RES_CONFIDENCE_31		0x95
#define RES_DISTANCE_31_LSB		0x96
#define RES_DISTANCE_31_MSB		0x97
#define RES_CONFIDENCE_32		0x98
#define RES_DISTANCE_32_LSB		0x99
#define RES_DISTANCE_32_MSB		0x9A
#define RES_CONFIDENCE_33		0x9B
#define RES_DISTANCE_33_LSB		0x9C
#define RES_DISTANCE_33_MSB		0x9D
#define RES_CONFIDENCE_34		0x9E
#define RES_DISTANCE_34_LSB		0x9F
#define RES_DISTANCE_34_MSB		0xA0
#define RES_CONFIDENCE_35		0xA1
#define RES_DISTANCE_35_LSB		0xA2
#define RES_DISTANCE_35_MSB		0xA3

//appid=0x03, cid_rid=0x16 – Configuration Page Registers
#define TOF_PERIOD_MS_LSB			0x24
#define TOF_PERIOD_MS_MSB			0x25
#define TOF_KILO_ITERATIONS_LSB		0x26
#define TOF_KILO_ITERATIONS_MSB		0x27
#define TOF_INT_THRESHOLD_LOW_LSB	0x28
#define TOF_INT_THRESHOLD_LOW_MSB	0x29
#define TOF_INT_THRESHOLD_HIGH_LSB	0x2A
#define TOF_INT_THRESHOLD_HIGH_MSB	0x2B
#define TOF_INT_ZONE_MASK_0			0x2C
#define TOF_INT_ZONE_MASK_1			0x2D
#define TOF_INT_ZONE_MASK_2			0x2E
#define TOF_INT_PERSISTENCE			0x2F
#define TOF_CONFIDENCE_THRESHOLD	0x30
#define TOF_GPIO_0					0x31
#define TOF_GPIO_1					0x32
#define TOF_POWER_CFG				0x33
#define TOF_SPAD_MAP_ID				0x34
#define TOF_ALG_SETTING_0			0x35
#define TOF_HIST_DUMP				0x39
#define TOF_I2C_SLAVE_ADDRESS		0x3B
#define TOF_OSC_TRIM_VALUE_LSB		0x3C
#define TOF_OSC_TRIM_VALUE_MSB		0x3D
#define TOF_I2C_ADDR_CHANGE			0x3E

//appid=0x03, cid_rid=0x17/0x18 – User Defined SPAD Configuration Registers
#define SPAD_ENABLE_FIRST		0x24
#define SPAD_ENABLE_LAST		0x41
#define SPAD_TDC_FIRST			0x42
#define SPAD_TDC_LAST			0x8C
#define SPAD_X_OFFSET_2			0x8D
#define SPAD_Y_OFFSET_2			0x8E
#define SPAD_X_SIZE				0x8F
#define SPAD_Y_SIZE				0x90

//appid=0x03, cid_rid=0x19 – Factory Calibration Registers
#define FACTORY_CALIBRATION_FIRST		0x24
#define CALIBRATION_STATUS_FC			0xDC
#define FACTORY_CALIBRATION_LAST		0xDF

//appid=0x03, cid_rid=0x81 – Raw Data Histograms Registers
#define SUBPACKET_NUMBER		0x24
#define SUBPACKET_PAYLOAD		0x25
#define SUBPACKET_CONFIG		0x26
#define SUBPACKET_DATA0			0x27
#define SUBPACKET_DATA127		0xA6

/*TOF return status */
#define TOF_OK 						0x00
#define TOF_ERROR			 		0x01

/*TOF sensor modes*/
#define TOF_MODE_TMF8828			0x08
#define TOF_MODE_TMF8821			0x00

/*TOF short range support*/
#define SHORT_RANGE_SUPPORTED		0x00
#define SHORT_RANGE_NOT_SUPPORTED	0x01

#define READ_COUNT					0x03

/*TOF device ready status*/
#define TOF_READY					0x00
#define TOF_NOT_READY				0x01

/*custom spad mask support*/
#define TOF_3x3 					0x00
#define TOF_3x6_4x4				    0x01

/*Bootloader status*/
#define STAT_OK       				0x00
#define STAT_ERR_SIZE 				0x01
#define	STAT_ERR_CSUM 				0x02

/*short/long range accuracy mode*/

#define	TOF_short_range	 			0x00
#define	TOF_long_range   			0x01

/*Pre-defined spad mask for 3x3 or 4x4 or 3x6 mode*/
#define SPAD_MAP_ID_1				0x01	//3x3 normal mode 45 degree fov
#define SPAD_MAP_ID_2				0x02	//3x3 macro mode 1 56 degree fov
#define SPAD_MAP_ID_3				0x03	//3x3 macro mode 2 56 degree fov
#define SPAD_MAP_ID_6				0x06	//3x3 wide mode 63 degree fov
#define SPAD_MAP_ID_11				0x0B	//3x3 checkerboard mode 45 degree fov
#define SPAD_MAP_ID_12				0x0C	//3x3 Inv. checkerboard mode 45 degree fov
#define SPAD_MAP_ID_7				0x07	//4x4 normal mode 63 degree fov
#define SPAD_MAP_ID_4				0x04	//4x4 macro mode 1 56 degree fov
#define SPAD_MAP_ID_5				0x05	//4x4 macro mode 2 56 degree fov
#define SPAD_MAP_ID_13				0x0D	//4x4 narrow mode 52 degree fov
#define SPAD_MAP_ID_10				0x0A	//3x6 mode 66 degree fov

/*Pre-defined spad mask for 8x8 mode*/
#define TOF_SPAD_8x8

#define CLK_CORR_WRAPAROUND      0x80000000L        // 32 bits

/*Measurement time*/
#define TOF_TIME_LSB				0xF4/*0x1E*/
#define TOF_TIME_MSB				0x01

/*helper functions*/
uint8_t tof_i2c_read(uint8_t *data, uint8_t regAddr, uint16_t len);
uint8_t tof_i2c_write(uint8_t *data, uint8_t regAddr, uint16_t len);
uint8_t tof_i2c_write_command(uint8_t *data,  uint16_t len);
uint8_t tof_calculate_checksum(uint8_t cmd, uint8_t size, uint8_t *data);
uint8_t bootloader_poll(void);
uint8_t bootloader_cmd_download_init(uint8_t *write_buffer);
uint8_t bootloader_cmd_addr_ram(uint8_t *write_buffer);
uint8_t bootloader_cmd_w_ram(uint8_t *write_buffer);
uint8_t bootloader_cmd_ramremap_reset(uint8_t *write_buffer);
uint8_t app_configuration(uint8_t spad_map, uint16_t time_ms, uint8_t time_msb);
uint8_t app_factory_callibration(void);
uint8_t app_measurement(void);
uint8_t app_full_config(void);
uint8_t custom_spad_mask(uint8_t spad_mask);
uint8_t switch_between_active_ranges(uint8_t range);
uint8_t app_cmd_measure_stop(void);
uint8_t enter_standby_mode(void);
uint8_t wake_up_device(void);
void power_cycle(void);
uint8_t is_device_ready(void);
uint8_t check_app_id(void);uint8_t exit_standby_timed_mode(void);
void bootloader_after_warm_start(void);
uint8_t bootloader_status(void);
uint8_t check_app_cmd_exec_cmpltd(void);
uint8_t check_config_page_loaded(uint8_t *write_reg);
uint8_t check_fw_support_for_short_range(void);
uint8_t check_active_range(void);
uint8_t switch_bw_tmf8828_and_tmf8821_mode(uint8_t mode);
uint8_t check_mode(void);
uint8_t tmf8821_store_factory_callibration(uint8_t *factory_buff);
uint8_t tmf8821_load_factory_callibration(uint8_t *factory_buff);
uint8_t startup(void);
uint8_t image_download(void);
uint8_t application_configuration(void);
uint8_t tof_patch_image_download(void);
uint8_t measurements_results(void);
uint8_t clock_correction(uint8_t *buff, uint32_t *host_diff, uint32_t *tof_diff);


#endif /* TOF_TOF_REG_H_ */
