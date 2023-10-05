/******************************************************************************
  * file name:    motoion_sensor.h
  * author name:  Sukhdeep Singh Soni (sukhdeepsingh@focally.in)
  * module name:  Motion Sensor(LSM6DSOX & LIS2MDL)
  * file version: 1.1
  * about:        This file provides bsp api's headers specific to motion sensor:
  *           	  + SPI read register
  *           	  + SPI write register
  *           	  + Local delay
  *           	  + Return status
  ******************************************************************************
**/

#ifndef BSP_MOTION_SENSOR_H_
#define BSP_MOTION_SENSOR_H_

#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "lsm6dsox.h"
#include "lsm6dsox_reg.h"
#include "lis2mdl.h"
#include "lis2mdl_reg.h"
#include<stdio.h>

#define I2C_SPI_EN		1 //enable/disable i2c
#define ACC_EN 			1 //enable/disable accelerometer
#define GYRO_EN 		1 //enable/disable gyrometer
#define MAG_EN			1 //enable/disable magnetometer
#define LATCH_PULSE_EN	1 //enable/disable magnetometer interrupt latching
#define INT1			1 //enable/disable interrupt2
#define INT2			1 //enable/disable interrupt1
#define INT_FLAG		1 //enable disable interrupt flag functioning

#define mtn_ctrl1_xl		0x10 //accelerometer control register
#define mtn_ctrl6_c			0x15 //control register 6
#define mtn_ctrl5_c			0x14 //control register 5
#define mtn_ctrl7_g			0x16 //control register 7
#define mtn_ctrl2_g			0x11 //gyroscope control register
#define mtn_ctrl1_xl		0x10 //accelerometer control register
#define mtn_ctrl4_c			0x13 //control register 4
#define ALL_INT_SRC			0x1A // interrupt source register
#define WAKE_UP_SRC			0x1B // wake up interrupt source register
#define mag_rst_reg			0x60 //magnetometer reset register address
#define mag_rst_val 		0x23 //magnetometer reset value
#define acc_gyro_rst_reg	0x12 //ac_gyro reset register address
#define cfg_reg_c			0x50 //lis2mdl cfg_reg_c reg addr
#define cfg_reg_a 			0x60 //lis2mdl cfg_reg_1 reg addr


#if (LATCH_PULSE_EN == 1)
#define int_ctrl_reg		0xE7 //lis2mdl int. control register
#define int_ctrl_reg_near	0xE7 //give int. and latching when magnet is near
#define int_ctrl_reg_far	0xE3 //give int. when magnet is away and latching when magnet is near(not giving int)
#elif (LATCH_PULSE_EN == 0)
#define int_ctrl_reg		0xE5 //lis2mdl int. control register
#define int_ctrl_reg_near	0xE5 //give int. when magnet is near
#define int_ctrl_reg_far	0xE1 //give int. when magnet is away
#endif

#define int_ths_l_reg 		0xdc //lis2mdl threshold lower byte
#define int_ths_h_reg 		0x05 //lis2mdl threshold higher byte
#define wk_ths 				0x06 //wake up thershold

#define EMB_FUNC_STATUS_MAINPAGE	0x35 //embedded functions status register

typedef enum {
	HIGH_PERFORMANCE_MODE,
	LOW_POWER_MODE,
	ULTRA_LOW_POWER_MODE,
	NORMAL_MODE,
	POWER_DOWN_MODE
} motion_sensor_pow_modes;



void custom_delay(void);
uint8_t mtn_acc_init(LSM6DSOX_Object_t *pObj);
uint8_t mtn_gyro_init(LSM6DSOX_Object_t *pObj);
uint8_t reg_ret_status(uint8_t regAddr, uint8_t *buff);
void spi_i2c_acc_gyro_setup(void);
void spi_i2c_mag_setup(void);

#if (I2C_SPI_EN == 0)
int32_t spi_read_reg_mtn_snsr(void *handle, uint8_t Reg, uint8_t *pdata, uint16_t Length);
int32_t spi_write_reg_mtn_snsr(void *handle, uint8_t Reg, uint8_t *pdata, uint16_t Length);
#elif (I2C_SPI_EN == 1)
int32_t i2c_write_8bit(void *handle, uint8_t Reg, uint8_t *pdata, uint16_t Length);
int32_t i2c_read_8bit(void *handle, uint8_t Reg, uint8_t *pdata, uint16_t Length);
#endif

int32_t mtn_snsr_delay(void);
void mtn_acc_gyro_init(void);
void mtn_mag_init(void);
void mtn_bsp_init(void);
void mtn_acc_gyro_process(void);
void mtn_mag_process(void);
void mtn_bsp_process(void);
void mtn_acc_mode_set(motion_sensor_pow_modes mode);
void mtn_gyro_mode_set(motion_sensor_pow_modes mode);
void mtn_mag_mode_set(motion_sensor_pow_modes mode);
void check_mtn_acc_mode(void);
void check_mtn_gyro_mode(void);
void check_mag_mode(void);
void acc_test(void);
void mtn_mag_int_gen(void);
int32_t mtn_read_sensorhub_reg(LIS2MDL_Object_t *pObj, uint8_t Reg, uint8_t *pData, uint16_t Length);
int32_t mtn_write_sensorhub_reg(LIS2MDL_Object_t *pObj, uint8_t Reg, uint8_t *pData, uint16_t Length);

//end of file


#endif /* BSP_MOTION_SENSOR_H_ */
