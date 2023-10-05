/******************************************************************************
  * file name:    magnetometer.h
  * author name:  Sukhdeep Singh Soni (sukhdeepsingh@focally.in)
  * module name:  Motion Sensor(LSM6DSOX & LIS2MDL)
  * file version: 1.1
  * about:        This file provides bsp headers specific to magnetometer(lis2mdl):
  ******************************************************************************
**/

#ifndef BSP_MAGNETOMETER_H_
#define BSP_MAGNETOMETER_H_

#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "lis2mdl.h"
#include "lis2mdl_reg.h"
#include<stdio.h>

#define INT_FLAG		1 //enable disable interrupt flag functioning
#define LATCH_PULSE_EN	0 //enable/disable magnetometer interrupt latching/pulsing
#define MAG_EN			1 //enable/disable magnetometer

#define mag_rst_val 		0x23 //magnetometer reset value
#define cfg_reg_c			0x50 //lis2mdl cfg_reg_c reg addr
#define cfg_reg_a 			0x60 //lis2mdl cfg_reg_1 reg addr
#define mtn_ctrl4_c			0x13 //control register 4

#if (LATCH_PULSE_EN == 1)
#define int_ctrl_reg		0xe7 //lis2mdl int. control register
#define int_ctrl_reg_near	0xe7 //give int. and latching when magnet is near
#define int_ctrl_reg_far	0xe3 //give int. when magnet is away and latching when magnet is near(not giving int)
#elif (LATCH_PULSE_EN == 0)
#define int_ctrl_reg		0xe5 //lis2mdl int. control register
#define int_ctrl_reg_near	0xe5 //give int. when magnet is near
#define int_ctrl_reg_far	0xe1 //give int. when magnet is away
#endif

#define int_ths_l_reg 		0xe8 //lis2mdl threshold lower byte
#define int_ths_h_reg 		0x03 //lis2mdl threshold higher byte

typedef enum {
	LOW_POWER_MODE			= 1,
	HIGH_PERFORMANCE_MODE	= 0
} mag_power_modes;

uint8_t i2c_scanner(void);
void i2c_mag_setup(void);
void mag_init(void);
void bsp_init(void);
void mag_process(void);
void mag_int_gen(void);
void bsp_process(void);
int32_t i2c_read_8bit(void *handle, uint8_t Reg, uint8_t *pdata, uint16_t Length);
int32_t i2c_write_8bit(void *handle, uint8_t Reg, uint8_t *pdata, uint16_t Length);
#endif /* BSP_MAGNETOMETER_H_ */

//end of file

