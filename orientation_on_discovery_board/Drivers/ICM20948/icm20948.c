/*
* icm20948.c
*
*  Created on: Dec 26, 2020
*      Author: mokhwasomssi
*/


#include "icm20948.h"
#include"usart.h"
#include"usart.h"
#include <string.h>
#include <stdio.h>
static float gyro_scale_factor;
static float accel_scale_factor;

uint8_t ret;
int i=0;
int j=0;
int l=0;
int m=0;

char Sendout[50];
/* Static Functions */
static void     cs_high();
static void     cs_low();

static void     select_user_bank(userbank ub);

static uint8_t  read_single_icm20948_reg(userbank ub, uint8_t reg);
static void     write_single_icm20948_reg(userbank ub, uint8_t reg, uint8_t val);
 uint8_t* read_multiple_icm20948_reg(userbank ub, uint8_t reg, uint8_t len);
static void     write_multiple_icm20948_reg(userbank ub, uint8_t reg, uint8_t* val, uint8_t len);

static uint8_t  read_single_ak09916_reg(uint8_t reg);
static void     write_single_ak09916_reg(uint8_t reg, uint8_t val);
static uint8_t* read_multiple_ak09916_reg(uint8_t reg, uint8_t len);

/*Step counter define and static function*/

#define BDADDR_SIZE             6
#define FILTER_SAMPLE           5
#define PRECISION               100
#define SAMPLE_PERIOD           20
#define MAX_VALUE               4096
#define MIN_TIMEWIN				350
#define MAX_TIMEWIN				2000
#define STEPTHRESHOLD			4

int32_t x[FILTER_SAMPLE], y[FILTER_SAMPLE], z[FILTER_SAMPLE];
int32_t  x_max = - MAX_VALUE;
int32_t x_min = MAX_VALUE;
int32_t  y_max = - MAX_VALUE;
int32_t y_min = MAX_VALUE;
int32_t  z_max = - MAX_VALUE;
int32_t z_min = MAX_VALUE;
int32_t  x_th, y_th, z_th;
int32_t x_absmax,y_absmax,z_absmax;
int32_t  x_new, x_old, y_new, y_old, z_new, z_old;
int32_t  x_result, y_result, z_result;
int32_t d1, d2;
float temperature;
int counter;
int32_t steps;
int tempSteps = 0;
int formerSteps = -1;
int active;
uint32_t old_time, new_time;

typedef enum
{
  PedometerMode_STEADY = 0,
  PedometerMode_WALK = 1
}PedometerMode_TypeDef;

PedometerMode_TypeDef mode;
static int32_t int32abs(int32_t num)
{
	return num >= 0 ? num : -num;
}

void change_absmax(void)
{
	x_absmax = int32abs(x_max) > int32abs(x_min) ? int32abs(x_max) : int32abs(x_min);
	y_absmax = int32abs(y_max) > int32abs(y_min) ? int32abs(y_max) : int32abs(y_min);
	z_absmax = int32abs(z_max) > int32abs(z_min) ? int32abs(z_max) : int32abs(z_min);
}

static void newStep()
{
	old_time = new_time;
	new_time = HAL_GetTick();
	uint32_t time = new_time - old_time;

	if(time >= MIN_TIMEWIN && time <= MAX_TIMEWIN)
	{
			++steps;

	}

}

uint8_t intToChar(int32_t num)
{
	switch(num){
		case 0: return '0';
		case 1: return '1';
		case 2: return '2';
		case 3: return '3';
		case 4: return '4';
		case 5: return '5';
		case 6: return '6';
		case 7: return '7';
		case 8: return '8';
		case 9: return '9';
	}
	return '0';
}


/* Main Functions */
MTN_StatusTypeDef arg_mvp1_snsr_mtn_icm20948_init()
{
	icm20948_device_reset();

	while(!icm20948_who_am_i());

	icm20948_device_reset();
	icm20948_wakeup();

	icm20948_clock_source(1);
	icm20948_odr_align_enable();

	icm20948_spi_slave_enable();

	icm20948_gyro_low_pass_filter(0);
	icm20948_accel_low_pass_filter(0);

	icm20948_gyro_sample_rate_divider(0);
	icm20948_accel_sample_rate_divider(100);

	icm20948_gyro_calibration();
	icm20948_accel_calibration();

	icm20948_gyro_full_scale_select(_250dps);
	icm20948_accel_full_scale_select(_16g);

	return MTN_OK;
}

MTN_StatusTypeDef arg_mvp1_snsr_mtn_ak09916_init()
{
	icm20948_i2c_master_reset();
	icm20948_i2c_master_enable();
	icm20948_i2c_master_clk_frq(7);

	while(!ak09916_who_am_i());

	ak09916_soft_reset();
	ak09916_operation_mode_setting(power_down_mode);//continuous_measurement_100hz
	return MTN_OK;
}

void icm20948_gyro_read(axises* data)
{
	uint8_t* temp = read_multiple_icm20948_reg(ub_0, B0_GYRO_XOUT_H, 6);

	data->x = (int16_t)(temp[0] << 8 | temp[1]);
	data->y = (int16_t)(temp[2] << 8 | temp[3]);
	data->z = (int16_t)(temp[4] << 8 | temp[5]);
}

void icm20948_accel_read(axises* data){
	uint8_t* temp = read_multiple_icm20948_reg(ub_0, B0_ACCEL_XOUT_H, 6);

	data->x = (int16_t)(temp[0] << 8 | temp[1]);
	data->y = (int16_t)(temp[2] << 8 | temp[3]);
	data->z = (int16_t)(temp[4] << 8 | temp[5]) + accel_scale_factor;
	// Add scale factor because calibraiton function offset gravity acceleration.
}

MTN_StatusTypeDef arg_mvp1_snsr_mtn_icm20948_accel_read_step_counts(axises* data)
{

	/*SUM FILTERING */
	uint8_t* temp = read_multiple_icm20948_reg(ub_0, B0_ACCEL_XOUT_H, 6);

	data->x = (int16_t)(temp[0] << 8 | temp[1]);
	data->y = (int16_t)(temp[2] << 8 | temp[3]);
	data->z = (int16_t)(temp[4] << 8 | temp[5]) + accel_scale_factor; 
	// Add scale factor because calibraiton function offset gravity acceleration.
//	 sprintf(Sendout,"Accele:  x->%.4f, y->%.4f z->%.4f \r\n",data->x ,data->y ,data->z);
//	 HAL_UART_Transmit(&huart4,(uint8_t *)Sendout, strlen(Sendout),HAL_MAX_DELAY);

		x[0]=data->x;
		y[0]=data->y;
		z[0]=data->z;

		uint8_t* temp1 = read_multiple_icm20948_reg(ub_0, B0_ACCEL_XOUT_H, 6);

		data->x = (int16_t)(temp1[0] << 8 | temp1[1]);
		data->y = (int16_t)(temp1[2] << 8 | temp1[3]);
		data->z = (int16_t)(temp1[4] << 8 | temp1[5]) + accel_scale_factor;

		x[1]=data->x;
		y[1]=data->y;
		z[1]=data->z;

		uint8_t* temp2 = read_multiple_icm20948_reg(ub_0, B0_ACCEL_XOUT_H, 6);

		data->x = (int16_t)(temp2[0] << 8 | temp2[1]);
		data->y = (int16_t)(temp2[2] << 8 | temp2[3]);
		data->z = (int16_t)(temp2[4] << 8 | temp2[5]) + accel_scale_factor;

		x[2]=data->x;
		y[2]=data->y;
		z[2]=data->z;

		uint8_t* temp3 = read_multiple_icm20948_reg(ub_0, B0_ACCEL_XOUT_H, 6);

		data->x = (int16_t)(temp3[0] << 8 | temp3[1]);
		data->y = (int16_t)(temp3[2] << 8 | temp3[3]);
		data->z = (int16_t)(temp3[4] << 8 | temp3[5]) + accel_scale_factor;

		x[3]=data->x;
		y[3]=data->y;
		z[3]=data->z;

		x_result = (x[0] + x[1] + x[2] + x[3])/4;
		y_result = (y[0] + y[1] + y[2] + y[3])/4;
		z_result = (z[0] + z[1] + z[2] + z[3])/4;

		/*END SUM FILTERING*/

		/*FIND MAX AND MIN VALUE*/
		if (x_result > x_max) {
		    x_max = x_result;
		    change_absmax();
			}
		if (x_result < x_min) {
			x_min = x_result;
			change_absmax();
			}

		if (y_result > y_max) {
			y_max = y_result;
			change_absmax();
			}
		if (y_result < y_min) {
			y_min = y_result;
			change_absmax();
			}

		if (z_result > z_max) {
			z_max = z_result;
			change_absmax();
			}
		if (z_result < z_min) {
			z_min = z_result;
			change_absmax();
			}
		/*END FIND MAX AND MIN VALUE*/

			++counter;
			if(counter > 50){

				counter=0;
				x_th = (x_max + x_min) >>1;
				y_th = (y_max + y_min) >>1;
				z_th = (z_max + z_min) >>1;

							/*INIT VALUE*/
							/*
							x_max = -MAX_VALUE;
							x_min = MAX_VALUE;
							y_max = -MAX_VALUE;
							y_min = MAX_VALUE;
							z_max = -MAX_VALUE;
							z_min = MAX_VALUE;
							*/
							x_max = x_result;
							x_min = x_result;
							y_max = y_result;
							y_min = y_result;
							z_max = z_result;
							z_min = z_result;

			}
				x_old = x_new;
				y_old = y_new;
				z_old = z_new;

				if (x_new - x_result > PRECISION || x_result - x_new > PRECISION){
					x_new = x_result;
				}
				if (y_new - y_result > PRECISION || y_result - y_new > PRECISION) {
					y_new = y_result;
				}
				if (z_new - z_result > PRECISION || z_result - z_new > PRECISION){
					z_new = z_result;
				}


				/*FIND LARGEST ACCEL AXIS*/
				if(x_absmax > y_absmax && x_absmax > z_absmax)
				{
					if(x_old > x_th && x_new < x_th)
					{
						newStep();
					}
				} else if (y_absmax > x_absmax && y_absmax > z_absmax)
				{
					if(y_old > y_th && y_new < y_th)
					{
						newStep();

					}
				} else
				{
					if(z_old > z_th && z_new < z_th)
					{
						newStep();
					}
				}

				if(steps != formerSteps)
				{
					unsigned char dataOut[20] ;//= {'s','t','e','p',':',intToChar(steps/1000%10), intToChar(steps/100%10),intToChar(steps/10%10),intToChar(steps%10)};
					sprintf(dataOut, "steps: %d\r\n", (int)steps);
					HAL_UART_Transmit(&huart1,(uint8_t *)dataOut, strlen(dataOut),HAL_MAX_DELAY);
					formerSteps = steps;
				}

	return MTN_OK;
}



bool ak09916_mag_read(axises* data)
{
	uint8_t* temp;
	uint8_t drdy, hofl;	// data ready, overflow

	drdy = read_single_ak09916_reg(MAG_ST1) & 0x01;
	if(!drdy)	return false;

	temp = read_multiple_ak09916_reg(MAG_HXL, 6);

	hofl = read_single_ak09916_reg(MAG_ST2) & 0x08;
	if(hofl)	return false;

	data->x = (int16_t)(temp[1] << 8 | temp[0]);
	data->y = (int16_t)(temp[3] << 8 | temp[2]);
	data->z = (int16_t)(temp[5] << 8 | temp[4]);

	return true;
}

void icm20948_gyro_read_dps(axises* data)
{
	icm20948_gyro_read(data);

	data->x /= gyro_scale_factor;
	data->y /= gyro_scale_factor;
	data->z /= gyro_scale_factor;
}

void icm20948_accel_read_g(axises* data)
{


	icm20948_accel_read(data);

	data->x /= accel_scale_factor;
	data->y /= accel_scale_factor;
	data->z /= accel_scale_factor;
	 sprintf(Sendout,"Accele:  x->%.4f\t y->%.4f\t z->%.4f \r\n",data->x ,data->y ,data->z);
	 HAL_UART_Transmit(&huart1,(uint8_t *)Sendout, strlen(Sendout),HAL_MAX_DELAY);

//	 if(data->x >= 0.3000 ){
//		 i++;
//		 printmsg("FORWORD STEP COUNT - %d\r\n",i);
//	 }
//	 else if(data->x <= -0.2773 ){
//		 j++;
//		 printmsg("BACKWORD STEP COUNT - %d\r\n",j);
//	 }
//	 else if(data->y >= 0.3000){
//		 l++;
//		 printmsg("RIGHT STEP COUNT - %d\r\n",l);
//	 }
//	 else if(data->y <= -0.2773){
//		 m++;
//		 printmsg("Left STEP COUNT - %d\r\n",m);
//	 }

}

bool ak09916_mag_read_uT(axises* data)
{
	axises temp;
	bool new_data = ak09916_mag_read(&temp);
	if(!new_data)	return false;

	data->x = (float)(temp.x * 0.15);
	data->y = (float)(temp.y * 0.15);
	data->z = (float)(temp.z * 0.15);

	return true;
}	


/* Sub Functions */
bool icm20948_who_am_i()
{
	uint8_t icm20948_id = read_single_icm20948_reg(ub_0, B0_WHO_AM_I);

	if(icm20948_id == ICM20948_ID)
		return true;
	else
		return false;
}

bool ak09916_who_am_i()
{
	uint8_t ak09916_id = read_single_ak09916_reg(MAG_WIA2);

	if(ak09916_id == AK09916_ID)
		return true;
	else
		return false;
}

void icm20948_device_reset()
{
	write_single_icm20948_reg(ub_0, B0_PWR_MGMT_1, 0x80 | 0x41);
	HAL_Delay(100);
}

void ak09916_soft_reset()
{
	write_single_ak09916_reg(MAG_CNTL3, 0x01);
	HAL_Delay(100);
}

void icm20948_wakeup()
{
	uint8_t new_val = read_single_icm20948_reg(ub_0, B0_PWR_MGMT_1);
	new_val &= 0xBF;

	write_single_icm20948_reg(ub_0, B0_PWR_MGMT_1, new_val);
	HAL_Delay(100);
}

void icm20948_sleep()
{
	uint8_t new_val = read_single_icm20948_reg(ub_0, B0_PWR_MGMT_1);
	new_val |= 0x40;

	write_single_icm20948_reg(ub_0, B0_PWR_MGMT_1, new_val);
	HAL_Delay(100);
}

void icm20948_spi_slave_enable()
{
	uint8_t new_val = read_single_icm20948_reg(ub_0, B0_USER_CTRL);
	new_val |= 0x10;

	write_single_icm20948_reg(ub_0, B0_USER_CTRL, new_val);
}

void icm20948_i2c_master_reset()
{
	uint8_t new_val = read_single_icm20948_reg(ub_0, B0_USER_CTRL);
	new_val |= 0x02;

	write_single_icm20948_reg(ub_0, B0_USER_CTRL, new_val);
}

void icm20948_i2c_master_enable()
{
	uint8_t new_val = read_single_icm20948_reg(ub_0, B0_USER_CTRL);
	new_val |= 0x20;

	write_single_icm20948_reg(ub_0, B0_USER_CTRL, new_val);
	HAL_Delay(100);
}

void icm20948_i2c_master_clk_frq(uint8_t config)
{
	uint8_t new_val = read_single_icm20948_reg(ub_3, B3_I2C_MST_CTRL);
	new_val |= config;

	write_single_icm20948_reg(ub_3, B3_I2C_MST_CTRL, new_val);	
}

void icm20948_clock_source(uint8_t source)
{
	uint8_t new_val = read_single_icm20948_reg(ub_0, B0_PWR_MGMT_1);
	new_val |= source;

	write_single_icm20948_reg(ub_0, B0_PWR_MGMT_1, new_val);
}

void icm20948_sleep_mode()
{
	uint8_t new_val = read_single_icm20948_reg(ub_0, B0_PWR_MGMT_1);
	new_val |= 0x40;

	write_single_icm20948_reg(ub_0, B0_PWR_MGMT_1, new_val);
}

void icm20948_lp_mode()
{
	uint8_t new_val = read_single_icm20948_reg(ub_0, B0_PWR_MGMT_1);
	new_val |= 0x20;

	write_single_icm20948_reg(ub_0, B0_PWR_MGMT_1, new_val);
}

void icm20948_odr_align_enable()
{
	write_single_icm20948_reg(ub_2, B2_ODR_ALIGN_EN, 0x01);
}

void icm20948_gyro_low_pass_filter(uint8_t config)
{
	uint8_t new_val = read_single_icm20948_reg(ub_2, B2_GYRO_CONFIG_1);
	new_val |= config << 3;

	write_single_icm20948_reg(ub_2, B2_GYRO_CONFIG_1, new_val);
}

void icm20948_accel_low_pass_filter(uint8_t config)
{
	uint8_t new_val = read_single_icm20948_reg(ub_2, B2_ACCEL_CONFIG);
	new_val |= config << 3;

	write_single_icm20948_reg(ub_2, B2_GYRO_CONFIG_1, new_val);
}

void icm20948_gyro_sample_rate_divider(uint8_t divider)
{
	write_single_icm20948_reg(ub_2, B2_GYRO_SMPLRT_DIV, divider);
}

void icm20948_accel_sample_rate_divider(uint16_t divider)
{
	uint8_t divider_1 = (uint8_t)(divider >> 8);
	uint8_t divider_2 = (uint8_t)(0x0F & divider);

	write_single_icm20948_reg(ub_2, B2_ACCEL_SMPLRT_DIV_1, divider_1);
	write_single_icm20948_reg(ub_2, B2_ACCEL_SMPLRT_DIV_2, divider_2);
}

void ak09916_operation_mode_setting(operation_mode mode)
{
	write_single_ak09916_reg(MAG_CNTL2, mode);
	HAL_Delay(100);
}

void icm20948_gyro_calibration()
{
	axises temp;
	int32_t gyro_bias[3] = {0};
	uint8_t gyro_offset[6] = {0};

	for(int i = 0; i < 100; i++)
	{
		icm20948_gyro_read(&temp);
		gyro_bias[0] += temp.x;
		gyro_bias[1] += temp.y;
		gyro_bias[2] += temp.z;
	}

	gyro_bias[0] /= 100;
	gyro_bias[1] /= 100;
	gyro_bias[2] /= 100;

	// Construct the gyro biases for push to the hardware gyro bias registers,
	// which are reset to zero upon device startup.
	// Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format.
	// Biases are additive, so change sign on calculated average gyro biases
	gyro_offset[0] = (-gyro_bias[0] / 4  >> 8) & 0xFF; 
	gyro_offset[1] = (-gyro_bias[0] / 4)       & 0xFF; 
	gyro_offset[2] = (-gyro_bias[1] / 4  >> 8) & 0xFF;
	gyro_offset[3] = (-gyro_bias[1] / 4)       & 0xFF;
	gyro_offset[4] = (-gyro_bias[2] / 4  >> 8) & 0xFF;
	gyro_offset[5] = (-gyro_bias[2] / 4)       & 0xFF;
	
	write_multiple_icm20948_reg(ub_2, B2_XG_OFFS_USRH, gyro_offset, 6);
}

void icm20948_accel_calibration()
{
	axises temp;
	uint8_t* temp2;
	uint8_t* temp3;
	uint8_t* temp4;
	
	int32_t accel_bias[3] = {0};
	int32_t accel_bias_reg[3] = {0};
	uint8_t accel_offset[6] = {0};

	for(int i = 0; i < 100; i++)
	{
		icm20948_accel_read(&temp);
		accel_bias[0] += temp.x;
		accel_bias[1] += temp.y;
		accel_bias[2] += temp.z;
	}

	accel_bias[0] /= 100;
	accel_bias[1] /= 100;
	accel_bias[2] /= 100;

	uint8_t mask_bit[3] = {0, 0, 0};

	temp2 = read_multiple_icm20948_reg(ub_1, B1_XA_OFFS_H, 2);
	accel_bias_reg[0] = (int32_t)(temp2[0] << 8 | temp2[1]);
	mask_bit[0] = temp2[1] & 0x01;

	temp3 = read_multiple_icm20948_reg(ub_1, B1_YA_OFFS_H, 2);
	accel_bias_reg[1] = (int32_t)(temp3[0] << 8 | temp3[1]);
	mask_bit[1] = temp3[1] & 0x01;

	temp4 = read_multiple_icm20948_reg(ub_1, B1_ZA_OFFS_H, 2);
	accel_bias_reg[2] = (int32_t)(temp4[0] << 8 | temp4[1]);
	mask_bit[2] = temp4[1] & 0x01;

	accel_bias_reg[0] -= (accel_bias[0] / 8);
	accel_bias_reg[1] -= (accel_bias[1] / 8);
	accel_bias_reg[2] -= (accel_bias[2] / 8);

	accel_offset[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  	accel_offset[1] = (accel_bias_reg[0])      & 0xFE;
	accel_offset[1] = accel_offset[1] | mask_bit[0];

	accel_offset[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  	accel_offset[3] = (accel_bias_reg[1])      & 0xFE;
	accel_offset[3] = accel_offset[3] | mask_bit[1];

	accel_offset[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	accel_offset[5] = (accel_bias_reg[2])      & 0xFE;
	accel_offset[5] = accel_offset[5] | mask_bit[2];
	
	write_multiple_icm20948_reg(ub_1, B1_XA_OFFS_H, &accel_offset[0], 2);
	write_multiple_icm20948_reg(ub_1, B1_YA_OFFS_H, &accel_offset[2], 2);
	write_multiple_icm20948_reg(ub_1, B1_ZA_OFFS_H, &accel_offset[4], 2);
}

void icm20948_gyro_full_scale_select(gyro_full_scale full_scale)
{
	uint8_t new_val = read_single_icm20948_reg(ub_2, B2_GYRO_CONFIG_1);
	
	switch(full_scale)
	{
		case _250dps :
			new_val |= 0x00;
			gyro_scale_factor = 131.0;
			break;
		case _500dps :
			new_val |= 0x02;
			gyro_scale_factor = 65.5;
			break;
		case _1000dps :
			new_val |= 0x04;
			gyro_scale_factor = 32.8;
			break;
		case _2000dps :
			new_val |= 0x06;
			gyro_scale_factor = 16.4;
			break;
	}

	write_single_icm20948_reg(ub_2, B2_GYRO_CONFIG_1, new_val);
}

void icm20948_accel_full_scale_select(accel_full_scale full_scale)
{
	uint8_t new_val = read_single_icm20948_reg(ub_2, B2_ACCEL_CONFIG);
	
	switch(full_scale)
	{
		case _2g :
			new_val |= 0x00;
			accel_scale_factor = 16384;
			break;
		case _4g :
			new_val |= 0x02;
			accel_scale_factor = 8192;
			break;
		case _8g :
			new_val |= 0x04;
			accel_scale_factor = 4096;
			break;
		case _16g :
			new_val |= 0x06;
			accel_scale_factor = 2048;
			break;
	}

	write_single_icm20948_reg(ub_2, B2_ACCEL_CONFIG, new_val);
}


/* Static Functions */
static void cs_high()
{
	//HAL_GPIO_WritePin(ICM20948_SPI_CS_PIN_PORT, ICM20948_SPI_CS_PIN_NUMBER, SET);
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);
}

static void cs_low()
{
	//HAL_GPIO_WritePin(ICM20948_SPI_CS_PIN_PORT, ICM20948_SPI_CS_PIN_NUMBER, RESET);
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);
}

static void select_user_bank(userbank ub)
{
	uint8_t write_reg[2];
	write_reg[0] = WRITE | REG_BANK_SEL;
	write_reg[1] = ub;

	cs_low();
	ret=HAL_SPI_Transmit(ICM20948_SPI, write_reg, 2, 10);
	cs_high();
}

static uint8_t read_single_icm20948_reg(userbank ub, uint8_t reg)
{
	uint8_t read_reg = READ | reg;
	uint8_t reg_val;
	select_user_bank(ub);

	cs_low();
	ret= HAL_SPI_Transmit(ICM20948_SPI, &read_reg, 1, 1000);
	ret= HAL_SPI_Receive(ICM20948_SPI, &reg_val, 1, 1000);
	cs_high();

	return reg_val;
}

static void write_single_icm20948_reg(userbank ub, uint8_t reg, uint8_t val)
{
	uint8_t write_reg[2];
	write_reg[0] = WRITE | reg;
	write_reg[1] = val;

	select_user_bank(ub);

	cs_low();
	ret=HAL_SPI_Transmit(ICM20948_SPI, write_reg, 2, 1000);
	cs_high();
}

 uint8_t* read_multiple_icm20948_reg(userbank ub, uint8_t reg, uint8_t len)
{
	uint8_t read_reg = READ | reg;
	static uint8_t reg_val[6];
	select_user_bank(ub);

	cs_low();
	ret=HAL_SPI_Transmit(ICM20948_SPI, &read_reg, 1, 1000);
	ret=HAL_SPI_Receive(ICM20948_SPI, reg_val, len, 1000);
	cs_high();

	return reg_val;
}

static void write_multiple_icm20948_reg(userbank ub, uint8_t reg, uint8_t* val, uint8_t len)
{
	uint8_t write_reg = WRITE | reg;
	select_user_bank(ub);

	cs_low();
	ret=HAL_SPI_Transmit(ICM20948_SPI, &write_reg, 1, 1000);
	ret=HAL_SPI_Transmit(ICM20948_SPI, val, len, 1000);
	cs_high();
}

static uint8_t read_single_ak09916_reg(uint8_t reg)
{
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_ADDR, READ | MAG_SLAVE_ADDR);
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_REG, reg);
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_CTRL, 0x81);

	HAL_Delay(1);
	return read_single_icm20948_reg(ub_0, B0_EXT_SLV_SENS_DATA_00);
}

static void write_single_ak09916_reg(uint8_t reg, uint8_t val)
{
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_ADDR, WRITE | MAG_SLAVE_ADDR);
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_REG, reg);
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_DO, val);
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_CTRL, 0x81);
}

static uint8_t* read_multiple_ak09916_reg(uint8_t reg, uint8_t len)
{	
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_ADDR, READ | MAG_SLAVE_ADDR);
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_REG, reg);
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_CTRL, 0x80 | len);

	HAL_Delay(1);
	return read_multiple_icm20948_reg(ub_0, B0_EXT_SLV_SENS_DATA_00, len);
}

extern uint8_t aShowTime[20];
extern uint8_t aShowDate[20];
char *data="12:00:00\r\n";
void arg_mvp1_snsr_mtn_icm20948_timestamp_steps(void){

//	arg_mvp1_snsr_rtc_calendar_show();
	if(aShowDate){

		if(aShowTime == (uint8_t*)data){
			steps=0;
		}
	}
}

#define BKUPWrite_Value 0x32F2
void arg_mvp1_snsr_mtn_icm20948_stpes_write_flash(void){

	//QSPI_flash_write_function(uint8_t steps);


	    	 //QSPI_flash_read_function(uint8_t steps);

}

uint32_t arg_mvp1_snsr_mtn_icm20948_stpes_read_flash(void){

	//QSPI_flash_read_function(uint8_t steps);

	     return steps;
}


