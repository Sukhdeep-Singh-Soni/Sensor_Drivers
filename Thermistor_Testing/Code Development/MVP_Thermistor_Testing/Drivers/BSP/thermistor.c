/******************************************************************************
  * file name:    thermistor.c
  * author name:  Sukhdeep Singh Soni (sukhdeepsingh@focally.in)
  * module name:  MVP Thermistor Testing
  * file version: 1.0
  * about:        This file provides the api's related to fuel gauge ext. thermistor:
  *  			  + i2c 16-bit read/write
  *           	  + read chip id
  ******************************************************************************
**/

#include "thermistor.h"

#define THRESHOLD_MIN	0X19 // eg. 0x19->25 degreeC
#define THRESHOLD_MAX	0x1b // eg. 0x1b-> degreeC
#define RW_8_BIT		0	//  enable/disable i2c 8 bit read/write
#define RW_16_BIT		1	//  enable//disable i2c 16 bit read/write
#define THM_EN			1	//  enable/disable thermistor

uint8_t slave_address = 0x36<<1;
uint16_t ret_val16=0,Temp_val=0, status0=0, status1=0;
uint8_t buff[2] = {0}, tmp[2] = {0};
uint8_t rst[2] = {0};

uint8_t i2c_scanner(void)
{
	uint8_t dev_detected, addr, err_code, buffer=0;
	for(addr=0; addr<255; addr++)
	{
		err_code = HAL_I2C_Master_Receive(&hi2c1, addr, &buffer, sizeof(buffer), HAL_MAX_DELAY);
		if(err_code == HAL_OK)
		{
			dev_detected = 1;
		}
	}
	if(!dev_detected)
		return HAL_ERROR;
	else
		return HAL_OK;
}

uint16_t chip_id(uint8_t reg_address)
{
	uint8_t cid[2] = {0};
	uint16_t val16;

	val16 = i2c_read_16bit(cid, reg_address, 2);
	if(val16 == 0x4031)
		return val16;
	else
		return HAL_ERROR;
}

#if RW_8_BIT
uint8_t i2c_read_8bit(uint8_t *buff, uint8_t reg_address, uint8_t bytes)
{
	uint8_t err_code;
	err_code = HAL_I2C_Master_Transmit(&hi2c1, slave_address, &reg_address, 1, HAL_MAX_DELAY);
	if(err_code != HAL_OK)
		return HAL_ERROR;
	err_code = HAL_I2C_Master_Receive(&hi2c1, slave_address, buff, bytes, HAL_MAX_DELAY);
	if(err_code != HAL_OK)
		return HAL_ERROR;
	return HAL_OK;
}

uint8_t i2c_write_8bit(uint8_t *buff, uint8_t reg_address, uint8_t bytes, uint8_t data)
{
	uint8_t err_code;
	buff[0] = reg_address;
	buff[1] = data;
	err_code = HAL_I2C_Master_Transmit(&hi2c1, slave_address, buff, bytes+1, HAL_MAX_DELAY);
	if(err_code != HAL_OK)
		return HAL_ERROR;
	return HAL_OK;
}
#endif

#if RW_16_BIT
uint16_t i2c_read_16bit(uint8_t *buff, uint8_t reg_address, uint8_t bytes)
{
		uint16_t  lsb, msb, data16;
		uint8_t err_code;
		err_code = HAL_I2C_Master_Transmit(&hi2c1, slave_address, &reg_address, 1, HAL_MAX_DELAY);
		if(err_code != HAL_OK)
			return HAL_ERROR;
		err_code = HAL_I2C_Master_Receive(&hi2c1, slave_address, buff, bytes, HAL_MAX_DELAY);
		if(err_code != HAL_OK)
		{
			return HAL_ERROR;
		}
		lsb = buff[0];
		msb = buff[1];
		msb = msb << 8;
		data16 = msb | lsb;
		return data16;
}
uint16_t i2c_write_16bit(uint8_t *buff, uint8_t reg_address, uint8_t bytes, uint8_t *data)
{
	uint8_t err_code;
	buff[0] = reg_address;
	buff[1] = data[0];
	buff[2] = data[1];
	err_code = HAL_I2C_Master_Transmit(&hi2c1, slave_address, buff, bytes+1, HAL_MAX_DELAY);
	if(err_code != HAL_OK)
		return HAL_ERROR;
	return HAL_OK;
}
#endif

#if THM_EN
void mvp_thermistor_init(void)
{
	   uint16_t cid_val=0;
	   uint8_t shdn_val[2];
	   shdn_val[0] = 0x80;
	   shdn_val[1] = 0x00;

	   i2c_scanner(); //slave address fuel guage = 0x6c,0x6d and pmic = 0x90,0x91
	   cid_val = chip_id(devname_reg); //DevName Register = 0x21, value = 0x4031.
	   if(cid_val != 0x4031)
	   {
		   i2c_write_16bit(buff, 0x1d, 2, shdn_val);
		   HAL_Delay(45000);
		   cid_val = chip_id(devname_reg);
	   }

	   if(cid_val == 0x4031)
	   {
	   //clearing POR bit
	   i2c_write_16bit(buff, status_reg, 2, rst);
	   status0 = i2c_read_16bit(buff, status_reg, 2);

	   //setting threshold for max and min temperature
	   i2c_read_16bit(buff, talrtth_reg, 2);
	   tmp[0] = THRESHOLD_MIN; //lower threshold     eg. 0x19->25 degreeC
	   tmp[1] = THRESHOLD_MAX; // higher threshold   eg. 0x1b-> degreeC
	   i2c_write_16bit(buff, talrtth_reg, 2, tmp);
	   i2c_read_16bit(buff, talrtth_reg, 2);

	   //read config2 reg
	   i2c_read_16bit(buff, config2_reg, 2);

	   //config TES=1, ETHRM=1, FTHRM=1 and Aen=1
	   ret_val16=i2c_read_16bit(buff, config_reg, 2);
	   tmp[0] = 0x14; //enabling ETHRM bit = 1 and Aen = 1
	   tmp[1] = 0xa2; //enabling Tes bit = 1
	   i2c_write_16bit(buff, config_reg, 2, tmp);
	   ret_val16=i2c_read_16bit(buff, config_reg, 2);
	   }
}

void mvp_thermistor_process(void)
{
	  ret_val16 = i2c_read_16bit(buff, temp_reg, 2); //temp register
	  if(ret_val16 & 0x8000)
	  {
		  ret_val16 = (~ret_val16) + 1;
	  }
	  ret_val16  = ret_val16 >> 8; // 1 degree Celsius is 1 lsb in upper byte.
	  Temp_val = ret_val16; //temperature value in degree celsius.
	  i2c_write_16bit(buff, status_reg, 2, rst);
	  status1 = i2c_read_16bit(buff, status_reg, 2);
}
#endif

//end of file
