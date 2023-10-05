/*
 * tof_reg.c
 *
 *  Created on: 06-Dec-2022
 *      Author: sukhdeep
 */
#ifndef TOF_REG_C
#define TOF_REG_C

#include "tof_reg.h"
#include "tof_image.h"

/*Global Variables*/
uint8_t read_val = 0, write_val = 0;
uint32_t counter = 0;
uint8_t result_buff[132] = {0};

uint32_t old_host_time = 0, old_tof_time = 0;
uint32_t new_host_time = 0, new_tof_time = 0;


/*@brief - i2c read function
  @ret   - return i2c data or TOF_ERROR otherwise*/
uint8_t tof_i2c_read(uint8_t *data, uint8_t regAddr, uint16_t len)
	{
		if(HAL_I2C_Master_Transmit(TOF_I2C_Handle, TOF_I2C_ADDR, &regAddr, 1, HAL_MAX_DELAY) != HAL_OK)
			return TOF_ERROR;

		if(HAL_I2C_Master_Receive(TOF_I2C_Handle, TOF_I2C_ADDR, data, len, HAL_MAX_DELAY) != HAL_OK)
			return TOF_ERROR;

		return *data;
	}

/*@breif - i2c write function
  @ret   - TOF_OK if success or TOF_ERROR otherwise*/
uint8_t tof_i2c_write(uint8_t *data, uint8_t regAddr, uint16_t len)
{
	uint8_t send_data[2];
	send_data[0] = regAddr;
	send_data[1] = *data;
	if(HAL_I2C_Master_Transmit(TOF_I2C_Handle, TOF_I2C_ADDR, send_data, len, HAL_MAX_DELAY) != HAL_OK)
		return TOF_ERROR;

	return TOF_OK;
}

/*@breif - i2c write command function
  @ret   - TOF_OK if success or TOF_ERROR otherwise*/
uint8_t tof_i2c_write_command(uint8_t *data,  uint16_t len)
	{

		if(HAL_I2C_Master_Transmit(TOF_I2C_Handle, TOF_I2C_ADDR, data, len, HAL_MAX_DELAY) != HAL_OK)
			return TOF_ERROR;

		return TOF_OK;
	}

/*@brief - bootloader checksum calculation function
  @ret   - return checksum value*/
uint8_t tof_calculate_checksum(uint8_t cmd, uint8_t size, uint8_t *data)
{
	uint8_t sum = 0;
	uint8_t private_size = size;
	for(; size > 0; size--)
	{
		sum += *data;
		data++;
	}

	sum += cmd + private_size;
	return ~sum;
}

/*@brief - poll bootloader BL_CMD_STAT register to get 0x00 value
  @ret   - TOF_OK if success or TOF_ERROR otherwise*/
uint8_t bootloader_poll(void)
{
	uint8_t read_back[3] = {0};

//	while(tof_i2c_read(buff, TOF_BL_CMD_STAT, 1) != 0x00);
	counter = 0;
	while(counter < READ_COUNT)
	{
		if(tof_i2c_read(read_back, TOF_CMD_STAT, 3) == TOF_ERROR)
			return TOF_ERROR;

		if(read_back[0] == 0x00 && read_back[1] == 0x00 && read_back[2] == 0xFF)
		{
			return TOF_OK;
		}

		counter++;
	}

	return TOF_ERROR;

}

/*@brief - bootloader command DOWNLOAD_INIT function
  @ret   - ok if success or error otherwise*/
uint8_t bootloader_cmd_download_init(uint8_t *write_buffer)
{
	uint8_t data_ptr = 0x29;

	write_buffer[0] = TOF_BL_CMD_STAT;
	write_buffer[1] = DOWNLOAD_INIT;
	write_buffer[2] = 0x01;
	write_buffer[3] = data_ptr;
	write_buffer[4] = tof_calculate_checksum(DOWNLOAD_INIT, 1, &data_ptr);
	if(tof_i2c_write_command(write_buffer, 5) != TOF_OK)
		return TOF_ERROR;

	return TOF_OK;
}

/*@brief - bootloader command ADDR_RAM function
  @ret   - oko if success or error otherwise*/
uint8_t bootloader_cmd_addr_ram(uint8_t *write_buffer)
{
	uint8_t data_ptr1[2] = {0x00, 0x00};

	write_buffer[0] = TOF_BL_CMD_STAT;
	write_buffer[1] = ADDR_RAM;
	write_buffer[2] = 0x02;
	write_buffer[3] = data_ptr1[0]; //addr_lsb;
	write_buffer[4] = data_ptr1[1]; //addr_msb;
	write_buffer[5] = tof_calculate_checksum(ADDR_RAM, 2, data_ptr1);
	if(tof_i2c_write_command(write_buffer, 6) != TOF_OK)
		return TOF_ERROR;

	return TOF_OK;
}

/*@brief - bootloader command W_RAM function
  @ret   - ok if success or error otherwise*/
uint8_t bootloader_cmd_w_ram(uint8_t *write_buffer)
{
	uint8_t i = 0,j = 0, k = 0;

	write_buffer[0] = TOF_BL_CMD_STAT;
	write_buffer[1] = W_RAM;
	write_buffer[2] = 0x80;

	for(j = 0; j < 55; j++)
	{
		for(i = 0; i < 128; i++)
		{
			write_buffer[3+i] = tof_bin_image[i+128*j];
		}
		write_buffer[3+i] = tof_calculate_checksum(W_RAM, 128, tof_bin_image+128*j);
		if(tof_i2c_write_command(write_buffer, 132) != TOF_OK)
			return TOF_ERROR;

		//poll until the bootloader signals READY
		if(bootloader_poll() != TOF_OK)
			return TOF_ERROR;
	}

//	for(k = 0; k < 8; k++)
//	{
//		write_buffer[3+k] = tof_bin_image[k+128*j];
//	}
//	write_buffer[2] = 0x08;
//	write_buffer[3+k] = tof_calculate_checksum(W_RAM, 8, tof_bin_image+128*j);
//	if(tof_i2c_write_command(write_buffer, 12) != TOF_OK)
//		return TOF_ERROR;

	for(k = 0; k < 88; k++)
		{
			write_buffer[3+k] = tof_bin_image[k+128*j];
		}
		write_buffer[2] = 0x58;
		write_buffer[3+k] = tof_calculate_checksum(W_RAM, 88, tof_bin_image+128*j);
		if(tof_i2c_write_command(write_buffer, 92) != TOF_OK)
			return TOF_ERROR;

	return TOF_OK;
}

/*@brief - bootloader command RAMREMAP_RESET function
  @ret   - ok if success or error otherwise*/
uint8_t bootloader_cmd_ramremap_reset(uint8_t *write_buffer)
{
	if(tof_i2c_read(&read_val, TOF_ENABLE,1) == TOF_ERROR)
		return TOF_ERROR;
	write_val = read_val | 0x21;
	if(tof_i2c_write(&write_val, TOF_ENABLE, 2) != TOF_OK)
		return TOF_ERROR;

	//RAMREMAP_RESET command
	write_buffer[0] = TOF_BL_CMD_STAT;
	write_buffer[1] = RAMREMAP_RESET;
	write_buffer[2] = 0x00;
	write_buffer[3] = 0xEE;
	if(tof_i2c_write_command(write_buffer, 4) != TOF_OK)
		return TOF_ERROR;

	return TOF_OK;
}

/*@brief - application configuration api
  @ret   - ok if success or error otherwise*/
uint8_t app_configuration(uint8_t spad_map, uint16_t time_lsb, uint8_t time_msb)
{
	uint8_t write_buff[3] = {0x24, time_lsb, time_msb};

	HAL_Delay(33);
	//load common config page LOAD_CONFIG_PAGE_COMMON command
	write_val = 0x16;
	if(tof_i2c_write(&write_val, TOF_CMD_STAT, 2) != TOF_OK)
		return TOF_ERROR;

	//Check that the command is executed: S 41 W 08 Sr 41 R N P
	if(check_app_cmd_exec_cmpltd() != TOF_OK)
		return TOF_ERROR;

	//Check that the configuration page is loaded: S 41 W 20 Sr 41 R A A A N P This should read back the values: 0x16 <do not care> 0xBC 0x00
	write_val = 0x16;
	if(check_config_page_loaded(&write_val) != TOF_OK)
		return TOF_ERROR;

	//Change the value of the measurement period to 100ms: S 41 W 24 64 00 P
	if(tof_i2c_write_command(write_buff, 3) != TOF_OK)
		return TOF_ERROR;

	//Select pre-defined SPAD mask 6: S 41 W 34 06 P
	write_val = spad_map;
	if(tof_i2c_write(&write_val, TOF_SPAD_MAP_ID, 2) != TOF_OK)
		return TOF_ERROR;

	//Configure the device for LOW on GPIO0 while the VCSEL is emitting light: S 41 W 31 03 P
	write_val = 0x03;
	if(tof_i2c_write(&write_val, TOF_GPIO_0, 2) != TOF_OK)
		return TOF_ERROR;

	//Write the common page to the device with command WRITE_CONFIG_PAGE: S 41 W 08 15 P
	write_val = 0x15;
	if(tof_i2c_write(&write_val, TOF_CMD_STAT, 2) != TOF_OK)
		return TOF_ERROR;

	//Check that the command is executed: S 41 W 08 Sr 41 R N P
	//This should read back as: 0x00 (if you read back a value >= 0x10 continue to read the register 0x08 until it changes to a value less than 0x10)
	if(check_app_cmd_exec_cmpltd() != TOF_OK)
		return TOF_ERROR;

	//Enable interrupts for results , S 41 W E2 02 P
	write_val = 0x02;
	if(tof_i2c_write(&write_val, TOF_INT_ENAB, 2) != TOF_OK)
		return TOF_ERROR;

	//Clear any old pending interrupts: S 41 W E1 FF P
	write_val = 0xFF;
	if(tof_i2c_write(&write_val, TOF_INT_STATUS, 2) != TOF_OK)
		return TOF_ERROR;

	return TOF_OK;
}

/*@brief - application factory callibration function
  @ret   - ok if success or error otherwise*/
uint8_t app_factory_callibration(void)
{
	uint8_t factory_buff[192] = {0}, i = 0;

	if(check_mode() == TOF_MODE_TMF8821)
	{
		if(tmf8821_store_factory_callibration(factory_buff) != TOF_OK)
			return TOF_ERROR;

		if(tmf8821_load_factory_callibration(factory_buff) != TOF_OK)
			return TOF_ERROR;

		return TOF_OK;
	}else if(check_mode() == TOF_MODE_TMF8828)
	{
		//Reset the factory calibration counter to ensure the first SPAD mask is calibrated first. The host needs to send the command RESET_FACTORY_CALIBRATION to the device:
		//S 41 W 08 1F P
		write_val = 0x1F;
		tof_i2c_write(&write_val, TOF_CMD_STAT, 2);

		//Initiate factory calibration through command FACTORY_CALIBRATION: S 41 W 08 20 P
		write_val = 0x20;
		tof_i2c_write(&write_val, TOF_CMD_STAT, 2);

		//Wait for the command to terminate by reading the CMD_STAT register, This cmd takes some time
		if(tof_i2c_read(&read_val, TOF_CMD_STAT, 1) == 0x01)
		{
			HAL_Delay(100);
			while(tof_i2c_read(&read_val, TOF_CMD_STAT, 1) != 0x00);

		}else if(tof_i2c_read(&read_val, TOF_CMD_STAT, 1) == 0x00);

		//The host needs to repeat steps 2-3 three times to perform calibrate for all four 4x4 SPAD masks.
		while(i < 3)
		{
			//Initiate factory calibration through command FACTORY_CALIBRATION: S 41 W 08 20 P
			write_val = 0x20;
			tof_i2c_write(&write_val, TOF_CMD_STAT, 2);

			//Wait for the command to terminate by reading the CMD_STAT register, This cmd takes some time
			if(tof_i2c_read(&read_val, TOF_CMD_STAT, 1) == 0x01)
			{
				HAL_Delay(100);
				while(tof_i2c_read(&read_val, TOF_CMD_STAT, 1) != 0x00);

			}else if(tof_i2c_read(&read_val, TOF_CMD_STAT, 1) == 0x00);

			i++;
		}

		i = 0;

		//To ensure that the firmware has the correct calibration load, the host can send the command
		//RESET_FACTORY_CALIBRATION to the device: S 41 W 08 1F P
		//This will not reset the factory calibration data, but ensure that the internal firmware points to the
		//factory calibration of the first 4x4 SPAD mask again.
		write_val = 0x1F;
		tof_i2c_write(&write_val, TOF_CMD_STAT, 2);

		//Next, the host must issue the LOAD_CONFIG_PAGE_FACTORY_CALIB command: S 41 W 08 19 P
		write_val = 0x19;
		tof_i2c_write(&write_val, TOF_CMD_STAT, 2);

		//Wait for the command to terminate by reading the CMD_STAT register
		while(tof_i2c_read(&read_val, TOF_CMD_STAT, 1) != 0x00);

		//Now the host must read out the complete factory calibration page: S 41 W 20 Sr 41 R A A A A .... <repeat> ... A N P
		//The host must read out the complete factory calibration page from address 0x20 .. 0xDF (inclusive)
		tof_i2c_read(factory_buff, TOF_CONFIG_RESULT, 192);

		//To switch to the next factory calibration data set, the host needs to send a
		//WRITE_CONFIG_PAGE to the device: S 41 W 08 15 P
		//When the device stores the factory calibration has it will automatically switch to the next factory
		//calibration set.
		write_val = 0x15;
		tof_i2c_write(&write_val, TOF_CMD_STAT, 2);

		//Check that the command is executed: S 41 W 08 Sr 41 R N P
		check_app_cmd_exec_cmpltd();

		//In order to read all four factory calibrations, the host needs to repeat steps 7-11 four times (the last WRITE_CONFIG_PAGE is optional).
		while(i < 4)
		{
			//Next, the host must issue the LOAD_CONFIG_PAGE_FACTORY_CALIB command: S 41 W 08 19 P
			write_val = 0x19;
			tof_i2c_write(&write_val, TOF_CMD_STAT, 2);

			//Wait for the command to terminate by reading the CMD_STAT register
			while(tof_i2c_read(&read_val, TOF_CMD_STAT, 1) != 0x00);

			//Now the host must read out the complete factory calibration page: S 41 W 20 Sr 41 R A A A A .... <repeat> ... A N P
			//The host must read out the complete factory calibration page from address 0x20 .. 0xDF (inclusive)
			tof_i2c_read(factory_buff, TOF_CONFIG_RESULT, 192);

			//To switch to the next factory calibration data set, the host needs to send a
			//WRITE_CONFIG_PAGE to the device: S 41 W 08 15 P
			//When the device stores the factory calibration has it will automatically switch to the next factory
			//calibration set.
			write_val = 0x15;
			tof_i2c_write(&write_val, TOF_CMD_STAT, 2);

			//Check that the command is executed: S 41 W 08 Sr 41 R N P
			check_app_cmd_exec_cmpltd();
		}
		i = 0;

		//The host must store all four factory calibration sets in its non-volatile memory.
		//TODO

		//Reset the factory calibration counter to ensure the first SPAD mask calibration is load first.
		//The host needs to send the command RESET_FACTORY_CALIBRATION to the device:
		//S 41 W 08 1F P
		write_val = 0x1F;
		tof_i2c_write(&write_val, TOF_CMD_STAT, 2);

		//Load the factory calibration page with command LOAD_CONFIG_PAGE_FACTORY_CALIB: S 41 W 08 19 P
		write_val = 0x19;
		tof_i2c_write(&write_val, TOF_CMD_STAT, 2);

		//Check that the command is executed: S 41 W 08 Sr 41 R N P
		check_app_cmd_exec_cmpltd();

		//Check that the configuration page is loaded: S 41 W 20 Sr 41 R A A A N P
		write_val = 0x19;
		check_config_page_loaded(&write_val);

		//Write the stored calibration data to the I2C registers: 0x24, 0x25, ... 0xDF.
		//TODO

		//Write back the calibration data with command WRITE_CONFIG_PAGE: S 41 W 08 15 P
		write_val = 0x15;
		tof_i2c_write(&write_val, TOF_CMD_STAT, 2);

		//Check that the command is executed: S 41 W 08 Sr 41 R N P
		check_app_cmd_exec_cmpltd();

		//The host has to repeat steps 2-7 four times
		while(i < 4)
		{
			//Load the factory calibration page with command LOAD_CONFIG_PAGE_FACTORY_CALIB: S 41 W 08 19 P
			write_val = 0x19;
			tof_i2c_write(&write_val, TOF_CMD_STAT, 2);

			//Check that the command is executed: S 41 W 08 Sr 41 R N P
			check_app_cmd_exec_cmpltd();

			//Check that the configuration page is loaded: S 41 W 20 Sr 41 R A A A N P
			write_val = 0x19;
			check_config_page_loaded(&write_val);

			//Write the stored calibration data to the I2C registers: 0x24, 0x25, ... 0xDF.
			//TODO

			//Write back the calibration data with command WRITE_CONFIG_PAGE: S 41 W 08 15 P
			write_val = 0x15;
			tof_i2c_write(&write_val, TOF_CMD_STAT, 2);

			//Check that the command is executed: S 41 W 08 Sr 41 R N P
			check_app_cmd_exec_cmpltd();
		}

		//wait for calliberation status to be success
		while(tof_i2c_read(&read_val, TOF_CALIBRATION_STATUS, 1) != 0x00);

		return TOF_OK;
	}

	return TOF_ERROR;
}

/*@brief - application measurements function
  @ret   - ok if success or error otherwise*/
uint8_t app_measurement(void)
{
	HAL_Delay(33);
	//Enable interrupts for results , S 41 W E2 02 P
	write_val = 0x02;
	if(tof_i2c_write(&write_val, TOF_INT_ENAB, 2) != TOF_OK)
		return TOF_ERROR;

	//Clear any old pending interrupts: S 41 W E1 FF P
	write_val = 0xFF;
	if(tof_i2c_write(&write_val, TOF_INT_STATUS, 2) != TOF_OK)
		return TOF_ERROR;

	//The starting of measurements is done by issuing the command MEASURE: S 41 W 08 10 P
	write_val = 0x10;
	if(tof_i2c_write(&write_val, TOF_CMD_STAT, 2) != TOF_OK)
		return TOF_ERROR;

	//The host should check that the command is accepted by reading back the register CMD_STAT
	if(tof_i2c_read(&read_val, TOF_CMD_STAT, 1) == 0x01)
	{
			;
	}else if(tof_i2c_read(&read_val, TOF_CMD_STAT, 1) >= 0x10)
	{
		counter = 0;
		while(counter < READ_COUNT)
		{
			if(tof_i2c_read(&read_val, TOF_CMD_STAT, 1) <= 0x10)
			{
				if(tof_i2c_read(&read_val, TOF_CMD_STAT, 1) == 0x01)
					break;
			}
			counter++;
		}
		if(counter == 3)
			return TOF_ERROR;
	}
	return TOF_OK;
}

/*@breif - application all configuration function sequence
  @ret   - ok if success or error otherwise*/
uint8_t app_full_config(void)
{
	//check APPID to be the measurement application ID.(APPID = 0x03)
	if(tof_i2c_read(&read_val, TOF_APPID, 1) == TOF_ERROR)
		return TOF_ERROR;

	if(read_val == 0x03)
	{
		//Read MODE register
		if(tof_i2c_read(&read_val, TOF_MODE, 1) == TOF_ERROR)
			return TOF_ERROR;

		if(read_val == 0x00)
		{
			//TMF8821 Application running

			//Application Configuration
			if(app_configuration(SPAD_MAP_ID_6, TOF_TIME_LSB, TOF_TIME_MSB) != TOF_OK)
				return TOF_ERROR;

			//Application Factory Callibration
			if(app_factory_callibration() != TOF_OK)
				return TOF_ERROR;

			//Application Measurements
			if(app_measurement() != TOF_OK)
				return TOF_ERROR;
			return TOF_OK;
		}
	}else if(read_val == 0x08)
	{
		//TMF8828 Application running
		return TOF_OK;
	}

	return TOF_ERROR;
}

/*@breif - custom spad mask configuration
  @ret   - ok if success or error otherwise*/
uint8_t custom_spad_mask(uint8_t spad_mask)
{
	//Load the common configuration page with command LOAD_CONFIG_PAGE_COMMON: S 41 W 08 16 P.
	write_val = 0x16;
	if(tof_i2c_write(&write_val, TOF_CMD_STAT, 2) != TOF_OK)
		return TOF_ERROR;

	//Check that the command is executed: S 41 W 08 Sr 41 R N P
	if(check_app_cmd_exec_cmpltd() != TOF_OK)
		return TOF_ERROR;

	//Check that the configuration page is loaded: S 41 W 20 Sr 41 R A A A N P
	write_val = 0x16;
	if(check_config_page_loaded(&write_val) != TOF_OK)
		return TOF_ERROR;

	if(spad_mask == TOF_3x3)
	{
		//For a 3x3-SPAD mask, the host needs to set user_defined_1 (14): S 41 W 34 0E P
		write_val = 0x0E;
		if(tof_i2c_write(&write_val, TOF_SPAD_MAP_ID, 2) != TOF_OK)
			return TOF_ERROR;
	}else if(spad_mask == TOF_3x6_4x4)
	{
		//For a 3x6 or 4x4 time-multiplexed SPAD mask, the host has to set the register to user_defined_2 (15): S 41 W 34 0F P
		write_val = 0x0F;
		if(tof_i2c_write(&write_val, TOF_SPAD_MAP_ID, 2) != TOF_OK)
			return TOF_ERROR;
	}else
	{
		return TOF_ERROR;
	}

	//Write the common page to the device with command WRITE_CONFIG_PAGE: S 41 W 08 15 P
	write_val = 0x15;
	if(tof_i2c_write(&write_val, TOF_CMD_STAT, 2) != TOF_OK)
		return TOF_ERROR;

	//Check that the command is executed: S 41 W 08 Sr 41 R N P
	if(check_app_cmd_exec_cmpltd() != TOF_OK)
		return TOF_ERROR;

	//Load SPAD page 1 with command LOAD_CONFIG_PAGE_SPAD_1: S 41 W 08 17 P.
	write_val = 0x17;
	if(tof_i2c_write(&write_val, TOF_CMD_STAT, 2) != TOF_OK)
		return TOF_ERROR;

	//Check that the command is executed: S 41 W 08 Sr 41 R N P
	if(check_app_cmd_exec_cmpltd() != TOF_OK)
		return TOF_ERROR;

	//Check that the SPAD page 1 is loaded: S 41 W 20 Sr 41 R A A A N P
	write_val = 0x17;
	if(check_config_page_loaded(&write_val) != TOF_OK)
		return TOF_ERROR;

	//Load the custom SPAD page 1 data to address 0x24-0x90: S 41 W 24 <SPAD page 1 data> P.
	//TODO

	//Write the SPAD page with command WRITE_CONFIG_PAGE: S 41 W 08 15 P
	write_val = 0x15;
	if(tof_i2c_write(&write_val, TOF_CMD_STAT, 2) != TOF_OK)
		return TOF_ERROR;

	//Check that the command is executed: S 41 W 08 Sr 41 R N P
	if(check_app_cmd_exec_cmpltd() != TOF_OK)
		return TOF_ERROR;

	if(spad_mask == TOF_3x6_4x4)
	{
		//Load SPAD page 2 with command LOAD_CONFIG_PAGE_SPAD_1: S 41 W 08 18 P.
		write_val = 0x18;
		if(tof_i2c_write(&write_val, TOF_CMD_STAT, 2) != TOF_OK)
			return TOF_ERROR;

		//Check that the command is executed: S 41 W 08 Sr 41 R N P
		if(check_app_cmd_exec_cmpltd() != TOF_OK)
			return TOF_ERROR;

		//Check that SPAD page 2 is loaded: S 41 W 20 Sr 41 R A A A N P
		write_val = 0x18;
		if(check_config_page_loaded(&write_val) != TOF_OK)
			return TOF_ERROR;

		//Load the SPAD page 2 data to address 0x24-0x90: S 41 W 24 <SPAD page 2 data> P.
		//TODO

		//Write the SPAD page with command WRITE_CONFIG_PAGE: S 41 W 08 15 P
		write_val = 0x15;
		if(tof_i2c_write(&write_val, TOF_CMD_STAT, 2) != TOF_OK)
			return TOF_ERROR;

		//Check that the command is executed: S 41 W 08 Sr 41 R N P
		if(check_app_cmd_exec_cmpltd() != TOF_OK)
			return TOF_ERROR;
	}

	return TOF_OK;
}

/*@breif - switch between short and long range
  @ret   - ok if success or error otherwise*/
uint8_t switch_between_active_ranges(uint8_t range)
{
	if(range == TOF_short_range)
	{
		//To switch short-range mode, the host needs to send the command SHORT RANGE_ACCURACY (0x6E): S 41 W 08 6E P.
		write_val = 0x6E;
		if(tof_i2c_write(&write_val, TOF_CMD_STAT, 2) != TOF_OK)
			return TOF_ERROR;
		return TOF_OK;

	}else if(range == TOF_long_range)
	{
		//To switch back to long-range mode, the host needs to send the command LONG RANGE_ACCURACY (0x6F): S 41 W 08 6F P.
		write_val = 0x6F;
		if(tof_i2c_write(&write_val, TOF_CMD_STAT, 2) != TOF_OK)
			return TOF_ERROR;
		return TOF_OK;
	}

	return TOF_ERROR;
}

/*@breif - execute measure stop command sequence
  @ret   - TOF_OK if success or TOF_ERROR otherwise*/
uint8_t app_cmd_measure_stop(void)
{
	//Check that the device is not in STANDBY_TIMED by reading the ENABLE register: S 41 W E0 Sr 41 R N P, which must have the value b_01xx_0001 to continue.
//	while((tof_i2c_read(&stop_val, TOF_ENABLE, 1) & 0x41) != 0x41);
	counter = 0;
	while(counter < READ_COUNT)
		{
			if((tof_i2c_read(&read_val, TOF_ENABLE, 1) & 0x41) == 0x41)
				break;
			counter++;
		}
	if(counter == 3)
		return TOF_ERROR;

	//The host may stop the device by issuing the command STOP: S 41 W 08 FF P
	write_val = 0xFF;
	if(tof_i2c_write(&write_val, TOF_CMD_STAT, 2) != TOF_OK)
		return TOF_ERROR;

	//The host should check that the command is executed successfully by reading back the register CMD_STAT: S 41 W 08 Sr 41 R N P.
	HAL_Delay(2);
//	while(tof_i2c_read(&read_val, TOF_CMD_STAT, 1) != 0x00);
	counter = 0;
	while(counter < READ_COUNT)
		{
			if(tof_i2c_read(&read_val, TOF_CMD_STAT, 1) == 0x00)
				return TOF_OK;
			counter++;
		}

	return TOF_ERROR;
}

/*@breif -
  @ret   - */
uint8_t enter_standby_mode(void)
{
	//If the measurement application is performing measurements, first stop the measurement application
	if(app_cmd_measure_stop() != TOF_OK)
		return TOF_ERROR;

	//Read back register ENABLE (0xE0) and check that the device is ready (reads back as b_01xx_0001).
	counter = 0;
	while(counter < READ_COUNT)
		{
			if((tof_i2c_read(&read_val, TOF_ENABLE, 1) & 0x41) == 0x41)
				break;
			counter++;
		}
	if(counter == 3)
		return TOF_ERROR;

	//Write to register ENABLE the value b_00xx_0000.
	if(tof_i2c_read(&read_val, TOF_ENABLE, 1) == TOF_ERROR)
		return TOF_ERROR;
	write_val = read_val & 0x30;
	if(tof_i2c_write(&write_val, TOF_ENABLE, 2) != TOF_OK)
		return TOF_ERROR;

	//Read back register ENABLE until it has the value b_00xx_0010.
	counter = 0;
	while(counter < READ_COUNT)
		{
			if((tof_i2c_read(&read_val, TOF_ENABLE, 1) & 0x02) == 0x02)
				return TOF_OK;
			counter++;
		}

	return TOF_ERROR;

}

/*@breif - wake up the device
  @ret   - TOF_OK if success or TOF_ERROR otherwise*/
uint8_t wake_up_device(void)
{
	//Read back register ENABLE (0xE0) and check that the device is in STANDBY state (reads back as b_00xx_0010).
	if((tof_i2c_read(&read_val, TOF_ENABLE, 1) & 0x02) == 0x02)
	{
		//Write to register ENABLE the value b_00xx_0001.
		if(tof_i2c_read(&read_val, TOF_ENABLE, 1) == TOF_ERROR)
			return TOF_ERROR;
		write_val = (read_val & 0x30) | 0x01;
		if(tof_i2c_write(&write_val, TOF_ENABLE, 2) != TOF_OK)
			return TOF_ERROR;

		//Read back register ENABLE until it has the value b_01xx_0001.
		counter = 0;
		while(counter < READ_COUNT)
			{
				if((tof_i2c_read(&read_val, TOF_ENABLE, 1) & 0x41) == 0x41)
					return TOF_OK;
				counter++;
			}
		return TOF_ERROR;
	}

	return TOF_ERROR;
}

/*@breif - power cycle sequence
  @ret   - void*/
void power_cycle(void)
{
	  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0|USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);
	  HAL_Delay(1);
	  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0|USB_PowerSwitchOn_Pin, GPIO_PIN_SET);
}

/*@breif - check if the device is ready
  @ret   - TOF_READY if success or TOF_NOT_READY otherwise*/
uint8_t is_device_ready(void)
{
	//The device is only ready for host communication if bit 6 of register ENABLE is set. 	S 41 W E0 Sr 41 R N P, Should return the value: b_01xx_0001
	if((tof_i2c_read(&read_val, TOF_ENABLE, 1) & 0x41) == 0x41)
	{
		return TOF_READY;
	}

	return TOF_NOT_READY;
}

/*@breif - exit from standby timed mode
  @ret   - TOF_OK if success or TOF_ERROR otherwise*/
uint8_t exit_standby_timed_mode(void)
{
	//Stop handling the result interrupts
	//Perform a wake-up sequence: S 41 W E0 21 P
	if(wake_up_device() != TOF_OK)
		return TOF_ERROR;

	//Check that the device is not in STANDBY_TIMED by reading the ENABLE register: S 41 W E0 Sr 41 R N P, which must have the value b_01xx_0001 to continue.
	if((tof_i2c_read(&read_val, TOF_ENABLE, 1) & 0x41) != 0x41)
		return TOF_ERROR;

	//Issue the STOP command: S 41 W 08 FF P
	write_val = 0xFF;
	if(tof_i2c_write(&write_val, TOF_CMD_STAT, 2) != TOF_OK)
		return TOF_ERROR;

	//Wait for the STOP to be successfully executed by reading CMD_STAT: S 41 W 08 Sr 41 R N P
	counter = 0;
	while(counter < READ_COUNT)
	{
		if(tof_i2c_read(&read_val, TOF_CMD_STAT, 1) == 0x00)
			return TOF_OK;
		counter++;
	}
	return TOF_ERROR;
}

/*@breif - check applicatio ID 0x03 - application, 0x80 for bootloader
  @ret   - TOF_OK if success or TOF_ERROR otherwise*/
uint8_t check_app_id(void)
{
	uint8_t id[4] = {0};

	if(tof_i2c_read(id, TOF_APPID, 4) == TOF_ERROR)
		return TOF_ERROR;

	return TOF_OK;
}

/*@breif -
  @ret   - */
void bootloader_after_warm_start(void)
{
	//TODO
	while(1);
}

/*@breif -
  @ret   - */
uint8_t bootloader_status(void)
{
	//TODO
	return TOF_OK;
}

/*@breif - check whether application command has been executed or not
  @ret   - TOF_OK if success or TOF_ERROR otherwise*/
uint8_t check_app_cmd_exec_cmpltd(void)
{
	HAL_Delay(3);
	if(tof_i2c_read(&read_val, TOF_CMD_STAT, 1) == 0x00)
		return TOF_OK;
	else if(read_val >= 0x10)
	{
//		while(!(tof_i2c_read(&read, TOF_CMD_STAT, 1) < 0x10));
		counter = 0;
		while(counter < READ_COUNT)
			{
				if(tof_i2c_read(&read_val, TOF_CMD_STAT, 1) < 0x10)
				{
					if(tof_i2c_read(&read_val, TOF_CMD_STAT, 1) == 0x00)
							return TOF_OK;
					else
						return TOF_ERROR;
				}
				counter++;
			}

		return TOF_ERROR;
	}

	return TOF_ERROR;
}

/*@breif - check whether configuration page has been loaded or not
  @ret   - TOF_OK if success or TOF_ERROR otherwise*/
uint8_t check_config_page_loaded(uint8_t *write_reg)
{
	uint8_t page[4] = {0};

	counter = 0;
	while(counter < READ_COUNT)
	{
		if(tof_i2c_read(page, TOF_CONFIG_RESULT, 4) == TOF_ERROR)
			return TOF_ERROR;
		if(page[0] == *write_reg && page[2] == 0xBC && page[3] == 0x00)
			return TOF_OK;
		counter++;
	}

	return TOF_ERROR;
}

/*@breif - check firmware support for short range
  @ret   - TOF_OK if success or TOF_ERROR otherwise*/
uint8_t check_fw_support_for_short_range(void)
{
	//read register BUILD_VERSION (0x3)
	if((tof_i2c_read(&read_val, TOF_BUILD_TYPE, 1) & 0x10) == 0x10)
		return SHORT_RANGE_SUPPORTED;
	else
		return SHORT_RANGE_NOT_SUPPORTED;
}

/*@breif - check active range function
  @ret   - TOF_OK if success or TOF_ERROR otherwise*/
uint8_t check_active_range(void)
{
	if(tof_i2c_read(&read_val, 0x19, 1) == TOF_ERROR)
		return TOF_ERROR;
	else
		return TOF_OK;
}

/*@breif - switch between tmf8828 and tmf8821 mode
  @ret   - ok if success or error otherwise*/
uint8_t switch_bw_tmf8828_and_tmf8821_mode(uint8_t mode)
{
	//switch from tmf8828 to tmf8821 mode
	if(mode == TOF_MODE_TMF8821)
	{
		if(check_mode() != TOF_MODE_TMF8821)
		{
			//Send the command to switch to TMF8821 mode: S 41 W 08 65 P
			write_val = 0x65;
			if(tof_i2c_write(&write_val, TOF_CMD_STAT, 2) != TOF_OK)
				return TOF_ERROR;

			//Wait for the application to restart in TMF8821 mode (see TMF882X datasheet for timing) 65ms
			HAL_Delay(65);
		//	while(tof_i2c_read(&read_val, TOF_CMD_STAT, 1) != 0x00);
			counter = 0;
			while(counter < READ_COUNT)
			{
				if(tof_i2c_read(&read_val, TOF_CMD_STAT, 1) == 0x00)
					break;
				counter++;
			}

			if(counter == 3)
				return TOF_ERROR;
		}else
			return TOF_OK;
	}else if(mode == TOF_MODE_TMF8828)
	{
		if(check_mode() != TOF_MODE_TMF8828)
		{
			//Send the command to switch to TMF8828 mode: S 41 W 08 6C P
			write_val = 0x6C;
			if(tof_i2c_write(&write_val, TOF_CMD_STAT, 2) != TOF_OK)
				return TOF_ERROR;

			//Wait for the application to restart in TMF8821 mode (see TMF882X datasheet for timing)
			HAL_Delay(115);
			//	while(tof_i2c_read(&read_val, TOF_CMD_STAT, 1) != 0x00);
			counter = 0;
			while(counter < READ_COUNT)
			{
				if(tof_i2c_read(&read_val, TOF_CMD_STAT, 1) == 0x00)
					break;
				counter++;
			}
			if(counter == 3)
				return TOF_ERROR;
		}else
			return TOF_OK;
	}
//	//Configure the device via the configuration pages
//	if(application_configuration() != TOF_OK)
//		return TOF_ERROR;

	return TOF_OK;
}

/*@breif - check application mode TMF8828(0x08) or TMF8821(0x00)
  @ret   - return TMF8828/TMF8821 macros or TOF_ERROR otherwise*/
uint8_t check_mode(void)
{
	if(tof_i2c_read(&read_val, TOF_MODE, 1) == TOF_ERROR)
		return TOF_ERROR;
	if(read_val == 0x00)
		return TOF_MODE_TMF8821;
	else if(read_val == TOF_MODE_TMF8828)
		return TOF_MODE_TMF8828;

	return TOF_ERROR;
}

uint8_t tmf8821_store_factory_callibration(uint8_t *factory_buff)
{
	//Initiate factory calibration through command FACTORY_CALIBRATION: S 41 W 08 20 P
	write_val = 0x20;
	if(tof_i2c_write(&write_val, TOF_CMD_STAT, 2) != TOF_OK)
		return TOF_ERROR;

	//Wait for the command to terminate by reading the CMD_STAT This command takes some time
	if(tof_i2c_read(&read_val, TOF_CMD_STAT, 1) == 0x01)
	{
		HAL_Delay(33);
//			while(tof_i2c_read(&read_val, TOF_CMD_STAT, 1) != 0x00);
		counter = 0;
		while(counter < READ_COUNT)
		{
			if(tof_i2c_read(&read_val, TOF_CMD_STAT, 1) == 0x00)
				break;
			counter++;
		}
	}else if(tof_i2c_read(&read_val, TOF_CMD_STAT, 1) == 0x00);

	if(counter == 3)
		return TOF_ERROR;

	//the host must issue the LOAD_CONFIG_PAGE_FACTORY_CALIB command: S 41 W 08 19 P
	write_val = 0x19;
	if(tof_i2c_write(&write_val, TOF_CMD_STAT, 2) != TOF_OK)
		return TOF_ERROR;

	HAL_Delay(190);
	//Wait for the command to terminate by reading the CMD_STAT register
//		while(tof_i2c_read(&read_val, TOF_CMD_STAT, 1) != 0x00);
	counter = 0;
	while(counter < READ_COUNT)
	{
		if(tof_i2c_read(&read_val, TOF_CMD_STAT, 1) == 0x00)
			break;
		counter++;
	}

	if(counter == 3)
		return TOF_ERROR;

	//Now the host must read out the complete factory calibration page: S 41 W 20 Sr 41 R A A A A .... <repeat> ... A N P
	//The host must read out the complete factory calibration page from address 0x20 .. 0xDF (inclusive)
	if(tof_i2c_read(factory_buff, TOF_CONFIG_RESULT, 192) == TOF_ERROR)
		return TOF_ERROR;

	HAL_Delay(33);
	return TOF_OK;
}

uint8_t tmf8821_load_factory_callibration(uint8_t *factory_buff)
{
	//Load the factory calibration page with command LOAD_CONFIG_PAGE_FACTORY_CALIB: S 41 W 08 19 P
	write_val = 0x19;
	if(tof_i2c_write(&write_val, TOF_CMD_STAT, 2) != TOF_OK)
		return TOF_ERROR;

	//Check that the command is executed
	if(check_app_cmd_exec_cmpltd() != TOF_OK)
		return TOF_ERROR;

	//Check that the configuration page is loaded: S 41 W 20 Sr 41 R A A A N P This should read back the values: 0x19 <do not care> 0xBC 0x00
	write_val = 0x19;
	if(check_config_page_loaded(&write_val) != TOF_OK)
		return TOF_ERROR;

	//Write the stored calibration data to the I²C registers: 0x24, 0x25, … 0xDF.
	write_val = TOF_PERIOD_MS_LSB;
	HAL_I2C_Master_Transmit(TOF_I2C_Handle, TOF_I2C_ADDR, &write_val, 1, HAL_MAX_DELAY);
	if(tof_i2c_write_command(factory_buff+4, 188) != TOF_OK)
		return TOF_ERROR;

	//Write back the calibration data with command WRITE_CONFIG_PAGE: S 41 W 08 15 P
	write_val = 0x15;
	if(tof_i2c_write(&write_val, TOF_CMD_STAT, 2) != TOF_OK)
		return TOF_ERROR;

	//Check that the command is executed
	if(check_app_cmd_exec_cmpltd() != TOF_OK)
		return TOF_ERROR;

	//Note that if the device does not have a valid factory calibration data or the factory calibration was done
	//for a different SPAD map the device will report a warning in register 0x07 CALIBRATION_STATUS.
	//The value of 0x31 means that no factory calibration has been loaded, the value of 0x32 means that
	//the factory calibration does not match to the selected SPAD map.
//		while(tof_i2c_read(&read_val, TOF_CALIBRATION_STATUS, 1) != 0x00);
//		counter = 0;
//		while(counter < READ_COUNT)
//		{
//			if(tof_i2c_read(&read_val, TOF_CALIBRATION_STATUS, 1) == 0x00)
//				break;
//			counter++;
//		}
//		if(counter == 3)
//			return TOF_ERROR;

	return TOF_OK;

}

uint8_t startup(void)
{
	  //after power on pull enable line HIGH
	  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0|USB_PowerSwitchOn_Pin, GPIO_PIN_SET);

	  HAL_Delay(2);
	  //write 0x01 to ENABLE register 0xE0
	  if(tof_i2c_read(&read_val, TOF_ENABLE, 1) == TOF_ERROR)
		  return TOF_ERROR;
	  write_val = read_val | 0x01;
	  if(tof_i2c_write(&write_val, TOF_ENABLE, 2) != TOF_OK)
		  return TOF_ERROR;

	  //Poll register ENABLE until the value 0x41 is read back
	  if(tof_i2c_read(&read_val, TOF_ENABLE, 1) == TOF_ERROR)
		  return TOF_ERROR;

	  if((read_val & 0x01) == 0x01){
//		  while((tof_i2c_read(&read_buffer, TOF_ENABLE, 1) & 0x41) != 0x41);
		  counter = 0;
		  while(counter < READ_COUNT)
		  {
			  if((tof_i2c_read(&read_val, TOF_ENABLE, 1) & 0x41) == 0x41)
				break;
			  counter++;
		  }
		  if(counter == 3)
			  return TOF_ERROR;
	  }
	  else if((read_val & 0x02) == 0x02)
	  {
		  if(wake_up_device() != TOF_OK)
			  return TOF_ERROR;
	  }
	  else if((read_val & 0x06) == 0x06)
	  {
		  //(either force wake up from standby timed mode by writing b_00xx_0001 to ENABLE register to send STOP command to abort measurements and enter IDLE states)
		  if(exit_standby_timed_mode() != TOF_OK)
			  return TOF_ERROR;
	  }

//	  //Read Minor Register
//	  if(tof_i2c_read(&read_val, TOF_MINOR, 1) == TOF_ERROR)
//		  return TOF_ERROR;
//
//	  //Read ID Rergister
//	  if(tof_i2c_read(&read_val, TOF_ID, 1) == TOF_ERROR)
//		  return TOF_ERROR;
//
//	  //Read REVID Register
//	  if(tof_i2c_read(&read_val, TOF_REVID, 1) == TOF_ERROR)
//		  return TOF_ERROR;

	 return TOF_OK;
}

uint8_t image_download(void)
{
	  //Read the register 0x00 = APPID to find out which application is running.
	  if(tof_i2c_read(&read_val, TOF_APPID, 1) == TOF_ERROR)
		  return TOF_ERROR;

	  if(read_val == 0x80)
	  {
		  //bootloader is running , download  patch image
		 if(tof_patch_image_download() != TOF_OK)
			 return TOF_ERROR;
		 return TOF_OK;
	  }else if(read_val == 0x03)
	  {
		  //application is running
		  while(1);
	  }
	  return TOF_ERROR;
}

uint8_t application_configuration(void)
{
	//Application configuration + factory callibration + measurements
	if(app_full_config() != TOF_OK)
		return TOF_ERROR;
	return TOF_OK;
}

/*patch image download function*/
uint8_t tof_patch_image_download(void)
{
	uint8_t write_buffer[150] = {0};

	//issue DOWNLOAD INIT command		S 41 W 08 14 01 29 C1 P
	if(bootloader_cmd_download_init(write_buffer) != TOF_OK)
		return TOF_ERROR;

	//poll until the bootloader signals READY
	if(bootloader_poll() != TOF_OK)
		return TOF_ERROR;

	//set the destination address to 0x0000 with the command ADDR_RAM
	if(bootloader_cmd_addr_ram(write_buffer) != TOF_OK)
		return TOF_ERROR;

	//poll until the bootloader signals READY
	if(bootloader_poll() != TOF_OK)
		return TOF_ERROR;

	//load the image with a series of the W_RAM commands
	if(bootloader_cmd_w_ram(write_buffer) != TOF_OK)
		return TOF_ERROR;

	//poll until the bootloader signals READY
	if(bootloader_poll() != TOF_OK)
		return TOF_ERROR;

	//When all data packets have successfully been downloaded the host must issue a RAMREMAP_RESET command
	//Before executing this command, set powerup_select = 2 in ENABLE register bits 4 & 5
	if(bootloader_cmd_ramremap_reset(write_buffer) != TOF_OK)
		return TOF_ERROR;

	//poll only for first byte of cmd stat register
	while(tof_i2c_read(&read_val, TOF_CMD_STAT, 1) != 0x00);

	//if after a maximum wait time of 2.5 ms the register APPID does not read back with the measurement application ID, the download failed and the host should power cycle the device
	while(1)
	{
		if(tof_i2c_read(&read_val, TOF_APPID, 1) == TOF_ERROR)
			return TOF_ERROR;
		if(read_val == 0x03)
		{
			break;
		}
		else
		{
			HAL_Delay(3);
			if(tof_i2c_read(&read_val, TOF_APPID, 1) == TOF_ERROR)
				return TOF_ERROR;
			if(read_val == 0x03)
			{
				break;
			}else
			{
				power_cycle();
				break;
			}
		}
	}

	return TOF_OK;
}

uint8_t measurements_results(void)
{
	//measurements results

	//read 132 bytes block read from register 0x20
	if(tof_i2c_read(result_buff, TOF_CONFIG_RESULT, 132) == TOF_ERROR)
		return TOF_ERROR;

	return TOF_OK;
}


uint8_t clock_correction(uint8_t *buff, uint32_t *host_diff, uint32_t *tof_diff)
{
	uint16_t distance = 0;
	distance = ((uint16_t)buff[26] << 8) | (uint16_t)buff[25];

	float correction_factor = (float)(*host_diff / (*tof_diff * 0.0002));

	distance = (uint16_t)((float)distance * correction_factor);
//	distance = (uint16_t)(distance * correction_factor);

	return TOF_OK;

}


#endif //TOF_REG_C



