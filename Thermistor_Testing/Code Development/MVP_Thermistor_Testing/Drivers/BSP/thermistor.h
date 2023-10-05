/******************************************************************************
  * file name:    thermistor.h
  * author name:  Sukhdeep Singh Soni (sukhdeepsingh@focally.in)
  * module name:  MVP Thermistor Testing
  * file version: 1.0
  * about:        This file provides the headers related to fuel gauge thermistor:
  ******************************************************************************
**/

#include "main.h"
#include "i2c.h"
#include "gpio.h"

extern uint8_t slave_address;
extern uint16_t ret_val16,Temp_val, status0, status1;
extern uint8_t buff[2], tmp[2];
extern uint8_t rst[2];

enum reg_addr {status_reg = 0x00, config_reg = 0x1d,
 	 	 	   config2_reg = 0xbb, temp_reg = 0x08,
			   talrtth_reg = 0x02, devname_reg = 0x21};

uint8_t i2c_sacnner(void);
uint16_t chip_id(uint8_t reg_address);
uint8_t i2c_read_8bit(uint8_t *buff, uint8_t reg_address, uint8_t bytes);
uint8_t i2c_write_8bit(uint8_t *buff, uint8_t reg_address, uint8_t bytes, uint8_t data);
uint16_t i2c_write_16bit(uint8_t *buff, uint8_t reg_address, uint8_t bytes, uint8_t *data);
uint16_t i2c_read_16bit(uint8_t *buff, uint8_t reg_address, uint8_t bytes);
void mvp_thermistor_init(void);
void mvp_thermistor_process(void);

//end of file
