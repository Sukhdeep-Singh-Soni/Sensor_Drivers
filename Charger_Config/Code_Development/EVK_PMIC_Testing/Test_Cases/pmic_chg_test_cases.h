/**
  ******************************************************************************
  * file name:    pmic_chg_test_cases.h
  * author name:  Sukhdeep Singh Soni (sukhdeepsingh@focally.in)
  * module name:  PMIC Charger
  * file version: 1.0
  * about:        This file provides firmware functions declarations and enums to manage the following
  *           	  + Test Chip ID
  *           	  + Test SBB Voltages
  *           	  + Test LDO Voltages
  *				  + Test I2C Transmission
  *				  + Test Register Read/Write
  *				  + Test Charger Status
  *				  + Test Whether Charger gets suspended or not
  *				  +
  ******************************************************************************
**/

/*Includes*/
#include "main.h"
#include "i2c.h"


enum Reg_addresses {
							CHG_OK = 0, CHG_DISCON = 1, CHG_ERR = 2, CHG_FULL = 3,
							CHG_STAT_B = 0x03, CHG_G = 0x26, CID = 0x14
						   };

/*defines*/
#define SlaveAddress 0x48<<1
/*Global variables*/

/*Declarations of Test Functions*/
uint8_t Test_Chip_ID(uint8_t *Buffer);
uint8_t Test_Charger_Status(uint8_t *Buffer);
uint8_t Test_Local_I2C_Transmit(uint8_t *Buffer, uint8_t Bytes);
uint8_t Test_Local_I2C_Receive(uint8_t *Buffer, uint8_t Bytes);
uint8_t Test_Reg_Write(uint8_t Reg_Address, uint8_t data, uint8_t *Buffer);
uint8_t Test_Reg_Read(uint8_t Reg_Address, uint8_t *Buffer);
uint8_t Test_Suspend_Charger(uint8_t *Buffer);
uint8_t Test_SBBx_Voltage(uint8_t RegAddr, uint8_t *Buffer);
uint8_t Test_LDOx_Voltage(uint8_t RegAddr, uint8_t *Buffer);
uint8_t Test_I2C_Scanner(uint8_t *Buffer);

/*End of File*/
