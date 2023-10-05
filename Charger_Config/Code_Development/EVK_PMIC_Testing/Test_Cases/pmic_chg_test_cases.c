/**
  ******************************************************************************
  * file name:    pmic_chg_test_cases.c
  * author name:  Sukhdeep Singh Soni (sukhdeepsingh@focally.in)
  * module name:  PMIC Charger
  * file version: 1.0
  * about:        This file provides firmware functions to manage the following
  *           	  + Test Chip ID
  *           	  + Test SBB Voltages
  *           	  + Test LDO Voltages
  *				  + Test I2C Transmission
  *				  + Test Register Read/Write
  *				  + Test Charger Status
  *				  + Test Whether Charger gets suspended or not
  ******************************************************************************
**/
	#include "pmic_chg_test_cases.h"

	uint8_t PMIC_Read_Reg(uint8_t Reg_Address, uint8_t *Buffer);

	/*Test Chip ID
	@retval 0 - Correct Chip ID
	  	  	1 - Incorrect Chip ID*/
	uint8_t Test_Chip_ID(uint8_t *Buffer)
	{
		uint8_t ret, Id;
		Id = PMIC_Read_Reg(CID, Buffer);
		Id &= 0x0F;
		if(Id == 0x06)
			ret = 0; //Chip ID is correct
		else
			ret = 1; // Chip ID is Incorrect

		return ret;
	}
	/*End*/

	/*Test SBB Voltages
	  @retval - 1 - I2C transmission error
	            Buffer[0] - recieved value*/
	uint8_t Test_SBBx_Voltage(uint8_t RegAddr, uint8_t *Buffer)
	{
		uint8_t transmit, recieve, ret;
		Buffer[0] = RegAddr;
		transmit = HAL_I2C_Master_Transmit(&hi2c1, SlaveAddress, Buffer, 1, 100);
		if(transmit != HAL_OK)
			ret = 1;
		else
		{
		recieve = HAL_I2C_Master_Receive(&hi2c1, SlaveAddress, Buffer, 1, 100);
		if(recieve != HAL_OK)
			ret = 1;
		else
			ret = Buffer[0];
		}
		return ret;
	}
	/*End*/

	/*Test LDO Voltages
	   @retval - 1 - I2C transmission error
	            Buffer[0] - recieved value*/
	uint8_t Test_LDOx_Voltage(uint8_t RegAddr, uint8_t *Buffer)
	{
		uint8_t transmit, recieve, ret;
		Buffer[0] = RegAddr;
		transmit = HAL_I2C_Master_Transmit(&hi2c1, SlaveAddress, Buffer, 1, 100);
		if(transmit != HAL_OK)
			ret = 1;
		else
		{
		recieve = HAL_I2C_Master_Receive(&hi2c1, SlaveAddress, Buffer, 1, 100);
		if(recieve != HAL_OK)
			ret = 1;
		else
			ret = Buffer[0];
		}
		return ret;
	}
	/*End*/

	/*Test I2C Scanner
	  @retval - 0 - I2C working
	            1 - I2C transmission error*/
	uint8_t Test_I2C_Scanner(uint8_t *Buffer)
	{
		uint8_t i,received,ret;
	for(i = 1; i <= 0x90; i++)
	  {
		  received = HAL_I2C_Master_Receive(&hi2c1, i, Buffer, 1, 100);
		  if(received != HAL_OK)
		  {
			  ret = 1;
		  }
		  else
		  {
			  ret = 0;

		  }
	  }
	  return ret;
	}
	/*End*/

	/*Test I2C transmit
	@retval - 1 - I2C transmit error
			  0 - Transmit Sucess*/
	uint8_t Test_Local_I2C_Transmit(uint8_t *Buffer, uint8_t Bytes)
	{
		uint8_t transmit,ret;
		transmit = HAL_I2C_Master_Transmit(&hi2c1, SlaveAddress, Buffer, Bytes, HAL_MAX_DELAY);
		if(transmit != HAL_OK)
		{
			ret = 1;
		}
		else
		{
			ret = 0;
		}
		return ret;
	}
	/*END*/

	/*Test I2C transmission recieve
	@retval - 1 - I2C transmission error
			  0 - Recieve Success*/
	uint8_t Test_Local_I2C_Receive(uint8_t *Buffer, uint8_t Bytes)
	{
		uint8_t received,ret;
		ret = HAL_I2C_Master_Transmit(&hi2c1, SlaveAddress, Buffer, Bytes, HAL_MAX_DELAY);
		if(ret != HAL_OK)
			ret = 1;
		else
		{
			received = HAL_I2C_Master_Receive(&hi2c1, SlaveAddress, Buffer, Bytes, HAL_MAX_DELAY);
			if(received != HAL_OK)
				ret = 1;
			else
			{
				ret = 0;
			}
		}
		return ret;
	}
	/*END*/

	/*Test Register Write
	  @retval - 0 - Write successful
	            1 - Write Unsuccessful*/
	uint8_t Test_Reg_Write(uint8_t Reg_Address, uint8_t data, uint8_t *Buffer)
	{
		uint8_t ret;
		Buffer[0] = Reg_Address;
		Buffer[1] = data;
		if(HAL_I2C_Master_Transmit(&hi2c1, SlaveAddress, Buffer, 2, 100) != HAL_OK)
			ret = 1;
		else
		{
			if(HAL_I2C_Master_Transmit(&hi2c1, SlaveAddress, Buffer, 1, 100)!= HAL_OK)
				ret = 1;
			else
			{
				if(HAL_I2C_Master_Receive(&hi2c1, SlaveAddress, Buffer, 1, 100) != HAL_OK)
					ret = 1;
				else
				{
					ret = Buffer[0];
					if(ret == data)
						ret = 0;
					else
						ret = 1;
				}
			}
		}
		return ret;
	}
	/*End*/

	/*Test Register Read
	  @retval - Buffer[0] - register value
	            1 - Read Unsuccessful*/
	uint8_t Test_Reg_Read(uint8_t Reg_Address, uint8_t *Buffer)
	{
		uint8_t ret;
		Buffer[0] = Reg_Address;
		if(HAL_I2C_Master_Transmit(&hi2c1, SlaveAddress, Buffer, 1, 100)!= HAL_OK)
			ret = 1;
		else
		{
			if(HAL_I2C_Master_Receive(&hi2c1, SlaveAddress, Buffer, 1, 100) != HAL_OK)
				ret = 1;
			else
			{
				ret = Buffer[0];
			}
		}
		return ret;
	}
	/*End*/

	/*Test Charger Status
	 @retval 0 - Charger OK and charging
	 	 	 1 - Charger OFF or Disconnected
	 	 	 2 - Charger Connected and enabled but not charging
	 	 	 3 - Charger done state*/
	uint8_t Test_Charger_Status(uint8_t *Buffer)
	{
		uint8_t status, ret;
		status = PMIC_Read_Reg(CHG_STAT_B, Buffer);
		if((status & 0xF0) == 0x80)
			ret = CHG_FULL;
		else if((status & 0xF0) != 0x00)
		{
			if((status & 0x0F) == 0x0E)
				ret  = CHG_OK; //charger debounced and charging OK
			else
				ret = CHG_ERR; //either charger in ULVO || OVP || i/p is being debounced || Charging Not-OK
		}
		else
			ret = CHG_DISCON; //Charger Disconnected or Charger OFF

		return ret;
	}
	/*End*/

	/*Test Whether charger is suspended or not
	  @retval - 0 - Suspended
	            1 - Not Suspended*/
	uint8_t Test_Suspend_Charger(uint8_t *Buffer)
	{
		//check USBS bit to suspend mode to disconnect charger
		uint8_t usbs, ret;
		usbs = PMIC_Read_Reg(CHG_G, Buffer);
		if(usbs == 0x6A)
			ret = 0;
		else
			ret = 1;
		return ret;
	}
	/*End*/

/*End of File*/

