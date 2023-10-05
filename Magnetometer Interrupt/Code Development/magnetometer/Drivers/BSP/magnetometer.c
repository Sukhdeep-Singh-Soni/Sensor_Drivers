/******************************************************************************
  * file name:    magnetometer.c
  * author name:  Sukhdeep Singh Soni (sukhdeepsingh@focally.in)
  * module name:  Motion Sensor(LSM6DSOX & LIS2MDL)
  * file version: 1.1
  * about:        This file provides the api's specific to magnetometer(lis2mdl):
  * 			  + i2c initialization functions
  *  			  + i2c read register
  *           	  + i2c write register
  *           	  + magnetometer bsp init and process fucntions
  *           	  + magnetometer init api
  *           	  + callback function for interrupt handling
  ******************************************************************************
**/

#include "magnetometer.h"

uint8_t DevAddr = 0x3D;
static uint8_t int_status = 0;
LIS2MDL_Object_t *mObj, mag;
LIS2MDL_Axes_t mag_axes;
static uint8_t lis2mdl_ID;
int32_t  threshold=0;
#if INT_FLAG
uint8_t int_flag = 0;
#endif

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

void i2c_mag_setup(void)
{
	mag.IO.BusType   = LIS2MDL_I2C_BUS;
	mag.IO.Address   = LIS2MDL_I2C_ADD;
	mag.Ctx.read_reg = i2c_read_8bit;
	mag.Ctx.write_reg = i2c_write_8bit;
	mag.Ctx.handle    = &hi2c1;
	mObj = &mag;
}

void mag_reset(void)
{
	uint8_t soft_rst = 0x23;// mem_reboot = 0x43;
	//LIS2MDL_Read_Reg(mObj, LIS2MDL_CFG_REG_A, &mem_reboot);
	LIS2MDL_Read_Reg(mObj, LIS2MDL_CFG_REG_A, &soft_rst);
}

void mag_init(void)
{
#if MAG_EN
		uint8_t buff;

		//reset magnetometer
		mag_reset();

	    //read chip ID of magnetometer(0x40).
		LIS2MDL_ReadID(mObj, &lis2mdl_ID);
		printf("\r\n1. 0x%X",lis2mdl_ID);
		if(lis2mdl_ID != 0x40)
		{
			printf("\r\nreset function call");
			//reset magnetometer(lis2mdl)
			mag_reset();

			//read chip ID of magnetometer(0x40).
			LIS2MDL_ReadID(mObj, &lis2mdl_ID);
			HAL_Delay(500);
			printf("\r\n2. 0x%X",lis2mdl_ID);
		}
		if(lis2mdl_ID == 0x40)
		{
			//Initialize magnetometer.
			LIS2MDL_Init(mObj);

			//Enable magnetometer.
			LIS2MDL_MAG_Enable(mObj);

			//enabling low pass filter
			LIS2MDL_Read_Reg(mObj, LIS2MDL_CFG_REG_B, &buff);
			buff = buff | 0x01;
			LIS2MDL_Write_Reg(mObj, LIS2MDL_CFG_REG_B, buff);
			LIS2MDL_Read_Reg(mObj, LIS2MDL_CFG_REG_B, &buff);
			printf("\r\nLPF register = 0x%X", buff);


			//set magnetometer in low power mode
			LIS2MDL_MAG_Set_Power_Mode(mObj, LOW_POWER_MODE);
			LIS2MDL_Read_Reg(mObj, LIS2MDL_CFG_REG_A, &buff);
			printf("\r\nLPM register and ODR = 0x%X", buff);

			//setting threshold lower byte (can set in magnetometer.h file)
			LIS2MDL_Write_Reg(mObj, LIS2MDL_INT_THS_L_REG, int_ths_l_reg);
			LIS2MDL_Read_Reg(mObj,LIS2MDL_INT_THS_L_REG, &buff);
			printf("\r\n0x%X", buff);

			//setting threshold higher byte (can set in magnetometer .h file)
			LIS2MDL_Write_Reg(mObj, LIS2MDL_INT_THS_H_REG, int_ths_h_reg);
			LIS2MDL_Read_Reg(mObj, LIS2MDL_INT_THS_H_REG, &buff);
			printf("\r\n0x%X", buff);

			//putting full 16 bit value in threshold variable
			threshold = (int_ths_h_reg << 8) | int_ths_l_reg;

			//enable interrupt on int/drdy pin of magnetometer
			LIS2MDL_Write_Reg(mObj, LIS2MDL_CFG_REG_C, cfg_reg_c);
			LIS2MDL_Read_Reg(mObj, LIS2MDL_CFG_REG_C, &buff);
			printf("\r\n0x%X", buff);

			//configuring interrupt control register with pulsed, INT = 1(give interrupt when magnet is near)
			LIS2MDL_Write_Reg(mObj, LIS2MDL_INT_CRTL_REG, int_ctrl_reg);
			LIS2MDL_Read_Reg(mObj, LIS2MDL_INT_CRTL_REG, &buff);
			printf("\r\n0x%X", buff);
		}
#else
			LIS2MDL_DeInit(mObj);
			if(mObj->mag_is_enabled == 1)
				LIS2MDL_MAG_Disable(mObj);
#endif
	}

void bsp_init(void)
{
	// i2c initialization setup
	i2c_mag_setup(); // mag setup

	//initialize magnetometer.
	mag_init();

	//enable accelerometer/gyrometer interrupt
	HAL_NVIC_EnableIRQ(EXTI7_IRQn);
}

void bsp_process(void)
{
#if MAG_EN
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

	//magnetometer interrupt generation on near magnet and far magnet
	mag_int_gen();

#if INT_FLAG
	if(int_flag != 1)
	{
#endif
	//magnetometer process function
	mag_process();

#if INT_FLAG
	}
#endif

	HAL_Delay(500);
#endif
}

void mag_process(void)
{
	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
#if MAG_EN

	  //get magnetometer axes values.
	  LIS2MDL_MAG_GetAxes(mObj, &mag_axes);
	  printf("\n\n");

	  //print magnetometer axes values.
	  printf("Mag x : %ld\r\n",mag_axes.x);
	  printf("Mag y : %ld\r\n",mag_axes.y);
	  printf("Mag z : %ld\r\n",mag_axes.z);
#endif
	    printf("\r\n");
}

void mag_int_gen(void)
{
	uint8_t buff;

	if(int_status == 1)
	  {

		//turning led on
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);


		printf("\r\n********************Int2 interrupt occured from MAG********************");
		LIS2MDL_Read_Reg(mObj, LIS2MDL_INT_CRTL_REG, &buff);
		//printf("\r\n0x%X", buff);

		//get magnetometer axes
		LIS2MDL_MAG_GetAxes(mObj, &mag_axes);

		//comparing magnetometer axes with threshold
		if(buff != 0xe1 && (mag_axes.x >= threshold || mag_axes.y >= threshold || mag_axes.z >= threshold))
		{
			printf("\r\nPower OFF");

  #if INT_FLAG
			int_flag = 1;
  #endif
			 //interrupt pulsed with INT = 0(give interrupt when magnet is away from sensor)
			LIS2MDL_Write_Reg(mObj, LIS2MDL_INT_CRTL_REG, int_ctrl_reg_far);
		}
		else if(buff != 0xe5 && (mag_axes.x < threshold && mag_axes.y < threshold && mag_axes.z < threshold))
		{
			printf("\r\nPower ON");
  #if INT_FLAG
			int_flag = 0;
  #endif
			//interrupt pulsed when INT = 1(give interrupt when magnet is near sensor)
			LIS2MDL_Write_Reg(mObj, LIS2MDL_INT_CRTL_REG, int_ctrl_reg_near);
		}
		int_status = 0;
	  }

#if LATCH_PULSE_EN
		  if(buff != 0xe3 || (mag_axes.x < threshold && mag_axes.y < threshold && mag_axes.z < threshold))
		  {
			  //read magnetometer interrupt source register to clear latching
			  printf("\r\nreading int_source_reg to clear latch");
			  LIS2MDL_Read_Reg(mObj, LIS2MDL_INT_SOURCE_REG, &buff);
			  //printf("\r\n0x%X", buff);
		  }
		  else if(buff == 0xe3 || (mag_axes.x >= threshold && mag_axes.y >= threshold && mag_axes.z >= threshold))
		  {
			  printf("\r\nnon interupt latched");
			  LIS2MDL_Read_Reg(mObj, LIS2MDL_INT_SOURCE_REG, &buff);
			  LIS2MDL_Read_Reg(mObj, LIS2MDL_INT_CRTL_REG, &buff);
			  if(buff == 0xe1 || buff == 0xe5)
			  {
				  LIS2MDL_Write_Reg(mObj, LIS2MDL_INT_CRTL_REG, buff & 0xFE);
			  }
		  }
#endif
}

int32_t i2c_read_8bit(void *handle, uint8_t Reg, uint8_t *pdata, uint16_t Length)
{
	uint8_t err_code;
	err_code = HAL_I2C_Master_Transmit(&hi2c1, DevAddr, &Reg, 1, HAL_MAX_DELAY);
	if(err_code != HAL_OK)
		return HAL_ERROR;
	err_code = HAL_I2C_Master_Receive(&hi2c1, DevAddr, pdata, Length, HAL_MAX_DELAY);
	if(err_code != HAL_OK)
		return HAL_ERROR;
	return HAL_OK;
}
int32_t i2c_write_8bit(void *handle, uint8_t Reg, uint8_t *pdata, uint16_t Length)
{
	uint8_t err_code, buff[2];
	buff[0] = Reg;
	buff[1] = *pdata;
	err_code = HAL_I2C_Master_Transmit(&hi2c1, DevAddr, buff, Length+1, HAL_MAX_DELAY);
	if(err_code != HAL_OK)
		return HAL_ERROR;
	return HAL_OK;
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
#if MAG_EN
	if(GPIO_Pin == INT2_Pin)
	{
		uint8_t buff;
		int_status = 1;

		//turn magnetometer interrupts off
		LIS2MDL_Read_Reg(mObj, LIS2MDL_INT_CRTL_REG, &buff);
		LIS2MDL_Write_Reg(mObj, LIS2MDL_INT_CRTL_REG, buff & 0xFE);
	}
#endif
}

#ifdef __GNUC__
  /* With GCC, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
int __io_putchar(int ch)
#else
int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the UART3 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);

  return ch;
}

//end of file
