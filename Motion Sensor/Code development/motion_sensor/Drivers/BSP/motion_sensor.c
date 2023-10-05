/******************************************************************************
  * file name:    motion_sensor.c
  * author name:  Sukhdeep Singh Soni (sukhdeepsingh@focally.in)
  * module name:  Motion Sensor(LSM6DSOX & LIS2MDL)
  * file version: 1.1
  * about:        This file provides the api's specific to motion sensor:
  * 			  + spi/i2c initialization functions
  *  			  + SPI read register
  *           	  + SPI write register
  *           	  + Local delay
  *           	  + Return status
  *           	  + i2c read/write api's
  *           	  + accelerometer/gyroscope/magnetometer power mode set api's
  *           	  + accelerometer/gyroscope/magnetometer power mode check api's
  *           	  + motion sensor bsp init and process fucntions
  *           	  + accelerometer/gyrometer/magnetometer init api's
  *           	  + accelerometer/gyrometer/magnetometer deinit api's
  *           	  + callback function for interrupt handling
  ******************************************************************************
**/

#ifndef BSP_MOTION_SENSOR_C_
#define BSP_MOTION_SENSOR_C_


#include "motion_sensor.h"

uint8_t DevAddr = 0xD7;
LSM6DSOX_Object_t *pObj, obj;
LSM6DSOX_Axes_t xl_axes, g_axes;
LIS2MDL_Object_t *mObj, mag;
LIS2MDL_Axes_t mag_axes;
stmdev_ctx_t ctx;
static uint8_t lsm6dsox_ID;
static uint8_t lis2mdl_ID;
uint16_t counter=0;
int32_t  threshold=0;
uint8_t  var, flag=0, acc_gyro_rst_val = 0x03;
#if INT_FLAG
uint8_t int_flag = 0;
#endif
uint8_t ctrl_4 = 0x20, pw_dn = 0x00, dben_set = 0x00;

/*@breif - initialize the structure "LSM6DSOX_Object_t" for LSM6DSOX(ACC & GYRO)
 *         with custom read/write register api's.
 *@param - void.
 *@ret   - void.*/
void spi_i2c_acc_gyro_setup(void)
{
	obj.IO.BusType = 1;
	obj.IO.Address = 0xD7;
#if (I2C_SPI_EN == 0)
	obj.Ctx.read_reg = spi_read_reg_mtn_snsr;
	obj.Ctx.write_reg = spi_write_reg_mtn_snsr;
	obj.Ctx.handle = &hspi1;
#elif (I2C_SPI_EN == 1)
	obj.Ctx.read_reg = i2c_read_8bit;
	obj.Ctx.write_reg = i2c_write_8bit;
	obj.Ctx.handle = &hi2c1;
#endif
	obj.IO.GetTick = mtn_snsr_delay;
	pObj = &obj;
}

/*@breif - initialize the structure "LIS2MDL_Object_t" for LIS2MDL(Magnetometer)
 *         with custom read/write api's.
 *@param - void.
 *@ret   - void.*/
void spi_i2c_mag_setup(void)
{
	mag.IO.BusType   = LSM6DSOX_SENSORHUB_LIS2MDL_I2C_BUS;
	mag.IO.Address   = LIS2MDL_I2C_ADD;
#if (I2C_SPI_EN == 0)
	mag.Ctx.read_reg = spi_read_reg_mtn_snsr;
	mag.Ctx.write_reg = spi_write_reg_mtn_snsr;
	mag.Ctx.handle    = &hspi1;
#elif (I2C_SPI_EN == 1)
	mag.Ctx.read_reg = i2c_read_8bit;
	mag.Ctx.write_reg = i2c_write_8bit;
	mag.Ctx.handle    = &hi2c1;
#endif
	mObj = &mag;
}

int32_t mtn_read_sensorhub_reg(LIS2MDL_Object_t *pObj, uint8_t Reg, uint8_t *pData, uint16_t Length)
{
	  uint8_t lsm6dsox_func_cfg_access = 0x01U;
	  uint8_t shub_reg_access_en = 0x40U;
	  uint8_t shub_reg_access_dis = 0x00U;
	  uint8_t ext_sens_addr_read = LIS2MDL_I2C_ADD | 0x01U;
	  uint8_t slv0_add = 0x15U;
	  uint8_t slv0_subadd = 0x16U;
	  uint8_t slave0_config = 0x17U;
	  uint8_t master_config = 0x14U;
	  uint8_t write_once_i2c_en = 0x44U;
	  uint8_t sensor_hub_1 = 0x02U;
	  uint8_t status_master_mainpage = 0x39U;
	  uint8_t sens_hub_endop = 0x01U;
	  uint8_t lsm6dsox_outx_h_a = 0x29U;
	  uint8_t lsm6dsox_status_reg = 0x1EU;
	  uint8_t xlda = 0x01U;
	  uint8_t len = (uint8_t)Length;
	  uint8_t lsm6dsox_ctrl1_xl = 0x10U;
	  uint8_t lsm6dsox_xl_prev_odr;
	  uint8_t lsm6dsox_xl_odr_off = 0x00U;
	  uint8_t lsm6dsox_xl_odr_104hz = 0x40U;
	  uint8_t data;
	  uint8_t data_array[6];

	  /* Enable access to sensor hub registers */
	  if (lis2mdl_write_reg(&(pObj->Ctx), lsm6dsox_func_cfg_access, &shub_reg_access_en, 1) != LIS2MDL_OK)
	  {
	    return LIS2MDL_ERROR;
	  }

	  /* Configure external device address, Enable read operation (rw_0 = 1) */
	  if (lis2mdl_write_reg(&(pObj->Ctx), slv0_add, &ext_sens_addr_read, 1) != LIS2MDL_OK)
	  {
	    return LIS2MDL_ERROR;
	  }

	  /* Configure address of the LIS2MDL register to be read */
	  if (lis2mdl_write_reg(&(pObj->Ctx), slv0_subadd, &Reg, 1) != LIS2MDL_OK)
	  {
	    return LIS2MDL_ERROR;
	  }

	  /* Read required number of bytes (up to 6), SHUB_ODR = 104 Hz */
	  if (lis2mdl_write_reg(&(pObj->Ctx), slave0_config, &len, 1) != LIS2MDL_OK)
	  {
	    return LIS2MDL_ERROR;
	  }

	  /* WRITE_ONCE is mandatory for read, I2C master enabled using slave 0, I2C pull-ups disabled */
	  if (lis2mdl_write_reg(&(pObj->Ctx), master_config, &write_once_i2c_en, 1) != LIS2MDL_OK)
	  {
	    return LIS2MDL_ERROR;
	  }

	  /* Disable access to sensor hub registers */
	  if (lis2mdl_write_reg(&(pObj->Ctx), lsm6dsox_func_cfg_access, &shub_reg_access_dis, 1) != LIS2MDL_OK)
	  {
	    return LIS2MDL_ERROR;
	  }

	  /* Read LSM6DSOX ODR */
	  if (lis2mdl_read_reg(&(pObj->Ctx), lsm6dsox_ctrl1_xl, &lsm6dsox_xl_prev_odr, 1) != LIS2MDL_OK)
	  {
	    return LIS2MDL_ERROR;
	  }

	  /* Set XL_ODR_104HZ if the accelerometer is disabled */
	  if (lsm6dsox_xl_prev_odr == lsm6dsox_xl_odr_off)
	  {
	    lsm6dsox_xl_prev_odr = lsm6dsox_xl_odr_104hz;
	  }

	  /* Enable accelerometer to trigger Sensor Hub operation */
	  if (lis2mdl_write_reg(&(pObj->Ctx), lsm6dsox_ctrl1_xl, &lsm6dsox_xl_prev_odr, 1) != LIS2MDL_OK)
	  {
	    return LIS2MDL_ERROR;
	  }

	  /* Read ACC data starting from LSM6DSOX OUTX_H_A register to clear accelerometer data-ready XLDA */
	  if (lis2mdl_read_reg(&(pObj->Ctx), lsm6dsox_outx_h_a, data_array, 6) != LIS2MDL_OK)
	  {
	    return LIS2MDL_ERROR;
	  }

	  /* Poll LSM6DSOX LSM6DSOX_STATUS_REG until XLDA = 1 (Wait for sensor hub trigger) */
	  do
	  {
	    if (lis2mdl_read_reg(&(pObj->Ctx), lsm6dsox_status_reg, &data, 1) != LIS2MDL_OK)
	    {
	      return LIS2MDL_ERROR;
	    }
	  } while ((data & xlda) != xlda);

	  /* Poll LSM6DSOX SensorHub SENS_HUB_ENDOP bit in STATUS_MASTER_MAINPAGE reg until the end of SW write operations */
	  do
	  {
	    if (lis2mdl_read_reg(&(pObj->Ctx), status_master_mainpage, &data, 1) != LIS2MDL_OK)
	    {
	      return LIS2MDL_ERROR;
	    }
	  } while ((data & sens_hub_endop) != sens_hub_endop);

	  /* Enable access to sensor hub registers */
	  if (lis2mdl_write_reg(&(pObj->Ctx), lsm6dsox_func_cfg_access, &shub_reg_access_en, 1) != LIS2MDL_OK)
	  {
	    return LIS2MDL_ERROR;
	  }

	  /* Read data from LSM6DSOX SensorHub regs containing values from required LIS2MDL regs */
	  if (lis2mdl_read_reg(&(pObj->Ctx), sensor_hub_1, pData, Length) != LIS2MDL_OK)
	  {
	    return LIS2MDL_ERROR;
	  }

	  /* Disable access to sensor hub registers */
	  if (lis2mdl_write_reg(&(pObj->Ctx), lsm6dsox_func_cfg_access, &shub_reg_access_dis, 1) != LIS2MDL_OK)
	  {
	    return LIS2MDL_ERROR;
	  }

	  return LIS2MDL_OK;
}

int32_t mtn_write_sensorhub_reg(LIS2MDL_Object_t *pObj, uint8_t Reg, uint8_t *pData, uint16_t Length)
{
	  uint8_t lsm6dsox_func_cfg_access = 0x01U;
	  uint8_t shub_reg_access_en = 0x40U;
	  uint8_t shub_reg_access_dis = 0x00U;
	  uint8_t ext_sens_addr_write = LIS2MDL_I2C_ADD & 0xFEU;
	  uint8_t slv0_add = 0x15U;
	  uint8_t slv0_subadd = 0x16U;
	  uint8_t slave0_config = 0x17U;
	  uint8_t shub_odr_104 = 0x00U;
	  uint8_t master_config = 0x14U;
	  uint8_t write_once_i2c_en = 0x44U;
	  uint8_t status_master_mainpage = 0x39U;
	  uint8_t wr_once_done = 0x80U;
	  uint8_t lsm6dsox_outx_h_a = 0x29U;
	  uint8_t lsm6dsox_status_reg = 0x1EU;
	  uint8_t xlda = 0x01U;
	  uint8_t lsm6dsox_ctrl1_xl = 0x10U;
	  uint8_t lsm6dsox_xl_prev_odr;
	  uint8_t lsm6dsox_xl_odr_off = 0x00U;
	  uint8_t lsm6dsox_xl_odr_104hz = 0x40U;
	  //uint8_t datawrite_slv0 = 0x0EU;
	  uint8_t datawrite_slv0 = 0x21;
	  uint8_t data;
	  uint8_t data_array[6];

	  /* Enable access to sensor hub registers */
	  if (lis2mdl_write_reg(&(pObj->Ctx), lsm6dsox_func_cfg_access, &shub_reg_access_en, 1) != LIS2MDL_OK)
	  {
	    return LIS2MDL_ERROR;
	  }

	  /* Configure external device address, Enable write operation (rw_0 = 0) */
	  if (lis2mdl_write_reg(&(pObj->Ctx), slv0_add, &ext_sens_addr_write, 1) != LIS2MDL_OK)
	  {
	    return LIS2MDL_ERROR;
	  }

	  /* Configure address of the LIS2MDL register to be written to */
	  if (lis2mdl_write_reg(&(pObj->Ctx), slv0_subadd, &Reg, 1) != LIS2MDL_OK)
	  {
	    return LIS2MDL_ERROR;
	  }

	  /* Set SHUB_ODR = 104 Hz */
	  if (lis2mdl_write_reg(&(pObj->Ctx), slave0_config, &shub_odr_104, 1) != LIS2MDL_OK)
	  {
	    return LIS2MDL_ERROR;
	  }

	  /* Store data to be written to LIS2MDL in LSM6DSOX SH reg */
	  if (lis2mdl_write_reg(&(pObj->Ctx), datawrite_slv0, pData, 1) != LIS2MDL_OK)
	  {
	    return LIS2MDL_ERROR;
	  }

	  /* WRITE_ONCE enabled for single write, I2C master enabled using slave 0, I2C pull-ups disabled */
	  if (lis2mdl_write_reg(&(pObj->Ctx), master_config, &write_once_i2c_en, 1) != LIS2MDL_OK)
	  {
	    return LIS2MDL_ERROR;
	  }

	  /* Disable access to sensor hub registers */
	  if (lis2mdl_write_reg(&(pObj->Ctx), lsm6dsox_func_cfg_access, &shub_reg_access_dis, 1) != LIS2MDL_OK)
	  {
	    return LIS2MDL_ERROR;
	  }

	  /* Read LSM6DSOX ODR */
	  if (lis2mdl_read_reg(&(pObj->Ctx), lsm6dsox_ctrl1_xl, &lsm6dsox_xl_prev_odr, 1) != LIS2MDL_OK)
	  {
	    return LIS2MDL_ERROR;
	  }

	  /* Set XL_ODR_104HZ if the accelerometer is disabled */
	  if (lsm6dsox_xl_prev_odr == lsm6dsox_xl_odr_off)
	  {
	    lsm6dsox_xl_prev_odr = lsm6dsox_xl_odr_104hz;
	  }

	  /* Enable accelerometer to trigger Sensor Hub operation */
	  if (lis2mdl_write_reg(&(pObj->Ctx), lsm6dsox_ctrl1_xl, &lsm6dsox_xl_prev_odr, 1) != LIS2MDL_OK)
	  {
	    return LIS2MDL_ERROR;
	  }

	  /* Read ACC data starting from LSM6DSOX OUTX_H_A register to clear accelerometer data-ready XLDA */
	  if (lis2mdl_read_reg(&(pObj->Ctx), lsm6dsox_outx_h_a, data_array, 6) != LIS2MDL_OK)
	  {
	    return LIS2MDL_ERROR;
	  }

	  /* Poll LSM6DSOX LSM6DSOX_STATUS_REG until XLDA = 1 (Wait for sensor hub trigger) */
	  do
	  {
	    if (lis2mdl_read_reg(&(pObj->Ctx), lsm6dsox_status_reg, &data, 1) != LIS2MDL_OK)
	    {
	      return LIS2MDL_ERROR;
	    }
	  } while ((data & xlda) != xlda);

	  /* Poll LSM6DSOX SensorHub WR_ONCE_DONE bit in STATUS_MASTER_MAINPAGE reg until the end of SW write operations */
	  for(int i=0; i< 50000; i++);
	  do
	  {
	    if (lis2mdl_read_reg(&(pObj->Ctx), status_master_mainpage, &data, 1) != LIS2MDL_OK)
	    {
	      return LIS2MDL_ERROR;
	    }
	  } while ((data & wr_once_done) != wr_once_done);

	  return LIS2MDL_OK;
}

uint8_t mtn_acc_deinit(LSM6DSOX_Object_t *pObj)
{
	/* Disable the component */
	  if (LSM6DSOX_ACC_Disable(pObj) != LSM6DSOX_OK)
	  {
	    return LSM6DSOX_ERROR;
	  }
	  /* Reset output data rate. */
	  pObj->acc_odr = LSM6DSOX_XL_ODR_OFF;
	  return LSM6DSOX_OK;
}


uint8_t mtn_gyro_deinit(LSM6DSOX_Object_t *pObj)
{
	//disable the component
	if (LSM6DSOX_GYRO_Disable(pObj) != LSM6DSOX_OK)
		  {
		    return LSM6DSOX_ERROR;
		  }

	  //Reset output data rate
	  pObj->gyro_odr = LSM6DSOX_GY_ODR_OFF;
	  return LSM6DSOX_OK;

}
uint8_t mtn_acc_init(LSM6DSOX_Object_t *pObj)
{
	/* Disable I3C */
	  if (lsm6dsox_i3c_disable_set(&(pObj->Ctx), LSM6DSOX_I3C_DISABLE) != LSM6DSOX_OK)
	  {
	    return LSM6DSOX_ERROR;
	  }

	  /* Enable register address automatically incremented during a multiple byte
	  access with a serial interface. */
	  if (lsm6dsox_auto_increment_set(&(pObj->Ctx), PROPERTY_ENABLE) != LSM6DSOX_OK)
	  {
	    return LSM6DSOX_ERROR;
	  }

	  /* Enable BDU */
	  if (lsm6dsox_block_data_update_set(&(pObj->Ctx), PROPERTY_ENABLE) != LSM6DSOX_OK)
	  {
	    return LSM6DSOX_ERROR;
	  }

	  /* FIFO mode selection */
	  if (lsm6dsox_fifo_mode_set(&(pObj->Ctx), LSM6DSOX_BYPASS_MODE) != LSM6DSOX_OK)
	  {
	    return LSM6DSOX_ERROR;
	  }

	  /* Select default output data rate. */
	    pObj->acc_odr = LSM6DSOX_XL_ODR_104Hz;

	    /* Output data rate selection - power down. */
	    if (lsm6dsox_xl_data_rate_set(&(pObj->Ctx), LSM6DSOX_XL_ODR_OFF) != LSM6DSOX_OK)
	    {
	      return LSM6DSOX_ERROR;
	    }

	    /* Full scale selection. */
	    if (lsm6dsox_xl_full_scale_set(&(pObj->Ctx), LSM6DSOX_2g) != LSM6DSOX_OK)
	    {
	      return LSM6DSOX_ERROR;
	    }

	     return LSM6DSOX_OK;
}

uint8_t mtn_gyro_init(LSM6DSOX_Object_t *pObj)
{
	/* Disable I3C */
	  if (lsm6dsox_i3c_disable_set(&(pObj->Ctx), LSM6DSOX_I3C_DISABLE) != LSM6DSOX_OK)
	  {
	    return LSM6DSOX_ERROR;
	  }

	  /* Enable register address automatically incremented during a multiple byte
	  access with a serial interface. */
	  if (lsm6dsox_auto_increment_set(&(pObj->Ctx), PROPERTY_ENABLE) != LSM6DSOX_OK)
	  {
	    return LSM6DSOX_ERROR;
	  }

	  /* Enable BDU */
	  if (lsm6dsox_block_data_update_set(&(pObj->Ctx), PROPERTY_ENABLE) != LSM6DSOX_OK)
	  {
	    return LSM6DSOX_ERROR;
	  }

	  /* FIFO mode selection */
	  if (lsm6dsox_fifo_mode_set(&(pObj->Ctx), LSM6DSOX_BYPASS_MODE) != LSM6DSOX_OK)
	  {
	    return LSM6DSOX_ERROR;
	  }

	  /* Select default output data rate. */
	   pObj->gyro_odr = LSM6DSOX_GY_ODR_104Hz;

	   /* Output data rate selection - power down. */
	   if (lsm6dsox_gy_data_rate_set(&(pObj->Ctx), LSM6DSOX_GY_ODR_OFF) != LSM6DSOX_OK)
	   {
	     return LSM6DSOX_ERROR;
	   }

	   /* Full scale selection. */
	   if (lsm6dsox_gy_full_scale_set(&(pObj->Ctx), LSM6DSOX_2000dps) != LSM6DSOX_OK)
	   {
	     return LSM6DSOX_ERROR;
	   }

	    return LSM6DSOX_OK;

}

/*@brief - read register value, print it and return that value.
 *@param - register address from which to read.
 *@param - variable to save the read value.
 *@ret   - value of the register.*/
uint8_t reg_ret_status(uint8_t regAddr, uint8_t *buff)
{
#if (I2C_SPI_EN == 0)
	spi_read_reg_mtn_snsr(&hspi1, regAddr, buff, 1);
#elif (I2C_SPI_EN == 1)
	i2c_read_8bit(&hi2c1, regAddr, buff, 1);
#endif
	printf("\r\n0x%X",*buff);
	return *buff;
}


#if (I2C_SPI_EN == 0)
/*@breif - static api for SPI read register.
 *@param - SPI handler.
 *@param - register address to read from.
 *@param - data to read into.
 *@param - number of bytes to read.
 *@ret   - SPI read error for spi transmission error or HAL_OK otherwise.*/
int32_t spi_read_reg_mtn_snsr(void *handle, uint8_t Reg, uint8_t *pdata, uint16_t Length)
{
	uint8_t ret;
	Reg = Reg|0x80;
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_RESET);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY){}
	ret=HAL_SPI_Transmit(&hspi1, &Reg, 1, HAL_MAX_DELAY);
	if(ret != HAL_OK)
		printf("\r\nSPI Read Error");
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY){}
	ret=HAL_SPI_Receive(&hspi1, pdata, Length, HAL_MAX_DELAY);
	if(ret != HAL_OK)
		printf("\r\nSPI Read Error");
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY){}
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_SET);
	return HAL_OK;
}

/*@breif - static api for SPI write register.
 *@param - SPI handler.
 *@param - register address to write into.
 *@param - data to write into the register.
 *@param - number of bytes to write.
 *@ret   - SPI write error for spi transmission error or HAL_OK otherwise.*/
int32_t spi_write_reg_mtn_snsr(void *handle, uint8_t Reg, uint8_t *pdata, uint16_t Length)
{
	uint8_t ret;
	Reg = Reg & 0x7F;
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_RESET);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY){}
	ret=HAL_SPI_Transmit(&hspi1, &Reg, 1, HAL_MAX_DELAY);
	if(ret != HAL_OK)
  		printf("\r\nSPI Write Error");
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY){}
	ret=HAL_SPI_Transmit(&hspi1, pdata, Length, HAL_MAX_DELAY);
	if(ret != HAL_OK)
  		printf("\r\nSPI Write Error");
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY){}
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_SET);
	return HAL_OK;
}
#endif

/*@breif - provide tick value.
 *@param - void.
 *@ret   - tick value in 32-bit format.*/
int32_t mtn_snsr_delay(void)
{
	return HAL_GetTick();
}

void mtn_acc_gyro_init(void)
{
#if (ACC_EN || GYRO_EN)

	LSM6DSOX_ReadID(pObj,&lsm6dsox_ID); //read chip ID of lsm6ds0x(0x6C).
	printf("\r\n1. 0x%X",lsm6dsox_ID);
	if(lsm6dsox_ID != 0x6C)
	{
	  printf("\r\nreset function called");

	  //software reset LSM6DSOX
	  LSM6DSOX_Write_Reg(pObj, acc_gyro_rst_reg, acc_gyro_rst_val);
	  HAL_Delay(500);

	  //read chip id of lsm6dsox(0x6C)
	  LSM6DSOX_ReadID(pObj,&lsm6dsox_ID);
	  printf("\r\n2. 0x%X",lsm6dsox_ID);
	}
	if(lsm6dsox_ID == 0x6C)
	{
#if ACC_EN
	  mtn_acc_init(pObj);
	  LSM6DSOX_ACC_Enable(pObj); //Enable Accelerometer
	  LSM6DSOX_ACC_Enable_Wake_Up_Detection(pObj, LSM6DSOX_INT1_PIN); //Enable wake up detection on int1 pin
	  LSM6DSOX_ACC_Set_Wake_Up_Threshold(pObj, wk_ths); //set wake up threshold to 0x06;

	  //enable pedometer for step detection.
	  LSM6DSOX_ACC_Enable_Pedometer(pObj);

	  //reset step counter of pedometer.
	  LSM6DSOX_ACC_Step_Counter_Reset(pObj);

	  //set pedometer debounce value bo 0x00.
	  lsm6dsox_pedo_debounce_steps_set(&(pObj->Ctx), &dben_set);

	  //Enable Inactivity detection.
	  LSM6DSOX_ACC_Enable_Inactivity_Detection(pObj, LSM6DSOX_XL_12Hz5_GY_PD, LSM6DSOX_INT1_PIN);
#else
	  mtn_acc_deinit(pObj);
#endif
#if GYRO_EN
	  mtn_gyro_init(pObj);
	  LSM6DSOX_GYRO_Enable(pObj); //Enable Gyrometer.
#else
	  mtn_gyro_deinit(pObj);
#endif
#if (ACC_EN && GYRO_EN)
	  pObj->is_initialized = 1;
#elif (!(ACC_EN || GYRO_EN))
	  pObj->is_initialized = 0;
#endif
	 }
#endif
}

void mtn_acc_mode_set(motion_sensor_pow_modes mode)
{
	int32_t ret;
	switch(mode)
	{
	case POWER_DOWN_MODE:
	{
#if (I2C_SPI_EN == 0)
		spi_write_reg_mtn_snsr(&hspi1, mtn_ctrl1_xl, &pw_dn, 1);
#elif (I2C_SPI_EN == 1)
		i2c_write_8bit(&hi2c1, mtn_ctrl1_xl, &pw_dn, 1);
#endif
		break;
	}
	case ULTRA_LOW_POWER_MODE:
	{
		  lsm6dsox_ctrl5_c_t val1;
		  lsm6dsox_ctrl6_c_t val2;
#if (I2C_SPI_EN == 0)
		  ret = spi_read_reg_mtn_snsr(&hspi1, mtn_ctrl6_c, (uint8_t *)&val2, 1);
#elif (I2C_SPI_EN == 1)
		  ret = i2c_read_8bit(&hi2c1, mtn_ctrl6_c, (uint8_t *)&val2, 1);
#endif
		  if(ret == 0)
		  {
			  if (val2.xl_hm_mode)
			  {
				  val2.xl_hm_mode = 0;
#if (I2C_SPI_EN == 0)
				  spi_write_reg_mtn_snsr(&hspi1, mtn_ctrl6_c, (uint8_t *)&val2, 1);
#elif (I2C_SPI_EN == 1)
				  i2c_write_8bit(&hi2c1, mtn_ctrl6_c, (uint8_t *)&val2, 1);
#endif
			  }
		  }

		  /* Disable Gyro */
		  if (pObj->gyro_is_enabled == 1U)
		  {
			LSM6DSOX_GYRO_Disable(pObj);
		  }
#if (I2C_SPI_EN == 0)
		  ret = spi_read_reg_mtn_snsr(&hspi1, mtn_ctrl5_c, (uint8_t *)&val1, 1);
#elif (I2C_SPI_EN == 1)
		  ret = i2c_read_8bit(&hi2c1, mtn_ctrl5_c, (uint8_t *)&val1, 1);
#endif
		  if(ret == 0)
		  {
			  if (!val1.xl_ulp_en)
			  {
				  /* Power off the accelerometer */
				  if (pObj->acc_is_enabled == 1U)
				  {
					 LSM6DSOX_ACC_Disable(pObj);
				  }

				  val1.xl_ulp_en = 1U;
#if (I2C_SPI_EN == 0)
				  spi_write_reg_mtn_snsr(&hspi1, mtn_ctrl5_c, (uint8_t *)&val1, 1);
#elif (I2C_SPI_EN == 1)
				  i2c_write_8bit(&hi2c1, mtn_ctrl5_c, (uint8_t *)&val1, 1);
#endif
			  }
		  }
		  lsm6dsox_xl_data_rate_set(&(pObj->Ctx), LSM6DSOX_XL_ODR_1Hz6);
		  break;
	}
	case LOW_POWER_MODE:
	{
		  /* We must uncheck Ultra Low Power bit if it is enabled */
		  /* and check the Low Power bit if it is unchecked       */
		  lsm6dsox_ctrl5_c_t val1;
		  lsm6dsox_ctrl6_c_t val2;
#if (I2C_SPI_EN == 0)
		  ret = spi_read_reg_mtn_snsr(&hspi1, mtn_ctrl5_c, (uint8_t *)&val1, 1);
#elif (I2C_SPI_EN == 1)
		  ret = i2c_read_8bit(&hi2c1, mtn_ctrl5_c, (uint8_t *)&val1, 1);
#endif
		  if(ret == 0)
		  {
			  if (val1.xl_ulp_en)
			  {
				  /* Power off the accelerometer */
				  LSM6DSOX_ACC_Disable(pObj);
			  }

			val1.xl_ulp_en = 0;
#if (I2C_SPI_EN == 0)
			spi_write_reg_mtn_snsr(&hspi1, mtn_ctrl5_c, (uint8_t *)&val1, 1);
#elif (I2C_SPI_EN == 1)
			i2c_write_8bit(&hi2c1, mtn_ctrl5_c, (uint8_t *)&val1, 1);
#endif
		  }
#if (I2C_SPI_EN == 0)
		  ret = spi_read_reg_mtn_snsr(&hspi1, mtn_ctrl6_c, (uint8_t *)&val2, 1);
#elif (I2C_SPI_EN == 1)
		  ret = i2c_read_8bit(&hi2c1, mtn_ctrl6_c, (uint8_t *)&val2, 1);
#endif
		  if(ret == 0)
		  {
			  if (!val2.xl_hm_mode)
			  {
				  val2.xl_hm_mode = 1U;
#if (I2C_SPI_EN == 0)
				  spi_write_reg_mtn_snsr(&hspi1, mtn_ctrl6_c, (uint8_t *)&val2, 1);
#elif (I2C_SPI_EN == 1)
				  i2c_write_8bit(&hi2c1, mtn_ctrl6_c, (uint8_t *)&val2, 1);
#endif
			  }
		  }
		  lsm6dsox_xl_data_rate_set(&(pObj->Ctx), LSM6DSOX_XL_ODR_12Hz5);
		  break;
	}
	case HIGH_PERFORMANCE_MODE:
	{
	  /* We must uncheck Low Power and Ultra Low Power bits if they are enabled */
	  lsm6dsox_ctrl5_c_t val1;
	  lsm6dsox_ctrl6_c_t val2;
#if (I2C_SPI_EN == 0)
	  ret = spi_read_reg_mtn_snsr(&hspi1, mtn_ctrl5_c, (uint8_t *)&val1, 1);
#elif (I2C_SPI_EN == 1)
	  ret = i2c_read_8bit(&hi2c1, mtn_ctrl5_c, (uint8_t *)&val1, 1);
#endif
	  if(ret == 0)
	  {
		  if (val1.xl_ulp_en)
		  {
			  /* Power off the accelerometer */
			  if (pObj->acc_is_enabled == 1U)
			  {
				  LSM6DSOX_ACC_Disable(pObj);
			  }

			  val1.xl_ulp_en = 0;
#if (I2C_SPI_EN == 0)
			  spi_write_reg_mtn_snsr(&hspi1, mtn_ctrl5_c, (uint8_t *)&val1, 1);
#elif (I2C_SPI_EN == 1)
			  i2c_write_8bit(&hi2c1, mtn_ctrl5_c, (uint8_t *)&val1, 1);
#endif
		  }
	  }
#if (I2C_SPI_EN == 0)
	  ret = spi_read_reg_mtn_snsr(&hspi1, mtn_ctrl6_c, (uint8_t *)&val2, 1);
#elif (I2C_SPI_EN == 1)
	  ret = i2c_read_8bit(&hi2c1, mtn_ctrl6_c, (uint8_t *)&val2, 1);
#endif
	  if(ret == 0)
	  {
		  if (val2.xl_hm_mode)
		  {
			  val2.xl_hm_mode = 0;
#if (I2C_SPI_EN == 0)
			  spi_write_reg_mtn_snsr(&hspi1, mtn_ctrl6_c, (uint8_t *)&val2, 1);
#elif (I2C_SPI_EN == 1)
			  i2c_write_8bit(&hi2c1, mtn_ctrl6_c, (uint8_t *)&val2, 1);
#endif
		  }
		  lsm6dsox_xl_data_rate_set(&(pObj->Ctx), LSM6DSOX_XL_ODR_417Hz);
	  }
	  break;
	}

	case NORMAL_MODE:
	{
		  /* We must uncheck Ultra Low Power bit if it is enabled */
				  /* and check the Low Power bit if it is unchecked       */
				  lsm6dsox_ctrl5_c_t val1;
				  lsm6dsox_ctrl6_c_t val2;
#if (I2C_SPI_EN == 0)
				  ret = spi_read_reg_mtn_snsr(&hspi1, mtn_ctrl5_c, (uint8_t *)&val1, 1);
#elif (I2C_SPI_EN == 1)
				  ret = i2c_read_8bit(&hi2c1, mtn_ctrl5_c, (uint8_t *)&val1, 1);
#endif
				  if(ret == 0)
				  {
					  if (val1.xl_ulp_en)
					  {
						  /* Power off the accelerometer */
						  LSM6DSOX_ACC_Disable(pObj);
					  }

					val1.xl_ulp_en = 0;
#if (I2C_SPI_EN == 0)
					spi_write_reg_mtn_snsr(&hspi1, mtn_ctrl5_c, (uint8_t *)&val1, 1);
#elif (I2C_SPI_EN == 1)
					i2c_write_8bit(&hi2c1, mtn_ctrl5_c, (uint8_t *)&val1, 1);
#endif
				  }
#if (I2C_SPI_EN == 0)
				  ret = spi_read_reg_mtn_snsr(&hspi1, mtn_ctrl6_c, (uint8_t *)&val2, 1);
#elif (I2C_SPI_EN == 1)
				  ret = i2c_read_8bit(&hi2c1, mtn_ctrl6_c, (uint8_t *)&val2, 1);
#endif
				  if(ret == 0)
				  {
					  if (!val2.xl_hm_mode)
					  {
						  val2.xl_hm_mode = 1U;
#if (I2C_SPI_EN == 0)
						  spi_write_reg_mtn_snsr(&hspi1, mtn_ctrl6_c, (uint8_t *)&val2, 1);
#elif (I2C_SPI_EN == 1)
						  i2c_write_8bit(&hi2c1, mtn_ctrl6_c, (uint8_t *)&val2, 1);
#endif
					  }
				  }
				  lsm6dsox_xl_data_rate_set(&(pObj->Ctx), LSM6DSOX_XL_ODR_104Hz);
				  break;
	}
  }
}

void mtn_gyro_mode_set(motion_sensor_pow_modes mode)
{
	lsm6dsox_ctrl7_g_t val;
	int32_t ret;
	switch(mode)
	{
	case POWER_DOWN_MODE:
	{
#if (I2C_SPI_EN == 0)
		spi_write_reg_mtn_snsr(&hspi1, mtn_ctrl2_g, &pw_dn, 1);
#elif (I2C_SPI_EN ==1)
		i2c_write_8bit(&hi2c1,  mtn_ctrl2_g, &pw_dn, 1);
#endif
		break;
	}

	case LOW_POWER_MODE:
	{
#if (I2C_SPI_EN == 0)
		ret = spi_read_reg_mtn_snsr(&hspi1, mtn_ctrl7_g, (uint8_t *)&val, 1);
#elif (I2C_SPI_EN == 1)
		ret = i2c_read_8bit(&hi2c1, mtn_ctrl7_g, (uint8_t *)&val, 1);
#endif

		if(ret == 0)
		{
			val.g_hm_mode = 1U;
#if (I2C_SPI_EN == 0)
			spi_write_reg_mtn_snsr(&hspi1, mtn_ctrl7_g, (uint8_t *)&val, 1);
#elif (I2C_SPI_EN == 1)
			i2c_write_8bit(&hi2c1, mtn_ctrl7_g, (uint8_t *)&val, 1);
#endif
		}
		lsm6dsox_gy_data_rate_set(&(pObj->Ctx), LSM6DSOX_GY_ODR_12Hz5);
		break;
	}
	case HIGH_PERFORMANCE_MODE:
	{
#if (I2C_SPI_EN == 0)
		ret = spi_read_reg_mtn_snsr(&hspi1, mtn_ctrl7_g, (uint8_t *)&val, 1);
#elif (I2C_SPI_EN == 1)
		ret = i2c_read_8bit(&hi2c1, mtn_ctrl7_g, (uint8_t *)&val, 1);
#endif
		if(ret == 0)
		{
			val.g_hm_mode = 0U;
#if (I2C_SPI_EN == 0)
			spi_write_reg_mtn_snsr(&hspi1, mtn_ctrl7_g, (uint8_t *)&val, 1);
#elif (I2C_SPI_EN == 1)
			i2c_write_8bit(&hi2c1, mtn_ctrl7_g, (uint8_t *)&val, 1);
#endif
		}
		lsm6dsox_gy_data_rate_set(&(pObj->Ctx), LSM6DSOX_GY_ODR_417Hz);
		break;
	}

	case NORMAL_MODE:
	{
#if (I2C_SPI_EN == 0)
				ret = spi_read_reg_mtn_snsr(&hspi1, mtn_ctrl7_g, (uint8_t *)&val, 1);
#elif (I2C_SPI_EN == 1)
				ret = i2c_read_8bit(&hi2c1, mtn_ctrl7_g, (uint8_t *)&val, 1);
#endif
				if(ret == 0)
				{
					val.g_hm_mode = 1U;
#if (I2C_SPI_EN == 0)
					spi_write_reg_mtn_snsr(&hspi1, mtn_ctrl7_g, (uint8_t *)&val, 1);
#elif (I2C_SPI_EN == 1)
					i2c_write_8bit(&hi2c1, mtn_ctrl7_g, (uint8_t *)&val, 1);
#endif
				}
				lsm6dsox_gy_data_rate_set(&(pObj->Ctx), LSM6DSOX_GY_ODR_104Hz);
				break;
	}
  }
}

void mtn_mag_mode_set(motion_sensor_pow_modes mode)
{
	switch(mode)
	{
	case POWER_DOWN_MODE:
	{
		LIS2MDL_MAG_Disable(mObj);
		break;
	}
	case LOW_POWER_MODE:
	{
		lis2mdl_cfg_reg_a_t reg;
		int32_t ret;
		uint8_t lval = 1;

		  ret = mtn_read_sensorhub_reg(mObj, cfg_reg_a, (uint8_t *)&reg, 1);
		  if (ret == 0)
		  {
		    reg.lp = (uint8_t)lval;
		    ret = mtn_write_sensorhub_reg(mObj, cfg_reg_a, (uint8_t *)&reg, 1);
		  }
		  ret = mtn_read_sensorhub_reg(mObj, cfg_reg_a, (uint8_t *)&reg, 1);
		  if(ret == 0)
		  {
			  reg.odr = LIS2MDL_ODR_10Hz;
			  mtn_write_sensorhub_reg(mObj, cfg_reg_a, (uint8_t *)&reg, 1);
		  }
		break;
	}
	case HIGH_PERFORMANCE_MODE:
	{
		lis2mdl_cfg_reg_a_t reg;
		int32_t ret;
		uint8_t hval = 0;

		  ret = mtn_read_sensorhub_reg(mObj, cfg_reg_a, (uint8_t *)&reg, 1);
		  if (ret == 0)
		  {
			reg.lp = (uint8_t)hval;
			ret = mtn_write_sensorhub_reg(mObj, cfg_reg_a, (uint8_t *)&reg, 1);
		  }
		  ret = mtn_read_sensorhub_reg(mObj, cfg_reg_a, (uint8_t *)&reg, 1);
		  if(ret == 0)
		  {
			reg.odr = LIS2MDL_ODR_50Hz;
			mtn_write_sensorhub_reg(mObj, cfg_reg_a, (uint8_t *)&reg, 1);
		  }
		break;
	}
  }
}

void check_mtn_acc_mode(void)
{
	uint8_t buff=0, ret=0, ret1=0;

	printf("\r\n Acc:");

	//checking ODR
	reg_ret_status(0x10, &buff);
	if((buff & 0xF0) == 0x00)
		printf("\r\nPower down");

	ret = buff;
	//checking high performance or low power/normal mode
	reg_ret_status(0x15, &buff);
	if((buff & 0x10) == 0x10 && ret == 0x40)
		printf("\r\nNormal mode");
	else if((buff & 0x10) == 0x10 && ret == 0x10)
		printf("\r\nLow power mode");

	ret1 = buff;
	//checking ultra low power mode
	reg_ret_status(0x14, &buff);
	if((buff & 0x80) == 0x80 && (ret1 & 0x10) == 0x00)
		printf("\r\nUltra low power mode");
	else if((buff & 0x80) == 0x00 && (ret1 & 0x10) == 0x00 && (ret & 0xF0) == 0x60)
		printf("\r\nHigh performance mode");

}

void check_mtn_gyro_mode(void)
{
	uint8_t buff=0, ret=0;

	printf("\r\n Gyro:");

	//checking ODR
	reg_ret_status(0x11, &buff);
	if((buff & 0xF0) == 0x00)
		printf("\r\nPower down mode");

	ret = buff;
	//checking high performance or low power/normal mode
	reg_ret_status(0x16, &buff);
	if(((buff & 0x80) == 0x80) && ((ret & 0xF0) == 0x10))
		printf("\r\nLow power mode");
	else if(((buff & 0x80) == 0x80) && ((ret & 0xF0) == 0x40))
		printf("\r\nNormal mode");
	else if((buff & 0x80) == 0x00 && (ret & 0xF0) == 0x60)
		printf("\r\nHigh performance mode");
}

void check_mag_mode(void)
{
	lis2mdl_cfg_reg_a_t reg;

	printf("\r\n Mag:");
	LIS2MDL_Read_Reg(mObj, cfg_reg_a, (uint8_t *)&reg);
	printf("\r\n0x%X,",reg);

	//checking low power or high performance mode and power down mode
	if(reg.lp == 1)
		printf("\r\nLow power mode");
	else if(reg.lp == 0)
		printf("\r\nHigh performance mode");
	else if(reg.md == 2 || reg.md == 3)
		printf("power down mode");
}

void mtn_mag_init(void)
{
#if MAG_EN

		uint8_t buff;

	    //read chip ID of magnetometer(0x40).
		LIS2MDL_ReadID(mObj, &lis2mdl_ID);
		printf("\r\n1. 0x%X",lis2mdl_ID);
		if(lis2mdl_ID != 0x40)
		{
			printf("\r\nreset function call");
#if (I2C_SPI_EN == 0)
			spi_write_reg_mtn_snsr(&hspi1, acc_gyro_rst_reg, &acc_gyro_rst_val, 1);
#elif (I2C_SPI_EN == 1)
			//reset lsm6dsox
			i2c_write_8bit(&hi2c1, acc_gyro_rst_reg, &acc_gyro_rst_val, 1);
#endif
			//reset magnetometer(lis2mdl)
			LIS2MDL_Write_Reg(mObj, mag_rst_reg, mag_rst_val);

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

			//setting threshold lower byte (0xe8)
			LIS2MDL_Write_Reg(mObj, LIS2MDL_INT_THS_L_REG, int_ths_l_reg);
			LIS2MDL_Read_Reg(mObj,LIS2MDL_INT_THS_L_REG, &buff);
			printf("\r\n0x%X", int_ths_l_reg);

			//setting threshold higher byte (0x03)
			LIS2MDL_Write_Reg(mObj, LIS2MDL_INT_THS_H_REG, int_ths_h_reg);
			LIS2MDL_Read_Reg(mObj, LIS2MDL_INT_THS_H_REG, &buff);
			printf("\r\n0x%X", int_ths_h_reg);

			//putting full 16 bit value in threshold variable
			threshold = (int_ths_h_reg << 8) | int_ths_l_reg;

			 //enable interrupt on int/drdy pin of magnetometer
			LIS2MDL_Write_Reg(mObj, LIS2MDL_CFG_REG_C, cfg_reg_c);
			LIS2MDL_Read_Reg(mObj, LIS2MDL_CFG_REG_C, &buff);
			printf("\r\n0x%X", cfg_reg_c);

			 //configuring interrupt control register with pulsed, INT = 0(give interrupt when magnet is near)
			LIS2MDL_Write_Reg(mObj, LIS2MDL_INT_CRTL_REG, int_ctrl_reg);
			LIS2MDL_Read_Reg(mObj, LIS2MDL_INT_CRTL_REG, &buff);
			printf("\r\n0x%X", int_ctrl_reg);

			//configure all interrupts on int1 pin of lsm6dsox
#if (I2C_SPI_EN == 0)
			spi_write_reg_mtn_snsr(&hspi1, mtn_ctrl4_c, &ctrl_4, 1);
			spi_read_reg_mtn_snsr(&hspi1, mtn_ctrl4_c, &buff, 1);
#elif (I2C_SPI_EN == 1)
			i2c_write_8bit(&hi2c1, mtn_ctrl4_c, &ctrl_4, 1);
			i2c_read_8bit(&hi2c1, mtn_ctrl4_c, &buff, 1);
#endif
			printf("\r\n0x%X", ctrl_4);
		}
#else
			LIS2MDL_DeInit(mObj);
#endif
	}

void mtn_bsp_init(void)
{
	// spi initialization setup
	spi_i2c_acc_gyro_setup(); //acc & gyro setup
	spi_i2c_mag_setup(); // mag setup

	//initialize accelerometer and gyrometer.
	mtn_acc_gyro_init();

	//initialize magnetometer.
	mtn_mag_init();

	//enable accelerometer/gyrometer interrupt
	HAL_NVIC_EnableIRQ(EXTI7_IRQn);

	//enable magnetometer interrupt
	HAL_NVIC_EnableIRQ(EXTI15_IRQn);
}

void mtn_acc_gyro_process(void)
{
#if ACC_EN
	  //get accelerometer axes values.
	  LSM6DSOX_ACC_GetAxes(pObj,&xl_axes);
	  printf("\n\n");

	  //print accelerometer axes values.
	  printf("Acc x : %ld\r\n",xl_axes.x);
	  printf("Acc y : %ld\r\n",xl_axes.y);
	  printf("Acc z : %ld\r\n",xl_axes.z);
#endif
#if GYRO_EN
	  //get gyrometer axes values.
	  LSM6DSOX_GYRO_GetAxes(pObj,&g_axes);
	  printf("\n\n");

      //print gyrometer axes values.
 	  printf("Gyro x : %ld\r\n",g_axes.x);
	  printf("Gyro y : %ld\r\n",g_axes.y);
	  printf("Gyro z : %ld\r\n",g_axes.z);
#endif
}

void mtn_mag_process(void)
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

void mtn_mag_int_gen(void)
{
	 uint8_t buff;

		  //read magnetometer axes
		  LIS2MDL_MAG_GetAxes(mObj, &mag_axes);

		  //read magnetometer interrupt control register
		  LIS2MDL_Read_Reg(mObj, LIS2MDL_INT_CRTL_REG, &buff);
		  //printf("\r\n0x%X", buff);

	#if (LATCH_PULSE_EN == 1)
		  if(buff != 0xE3 || (mag_axes.x < threshold && mag_axes.y < threshold && mag_axes.z < threshold))
		  {
		  //read magnetometer interrupt source register to clear latching
		  LIS2MDL_Read_Reg(mObj, LIS2MDL_INT_SOURCE_REG, &buff);
		  //printf("\r\n0x%X", buff);
		  }
	#endif
		  //enable magnetometer interrupt
		  HAL_NVIC_EnableIRQ(EXTI7_IRQn);
}

void mtn_bsp_process(void)
{
	//magnetometer interrupt on near magnet and far magnet
	mtn_mag_int_gen();

#if INT_FLAG
	if(int_flag != 1)
	{
#endif

	//accelerometer/gyrometer process function
	mtn_acc_gyro_process();

	//magnetometer process function
	mtn_mag_process();

#if INT_FLAG
	}
#endif

	HAL_Delay(500);
}


#if (I2C_SPI_EN == 1)
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
#endif

/*@breif - INT1 interrupt callback function set priority, differentiates between different
 *         interrupts and print which interrupt occured.
 *param  - gpio interrupt pin.
 *ret    - void.*/
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
#if INT1
	if(GPIO_Pin == INT1_Pin)
	{
	uint8_t buff, buff1, buff2;
#if ACC_EN
	//Reading from 0x1A(ALL_INT_SRC) register
#if (LATCH_PULSE_EN == 0)
	uint8_t Reg;
	Reg = ALL_INT_SRC | 0x80;
	spi_read_reg_mtn_snsr(&hspi1, Reg, &buff, 1);
#elif (LATCH_PULSE_EN == 1)
	i2c_read_8bit(&hi2c1, ALL_INT_SRC, &buff, 1);
#endif

	//reading from 0x1B(WAKE_UP_SRC) register

#if (LATCH_PULSE_EN == 0)
	Reg = WAKE_UP_SRC | 0x80;
	spi_read_reg_mtn_snsr(&hspi1, Reg, &buff1, 1);
#elif (LATCH_PULSE_EN == 1)
	i2c_read_8bit(&hi2c1, WAKE_UP_SRC, &buff1, 1);
#endif

    //Reading from 0x35 register(EMB_FUNC_STATUS_MAINPAGE)

#if (LATCH_PULSE_EN == 0)
	Reg = EMB_FUNC_STATUS_MAINPAGE | 0x80;
	spi_read_reg_mtn_snsr(&hspi1, Reg, &buff2, 1);
#elif (LATCH_PULSE_EN == 1)
	i2c_read_8bit(&hi2c1, EMB_FUNC_STATUS_MAINPAGE, &buff2, 1);
#endif

	if((buff & 0x02) == 0x02 || (buff1 & 0x08) == 0x08) //check for wake up interrupt.
	{
		if(flag == 0)
		{
			flag = 1;
			printf("\r\nWake Up event\r\n");
			mtn_acc_mode_set(NORMAL_MODE);
			check_mtn_acc_mode();
#if GYRO_EN
			mtn_gyro_mode_set(NORMAL_MODE);
			check_mtn_gyro_mode();
#endif
#if MAG_EN
			mtn_mag_mode_set(HIGH_PERFORMANCE_MODE);
			check_mag_mode();
#endif
		}
	}
	else if((buff2 & 0x08) == 0x08) //check for step detection interrupt.
	{
		printf("\r\nStep Detected\r\n");
		printf("Counter ->%d\r\n",counter++);
	}
	else if((buff1 & 0x10) == 0x10) //check for inactivity interrupt.
	{
		flag = 0;
		printf("\r\nactivity/Inactivity event\r\n");
			mtn_acc_mode_set(LOW_POWER_MODE);
			check_mtn_acc_mode();
#if GYRO_EN
			mtn_gyro_mode_set(POWER_DOWN_MODE);
			check_mtn_gyro_mode();
#endif
#if MAG_EN
			mtn_mag_mode_set(LOW_POWER_MODE);
			check_mag_mode();
#endif
		}
	}
#endif // end of ACC_EN
#endif //end of INT1

#if INT2
	else if(GPIO_Pin == INT2_Pin)
	{
		//turning led on
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

		printf("\r\n********************Int2 interrupt occured from MAG********************");
		LIS2MDL_MAG_GetAxes(mObj, &mag_axes);
		if((mag_axes.x >= threshold || mag_axes.y >= threshold || mag_axes.z >= threshold))
		{
			HAL_NVIC_DisableIRQ(EXTI7_IRQn);
			printf("\r\nPower OFF");
#if INT_FLAG
			int_flag = 1;
#endif
			 //interrupt pulsed with INT = 0(give interrupt when magnet is away from sensor)
			LIS2MDL_Write_Reg(mObj, LIS2MDL_INT_CRTL_REG, int_ctrl_reg_far);
		}
		else if((mag_axes.x < threshold && mag_axes.y < threshold && mag_axes.z < threshold))
		{
			HAL_NVIC_DisableIRQ(EXTI7_IRQn);
			printf("\r\nPower ON");
#if INT_FLAG
			int_flag = 0;
#endif
			//interrupt pulsed when INT = 1(give interrupt when magnet is near sensor)
			LIS2MDL_Write_Reg(mObj, LIS2MDL_INT_CRTL_REG, int_ctrl_reg_near);
		}
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


#endif /* BSP_MOTION_SENSOR_C_ */
