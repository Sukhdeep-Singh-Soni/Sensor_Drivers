/*
 * at42qt2120.c
 *
 *  Created on: Oct 11, 2022
 *      Author: Sukhdeep Singh Soni
 *      @about: AT42QT2120 Touch sensor driver using Timers.
 *
 *      #################################################################
 *      Please include below instructions in main.c
 *     - touchSensor_Init()
 *     - HAL_TIM_Base_Start(&htim2); //1ms timer for delay
 *	   - HAL_TIM_Base_Start_IT(&htim5); //20ms timer for checking touch press and release and logic implementation.
 *	   ##################################################################
 */

#include "at42qt2120.h"
#include "tim.h"

uint8_t slider_previousVal=0, slider_previousVal1=0;
uint8_t tap_speed=0, tap_detect=0, touch_status;
uint8_t cid=0;
int slider_newVal=0, slider_detected=0, slider_ret_val=0, sl_prev_val=0,sl_new_val=0;
uint8_t glbl_int, glbl_reg;
uint8_t press=0, release=0;

/*
 * @brief: check chip id if touch sensor
 * @ret  : return cid if true and 0 otherwise
*/
uint8_t check_cid(void)
{
	uint8_t reg = 0x00;
	uint8_t ret=0;
	uint8_t data=0;
	while(HAL_I2C_GetState(I2C_HANDLE)!= HAL_I2C_STATE_READY);
	ret = HAL_I2C_Master_Transmit(I2C_HANDLE, DEV_ADDR, &reg, 1, HAL_MAX_DELAY);
	if(ret!=HAL_OK){

		Error_Handler();
	}
	while(HAL_I2C_GetState(I2C_HANDLE)!= HAL_I2C_STATE_READY);
	ret = HAL_I2C_Master_Receive(I2C_HANDLE, DEV_ADDR, &data, 1, HAL_MAX_DELAY);
	if(ret!=HAL_OK){

		Error_Handler();
	}

	if(data != CID)
		return 0;
	else
		return data;
}

/*
 * @brief: read 8-bit register value of touch sensor
 * @ret  : return register value if true or go to error handler otherwise
*/
uint8_t read8_i2c(uint8_t reg)
{
		uint8_t ret=0;
		uint8_t data=0;
		while(HAL_I2C_GetState(I2C_HANDLE)!= HAL_I2C_STATE_READY);
		ret = HAL_I2C_Master_Transmit(I2C_HANDLE, DEV_ADDR, &reg, 1, HAL_MAX_DELAY);
		if(ret!=HAL_OK){

			Error_Handler();
		}
		while(HAL_I2C_GetState(I2C_HANDLE)!= HAL_I2C_STATE_READY);
		ret = HAL_I2C_Master_Receive(I2C_HANDLE, DEV_ADDR, &data, 1, HAL_MAX_DELAY);
		if(ret!=HAL_OK){

			Error_Handler();
		}

		return data;
}

/*
 * @brief: read 16-bit register value of touch sensor
 * @arg1 : refister address
 * @ret  : return register value if true or go to error handler otherwise
*/
uint16_t read16_i2c(uint8_t reg)
{
		uint8_t ret=0;
		uint8_t val[2]={0};
		uint16_t data=0;
		while(HAL_I2C_GetState(I2C_HANDLE)!= HAL_I2C_STATE_READY);
		ret = HAL_I2C_Master_Transmit(I2C_HANDLE, DEV_ADDR, &reg, 1, HAL_MAX_DELAY);
		if(ret!=HAL_OK){

			Error_Handler();
		}
		while(HAL_I2C_GetState(I2C_HANDLE)!= HAL_I2C_STATE_READY);
		ret = HAL_I2C_Master_Receive(I2C_HANDLE, DEV_ADDR, val, 2, HAL_MAX_DELAY);
		if(ret!=HAL_OK){

			Error_Handler();
		}

		data = val[1]<<8;
		data = data | val[0];

		return data;
}

/*
 * @brief: write 8-bit register value to touch sensor
 * @arg1 : register address
 * @arg2 : value to write
 * @ret  : return 0 if write ok otherwise go to error handler
*/
uint8_t write8_i2c(uint8_t reg, uint8_t value)
{
		uint8_t ret=0;
		uint8_t val[2]={0};
		val[0]=reg;
		val[1]=value;
		while(HAL_I2C_GetState(I2C_HANDLE)!= HAL_I2C_STATE_READY);
		ret = HAL_I2C_Master_Transmit(I2C_HANDLE, DEV_ADDR, val, 2, HAL_MAX_DELAY);
		if(ret!=HAL_OK){

			Error_Handler();
		}

		return 0;
}

/*
 * @brief: enable slider
*/
void enable_slider(void)
{
	write8_i2c(SLIDER_OPTIONS_REG, 0x80);
}

/*
 * @brief: detect key
 * @ret  : returns 1 if detected and 0 otherwise
*/
uint8_t key_detect(void)
{
	uint8_t detectStatus = read8_i2c(DETECTION_STATUS_REG);
	if(is_bitSet(detectStatus,0))
		return 1;
	else
		return 0;
}

/*
 * @brief: check which key is pressed
 * @arg  : key no or index
 * @ret  : returns 1 if indexed key is pressed and 0 otherwise
*/
uint8_t is_keyPressed(uint8_t index)
{
	if(index > 11)
		return 0;
	uint8_t address = 3;
	if(index > 7)
	{
		address = 4;
		index -= 8;
	}
	uint8_t keyStatus = read8_i2c(address);
	if(is_bitSet(keyStatus, index))
		return 1;
	else
		return 0;
}

/*
 * @brief: helping function to check bit position of register
 * @arg1 : register value
 * @arg2 : position whose bit needs to be detected
 * @ret  : rerturns 1 if that bit is 1 otherwise returns 0
*/
uint8_t is_bitSet(uint8_t byte, uint8_t pos)
{
	if(byte & (1 << pos))
		return 1;
	else
		return 0;
}

/*
 * @brief: check if key pressed
 * @ret  : returns 1 if key pressed and 0 otherwise
*/
uint8_t key_val(uint8_t key_no)
{
	if(key_detect())
	{
		if(is_keyPressed(key_no))
			return 1;
		else
			return 0;
	}
	return 0;

}

/*
 * @brief: returns slider position
*/
uint8_t sliderPosition(void)
{
	uint8_t position = read8_i2c(SLIDER_POSITION_REG);
	return position;
}

/*
 * @brief: detect slider and returns 1 if detected and 0 otherwise
*/
uint8_t slider_detect(void)
{
	uint8_t detectStatus = read8_i2c(DETECTION_STATUS_REG);
		if(is_bitSet(detectStatus,1))
			return 1;
		else
			return 0;
}

/*
 * @brief: returns slider position if detected
*/
uint8_t slider_val(void)
{
	if(slider_detect())
	{
		return sliderPosition();
	}
	return 255;
}

/*
 * @brief: set detection integrator
*/
void set_detectIntegrator(uint8_t value)
{
	if(value <= 0) value = 1;
	if(value > 32) value = 32;

	write8_i2c(DI_REG, value);
}

/*
 * @brief: set key threshold
 * @arg1 : key index
 * @arg2 : threshold
*/
void set_keyThreshold(uint8_t index, uint8_t threshold)
{
	if(index > 11) return;
	uint8_t address = KEY0_DETECT_THRESHOLD_REG + index;
	write8_i2c(address, threshold);
}

/*
 * @brief: set time recalliberation delay
*/
void set_timeRecalDelay(uint8_t delay)
{
	write8_i2c(TRD_REG, delay);
}

/*
 * @brief: set towards touch detection
*/
void set_TTD(uint8_t value)
{
	write8_i2c(TTD_REG, value);
}

/*
 * @brief: set away from touch detection
*/
void set_ATD(uint8_t value)
{
	write8_i2c(ATD_REG, value);
}

/*
 * @brief: enable wheel
*/
void enable_wheel(void)
{
	write8_i2c(SLIDER_OPTIONS_REG, 0xC0);
}

/*
 * @brief: returns TTD value
*/
uint8_t get_TTD(void)
{
	uint8_t data = read8_i2c(TTD_REG);
	return data;
}

/*
 * @brief: returns ATD value
*/
uint8_t get_ATD(void)
{
	uint8_t data = read8_i2c(ATD_REG);
	return data;
}

/*
 * @brief: returns threshold value
*/
uint8_t get_ketThreshold(uint8_t index)
{
	if(index > 11) return 0;
	uint8_t address = KEY0_DETECT_THRESHOLD_REG + index;
	uint8_t data = read8_i2c(address);
	return data;
}

/*
 * @brief: returns DI value
*/
uint8_t get_detectIntegrator(void)
{
	uint8_t data = read8_i2c(DI_REG);
	return data;
}

/*
 * @brief: returns DHT value
*/
void set_DHT(uint8_t time)
{
	write8_i2c(DHT_REG, time);
}

/*
 * @brief: touch sensor initialization function
*/
void touchSensor_Init(void)
{
	  //check chip id
	  cid=check_cid();

	  //enable slider
	  enable_slider();

	  //set key 0,1,2 threshold values
	  set_keyThreshold(0, KEYS_THRESHOLD);
	  set_keyThreshold(1, KEYS_THRESHOLD);
	  set_keyThreshold(2, KEYS_THRESHOLD);

	  //set no. of samples before a valid key detect
	  set_detectIntegrator(3);

	  //set re-caliberation timer(30 * 0.16s)
	  set_timeRecalDelay(30);

	  //set drift hold time, touch towards drift and touch away from drift
	  set_DHT(25);
	  set_TTD(6);
	  set_ATD(5);
}

/*
 * @brief: touch sensor process function
*/
void touchSensor_Process(void)
{

	tap_detect = 0;
	slider_detected = 0;

	 /*--------------------------SLIDER DIRECTION DETECTION + TAP DETECTION + HOLD DETECTION------------------------------*/
		touch_status = mvp_touch_forward_backward_tap_detect();

		  printf("Slider: %d\r\n", sliderPosition());
		  slider_newVal = 0;
		  slider_previousVal = 0;
		  slider_previousVal1 = 0;
		  delay_us(100);
}

/*
 * @brief: function to detect key press
 * @arg1 : variable which stores press value
 * @note : not using
*/
void finger_press_detect(uint8_t* prev_value)
{
	 while(1)
	  {
		 *prev_value = sliderPosition();
		  delay_us(30);
		  if(*prev_value != sliderPosition())
			  break;
	  }
}

/*
 * @brief: function to detect key leave
 * @arg1 : variable which stores leave value
 * @note : not using
*/
void finger_leave_detect(uint8_t* prev_value)
{
	 while(1)
	  {
		 *prev_value = sliderPosition();
		  HAL_Delay(70);
		  if(*prev_value == sliderPosition())
		  {
			break;
		  }
	  }
}

/*
 * @brief: function to provide delay using timer 2 (1ms time period)
 * @arg1 : value in milliseconds to provide required delay
*/
void delay_us (uint16_t us)
	  {

	  	__HAL_TIM_SET_COUNTER(&htim2,0);  // set the counter value a 0
	  	while (__HAL_TIM_GET_COUNTER(&htim2) < us);  // wait for the counter to reach the us input in the parameter
	  }

/*
 * @brief: timer 5 callback function which has whole touch logic (20ms time period)
 * @arg1 : timer handle
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) //20ms timer 5(gives interrupt)
{

	if(htim->Instance == TIM5)
	{
		if(key_detect())
		{
			  press++;
			  if(press == 1)
			  {
			  slider_previousVal = sliderPosition();
			  }
			  release = 0;
		}
		else
		{
			release++;
			if(release == 1)
			{
				slider_previousVal1 = sliderPosition();
				touchSensor_Process();
				if(press > 50 && press < 250)
				{
					printf("\t\tHold Detected\r\n");
					touch_status = HOLD;
				}
			}
				press = 0;

		}

	}
}

/*
 * @brief: function with forward and backward slide logic with single tap detection logic
 * @ret  : returns forward/backward/tap detection status(check enum in at42qt2120.h)
*/
int8_t mvp_touch_forward_backward_tap_detect(void)
{
	 	  sl_prev_val = sl_new_val;
	      sl_new_val = slider_previousVal1;
	      //compare that position with previous value
	      if(sl_new_val != sl_prev_val)
	      {
	    	  slider_newVal = slider_previousVal1 - slider_previousVal;
	      }

		  //print direction of slide
		  if(slider_newVal > 0 && slider_newVal >= SLIDER_SENSITIVITY){
			  printf("Slider FORWARD Direction\r\n");
			  slider_detected = 1;
			  return SLIDE_FORWARD;
		  }
		  else if(slider_newVal < 0 && slider_newVal <= -SLIDER_SENSITIVITY){
			  printf("Slider BACKWARD Direction\r\n");
			  slider_detected = 1;
			  return SLIDE_BACKWARD;
		  }
		  else if(1)
		  {
			  if(slider_newVal < 0)
				  tap_speed = -slider_newVal;
			  else
				  tap_speed = slider_newVal;
			  if(tap_speed < TAP_SENSITIVITY && tap_speed > 0)
			  {
				  printf("\tTap Detected\r\n");
				  tap_detect = 1;
				  return TAP;
			  }
		  }

			  return NO_TOUCH;
}


//@brief: This function is used to print values on uart using printf
//@note : used with UART support only
//extern "C" {
int __io_putchar(uint8_t ch){

	HAL_UART_Transmit(&huart1, &ch, sizeof(ch), HAL_MAX_DELAY);

	return ch;
}

//}
