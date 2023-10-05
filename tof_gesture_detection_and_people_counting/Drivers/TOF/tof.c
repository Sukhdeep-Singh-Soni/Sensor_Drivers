/*
 * tof.c
 *
 *  Created on: 02-Dec-2022
 *      Author: sukhdeep
 */

#ifndef TOF_C
#define TOF_C

#include "tof.h"
#include <stdio.h>
#include "usart.h"

volatile uint8_t zone_3x3_first[9] = {0}, zone_3x3_second[9] = {0}, int_flag = 0;

uint8_t tof_init(void)
{
	 if(startup() != TOF_OK)
		 return TOF_ERROR;

	 if(image_download() != TOF_OK)
		 return TOF_ERROR;

	 if(application_configuration() != TOF_OK)
		 return TOF_ERROR;
//	 if(check_mode() == TOF_ERROR)
//			 return TOF_ERROR;

//	 tof_i2c_read(&read_val, TOF_MINOR, 1);

//	 if(application_configuration() != TOF_OK)
//		 return TOF_ERROR;

	 //test api
//	 if(switch_bw_tmf8828_and_tmf8821_mode(TOF_MODE_TMF8821) != TOF_OK)
//		 return TOF_ERROR;
//
//	 tof_i2c_read(&read_val, TOF_PREV_CMD, 1);
//
//	 if(application_configuration() != TOF_OK)
//	 		 return TOF_ERROR;
//
//	 if(switch_bw_tmf8828_and_tmf8821_mode(TOF_MODE_TMF8828) != TOF_OK)
//			 return TOF_ERROR;
//
//	 tof_i2c_read(&read_val, TOF_PREV_CMD, 1);
//
//
//	 if(check_mode() == TOF_MODE_TMF8828)
//		 return TOF_ERROR;


	 //end of test api


	 return TOF_OK;
}

uint8_t tof_process(void)
{
	if(measurements_results() != TOF_OK)
		return TOF_ERROR;

	return TOF_OK;
}

uint8_t Gesture_Detection(uint8_t *read_buffer)
{
	//Basic gesture algorithm for hand movement (left-to-right or right-to left)
	uint8_t zone_3x3[9] = {0};
	zone_3x3[0] = *(read_buffer+24);  //			3x3 map will be like this
	zone_3x3[1] = *(read_buffer+27);  //
	zone_3x3[2] = *(read_buffer+30);  //			zone3	zone2	zone1
	zone_3x3[3] = *(read_buffer+33);  //			zone6	zone5	zone4
	zone_3x3[4] = *(read_buffer+36);  //			zone9	zone8	zonr7
	zone_3x3[5] = *(read_buffer+39);  //
	zone_3x3[6] = *(read_buffer+42);  //
	zone_3x3[7] = *(read_buffer+45);  //
	zone_3x3[8] = *(read_buffer+48);  //

	if(zone_3x3[0] == 0xff && zone_3x3[3] == 0xff && zone_3x3[6] == 0xff)
	{
		HAL_Delay(100);
		measurements_results();
		zone_3x3[0] = *(read_buffer+24);  //			3x3 map will be like this
		zone_3x3[1] = *(read_buffer+27);  //
		zone_3x3[2] = *(read_buffer+30);  //			zone3	zone2	zone1
		zone_3x3[3] = *(read_buffer+33);  //			zone6	zone5	zone4
		zone_3x3[4] = *(read_buffer+36);  //			zone9	zone8	zonr7
		zone_3x3[5] = *(read_buffer+39);  //
		zone_3x3[6] = *(read_buffer+42);  //
		zone_3x3[7] = *(read_buffer+45);  //
		zone_3x3[8] = *(read_buffer+48);  //
		if(zone_3x3[2] == 0xff && zone_3x3[5] == 0xff && zone_3x3[8] == 0xff)
		{
			//right to left
			counter = 3;
			while(counter-- > 0);
		}
	}else if(zone_3x3[2] == 0xff && zone_3x3[5] == 0xff && zone_3x3[8] == 0xff)
	{
		HAL_Delay(100);
		measurements_results();
		zone_3x3[0] = *(read_buffer+24);  //			3x3 map will be like this
		zone_3x3[1] = *(read_buffer+27);  //
		zone_3x3[2] = *(read_buffer+30);  //			zone3	zone2	zone1
		zone_3x3[3] = *(read_buffer+33);  //			zone6	zone5	zone4
		zone_3x3[4] = *(read_buffer+36);  //			zone9	zone8	zonr7
		zone_3x3[5] = *(read_buffer+39);  //
		zone_3x3[6] = *(read_buffer+42);  //
		zone_3x3[7] = *(read_buffer+45);  //
		zone_3x3[8] = *(read_buffer+48);  //
		if(zone_3x3[0] == 0xff && zone_3x3[3] == 0xff && zone_3x3[6] == 0xff)
		{
			//left to right
			counter = 3;
			while(counter-- > 0);
		}
	}

	return TOF_ERROR;
}

void Gesture_Detection_IT(void)
{
	if(counter <= 2)
	{
	//Basic gesture algorithm for hand movement (left-to-right or right-to left)

		if(counter ==1)
		{
			zone_3x3_first[0] = result_buff[24];  //			3x3 map will be like this
			zone_3x3_first[1] = result_buff[27];  //
			zone_3x3_first[2] = result_buff[30];  //			zone3	zone2	zone1
			zone_3x3_first[3] = result_buff[33];  //			zone6	zone5	zone4
			zone_3x3_first[4] = result_buff[36];  //			zone9	zone8	zonr7
			zone_3x3_first[5] = result_buff[39];  //
			zone_3x3_first[6] = result_buff[42];  //
			zone_3x3_first[7] = result_buff[45];  //
			zone_3x3_first[8] = result_buff[48];  //

			if(zone_3x3_first[0] == 0xff && zone_3x3_first[1] == 0xff && zone_3x3_first[2] == 0xff && zone_3x3_first[3] == 0xff && zone_3x3_first[4] == 0xff && zone_3x3_first[5] == 0xff && zone_3x3_first[6] == 0xff && zone_3x3_first[7] == 0xff && zone_3x3_first[8] == 0xff)
			{
				counter = 0;
			}
		}else if(counter ==2)
		{
			if(/*zone_3x3_first[2] == 0xff && */zone_3x3_first[5] == 0xff && zone_3x3_first[8] == 0xff)
			{
														//			3x3 map will be like this

														//			zone3	zone2	zone1
				zone_3x3_second[3] = result_buff[33];   //			zone6	zone5	zone4
														//			zone9	zone8	zone7
				zone_3x3_second[6] = result_buff[42];  //


//			if(zone_3x3_first[2] == 0xff && zone_3x3_first[5] == 0xff && zone_3x3_first[8] == 0xff)
//			{
				if(/*zone_3x3_second[0] == 0xff && */zone_3x3_second[3] == 0xff && zone_3x3_second[6] == 0xff)
				{
					//left to right
					int_flag = 0x02;
					printf("LEFT-TO-RIGHT %u\r\n", counter);
				}
			}else if(/*zone_3x3_first[0] == 0xff && */zone_3x3_first[3] == 0xff && zone_3x3_first[6] == 0xff)
			{

				zone_3x3_second[5] = result_buff[39];  //
				zone_3x3_second[8] = result_buff[48];  //

				if(/*zone_3x3_second[2] == 0xff && */zone_3x3_second[5] == 0xff && zone_3x3_second[8] == 0xff)
				{
					//right to left
					int_flag = 0x01;
					printf("RIGHT-TO-LEFT %u\r\n", counter);
				}
			}
			counter = 0;
		}

	}else
	{
		counter = 0;
	}
}

/**
  * @brief Print the characters to UART (printf).
  * @retval int
  */
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
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);

  return ch;
}

#endif // TOF_C



















