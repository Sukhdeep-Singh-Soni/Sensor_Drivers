/*******************************************************************************
  * file name:    orientation_bsp.c
  * author name:  Sukhdeep Singh Soni (sukhdeepsingh@focally.in)
  * module name:  icm-20948 head movement
  * file version: 1.0
  * about:        This file provides the api's specific to accelerometer for head movement application:
  *
  ******************************************************************************
**/

#include "icm20948.h"
#include "orientation_bsp.h"
#include<stdio.h>
#include "main.h"
#include<math.h>

axises acc_axes;

float angle = 0, pi = 3.1415;


void icm_hd_mvmt_init(void)
{
	  //check chip id
	  icm20948_who_am_i();

	  //initialize icm-20948
	  arg_mvp1_snsr_mtn_icm20948_init();

	  //message for initial orientation
	  printf("\r\nPlease put glasses in horizontal position to ground for proper functioning.");

	  //print degrees until the user button is pressed.
	  while(!HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin))
	  {
		    //read accelerometer g values.
		  	icm20948_accel_read_g(&acc_axes);

		  	//converting values to degrees.
		  	angle = atan2(acc_axes.x, acc_axes.z)*180/pi;

		    printf("\r\n degree - %.0f", angle);
		    HAL_Delay(100);
	  }
}

int icm_hd_mvmt_process(void)
{
	  //reading accelerometer raw values and converting it to g.
	  icm20948_accel_read_g(&acc_axes);

	  //conversion formula to convert accelerometer g values to degrees(taking x-axis as reference).
	  angle = atan2(acc_axes.x, acc_axes.z)*180/pi;


	  //logic to print angle between -90 to +90 range.
	  if(acc_axes.x >= 0 && acc_axes.z >= 0)
	  {
		  //going to right
		  return (int)90 - angle*MUL_FACTOR;

	  }
	  else if(acc_axes.x >= 0 && acc_axes.z < 0)
	  {
		  //going to left
		  return (int)90 - angle*MUL_FACTOR;
	  }

	  return 404;
}

int32_t int32abs(int32_t num)
{
	return num >= 0 ? num : -num;
}

int head_mvmt_get(void)
{
	 // logic for head movement and it returns angle
	 return icm_hd_mvmt_process();

}

//end of file
