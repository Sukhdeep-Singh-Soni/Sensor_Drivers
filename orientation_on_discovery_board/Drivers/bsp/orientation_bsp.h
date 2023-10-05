/*******************************************************************************
  * file name:    orientation_bsp.h
  * author name:  Sukhdeep Singh Soni (sukhdeepsingh@focally.in)
  * module name:  icm-20948 head movement
  * file version: 1.0
  * about:        This file provides the headers specific to accelerometer for head movement application:
  *
  ******************************************************************************
**/

#ifndef BSP_ORIENTATION_BSP_H_
#define BSP_ORIENTATION_BSP_H_

#define MUL_FACTOR	1


void icm_hd_mvmt_init(void);
int icm_hd_mvmt_process(void);
int32_t int32abs(int32_t num);
int head_mvmt_get(void);

#endif /* BSP_ORIENTATION_BSP_H_ */
