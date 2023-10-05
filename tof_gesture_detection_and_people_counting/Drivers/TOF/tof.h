/*
 * tof.h
 *
 *  Created on: 02-Dec-2022
 *      Author: sukhdeep
 */

#ifndef TOF_TOF_H_
#define TOF_TOF_H_

#include "tof_reg.h"

uint8_t tof_init(void);
uint8_t tof_process(void);

uint8_t Gesture_Detection(uint8_t *read_buffer);
void Gesture_Detection_IT(void);


#endif /* TOF_TOF_H_ */
