/*
 * Arg_mvp_Qtouch.h
 *
 *  Created on: Aug 24, 2022
 *      Author: Mithlesh Katre
 */

#ifndef ARG_MVP_QTOUCH_H_
#define ARG_MVP_QTOUCH_H_

#include "stdbool.h"
#include "stdio.h"

#ifdef __cplusplus
extern "C" {
#endif


void QTouch_loop(void);
void QTouch_Init(void);
bool QTouch_INT(void);

bool QTouch_key0(void);
bool QTouch_key1(void);
bool QTouch_key2(void);
bool QTouch_key3(void);
bool QTouch_key4(void);
bool QTouch_key5(void);
uint8_t QTouch_Slider(void);


#ifdef __cplusplus
}
#endif

#endif /* ARG_MVP_QTOUCH_H_ */
