/*
 * at42qt2120.h
 *
 *  Created on: Oct 11, 2022
 *      Author: Sukhdeep
 */

#ifndef TOUCH_SENSOR_AT42QT2120_H_
#define TOUCH_SENSOR_AT42QT2120_H_

//includes
#include<stdint.h>
#include<stdio.h>
#include "usart.h"
#include "i2c.h"

//defines
#define DEV_ADDR						(0x1C << 1)
#define I2C_HANDLE						(&hi2c1)
#define CID								(0x3E)
#define SLIDER_OPTIONS_REG				(14)
#define DETECTION_STATUS_REG			(2)
#define SLIDER_POSITION_REG				(5)
#define DI_REG							(11)
#define KEY0_DETECT_THRESHOLD_REG		(16)
#define TRD_REG							(12)
#define TTD_REG							(9)
#define ATD_REG							(10)
#define DHT_REG							(13)
#define HOLD_TIME						(5)
#define KEYS_THRESHOLD					(8)
#define SLIDER_SENSITIVITY				(90)
#define TAP_SENSITIVITY					(30)

/*Global Variables*/
extern uint8_t slider_previousVal, slider_previousVal1;
extern uint8_t tap_speed, tap_detect, touch_status;
extern uint8_t cid;
extern int slider_newVal, slider_detected, slider_ret_val;
extern uint8_t glbl_int, glbl_reg;
extern uint8_t press, release;

//function prototypes
uint8_t check_cid(void);
uint8_t read8_i2c(uint8_t reg);
uint16_t read16_i2c(uint8_t reg);
uint8_t write8_i2c(uint8_t reg, uint8_t value);
void enable_slider(void);
uint8_t key_detect(void);
uint8_t is_keyPressed(uint8_t index);
uint8_t is_bitSet(uint8_t byte, uint8_t pos);
uint8_t key_val(uint8_t key_no);
uint8_t sliderPosition(void);
uint8_t slider_detect(void);
uint8_t slider_val(void);
void set_detectIntegrator(uint8_t value);
void set_keyThreshold(uint8_t index, uint8_t threshold);
void set_timeRecalDelay(uint8_t delay);
void set_TTD(uint8_t value);
void set_ATD(uint8_t value);
void enable_wheel(void);
uint8_t get_TTD(void);
uint8_t get_ATD(void);
uint8_t get_ketThreshold(uint8_t index);
uint8_t get_detectIntegrator(void);
void set_DHT(uint8_t time);
void touchSensor_Init(void);
void touchSensor_Process(void);
void finger_leave_detect(uint8_t* prev_value);
void finger_press_detect(uint8_t* prev_value);
void delay_us (uint16_t us);
void mvp_touch_hold_detect(void);
void mvp_touch_single_tap_detect(void);
int8_t mvp_touch_forward_backward_tap_detect(void);

typedef enum {
	NO_TOUCH 		= 0,
	HOLD 			= 1,
	TAP				= 2,
	SLIDE_FORWARD 	= 3,
	SLIDE_BACKWARD 	= 4
} Touch_enum_t;

#endif /* TOUCH_SENSOR_AT42QT2120_H_ */
