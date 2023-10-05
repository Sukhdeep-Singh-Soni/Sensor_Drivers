/*
 * Arg_mvp_Qtouch.h
 *
 *  Created on: Aug 24, 2022
 *      Author: Mithlesh Katre
 */

#include "Arg_mvp_Qtouch.h"

#include "Appfruits_QTouch.h"
#include "TouchWheel.h"


#ifdef __cplusplus
extern "C" {
#endif

Appfruits_QTouch qtouch = Appfruits_QTouch();
TouchWheel touchWheel(&qtouch);


/**
  * @brief  Initialize the QT2120 Touch Sensor
  * @retval None
  */
void QTouch_Init(void){

	 touchWheel.begin();
	 qtouch.Touch_sensitivity_set();

}

/**
  * @brief  Read the Keys and Slider data the QT2120 Touch Sensor
  * @retval None
  */
void QTouch_loop(void){

	while(qtouch.touchDetected())
	  {


	    for(uint8_t i=0;i<12;i++)
	    {
	      printf("Key[%d]: %d\r\n",i,qtouch.isKeyPressed(i));

	     }


	    touchWheel.update();

	    uint8_t P = touchWheel.getSliderPosition();

	    printf("TOUCHING...%d\r\n",P);

	    HAL_Delay(200);

	  }

}

/**
  * @brief  Touch event interrupt available on the QT2120 Touch Sensor
  * @retval None
  */
bool QTouch_INT(void){

	return qtouch.eventAvailable();
}

/**
  * @brief  Key0 status of the QT2120 Touch Sensor
  * @retval bool
  */
bool QTouch_key0(void){

	while(qtouch.touchDetected())
	  {

	    return  qtouch.isKeyPressed(0);

	  }
	return false;
}

/**
  * @brief  Key1 status of the QT2120 Touch Sensor
  * @retval bool
  */
bool QTouch_key1(void){

	while(qtouch.touchDetected())
	  {

	    return  qtouch.isKeyPressed(1);

	  }
	return false;
}

/**
  * @brief  Key2 status of the QT2120 Touch Sensor
  * @retval bool
  */
bool QTouch_key2(void){

	while(qtouch.touchDetected())
	  {

	    return  qtouch.isKeyPressed(2);

	  }
	return false;
}

/**
  * @brief  Key3 status of the QT2120 Touch Sensor
  * @retval bool
  */
bool QTouch_key3(void){

	while(qtouch.touchDetected())
	  {

	    return  qtouch.isKeyPressed(3);

	  }
	return false;
}

/**
  * @brief  Key4 status of the QT2120 Touch Sensor
  * @retval bool
  */
bool QTouch_key4(void){

	while(qtouch.touchDetected())
	  {

	    return  qtouch.isKeyPressed(4);

	  }
	return false;
}

/**
  * @brief  Key5 status of the QT2120 Touch Sensor
  * @retval bool
  */
bool QTouch_key5(void){

	while(qtouch.touchDetected())
	  {

	    return  qtouch.isKeyPressed(5);

	  }
	return false;
}

/**
  * @brief  Slider data of the QT2120 Touch Sensor
  * @retval bool
  */
uint8_t QTouch_Slider(void){

	while(qtouch.touchDetected())
	  {

	    touchWheel.update();

	    uint8_t P = touchWheel.getSliderPosition();

	    return P;

	  }
	return false;
}


#ifdef __cplusplus
}
#endif
