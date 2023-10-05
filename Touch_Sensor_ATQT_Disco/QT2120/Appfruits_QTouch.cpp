/*
 * This is a library for the Atmel QTouch AT42QT2120
 * But it should work with other QTouch Devices as well
 *
 * This software is part of the firmware running Little Helper. You may use, copy, 
 * modify, merge, publish, distribute, sublicense, and/or sell copies of the Software 
 * but you must include this copyright notice and this permission in all copies or 
 * substantial portions of the Software.
 *
 * Appfruits invests time and resources to make this open source. Please support us
 * and the open-source hardware initiative.
 * 
 * Copyright 2014-2015 Phillip Schuster (@appfruits)
 * http://www.appfruits.com
 * 
 * MIT-License
 */

#include "Appfruits_QTouch.h"

#include "stdbool.h"
#include <stdint-gcc.h>
#include "stdlib.h"

//Appfruits_QTouch::Appfruits_QTouch(uint8_t changePin)

#define QT_Handler &hi2c1


Appfruits_QTouch::Appfruits_QTouch()
{
	_delegate = NULL;
	//_changePin = changePin;
	
	
	for (int i=0;i<QT2120_NUMBER_OF_SAMPLES;i++)
	{
		_deltas[i] = 0.0f;
	}
	_currentDelta = 0;

}

void Appfruits_QTouch::begin()
{

}

bool Appfruits_QTouch::isBitSet(uint8_t b, uint8_t n) 
{ 
  return b & ( 1 << n); 
}

uint8_t Appfruits_QTouch::readRegister8(uint8_t reg) 
{

	uint8_t ret=0;
	uint8_t data=0;
	while(HAL_I2C_GetState(QT_Handler)!= HAL_I2C_STATE_READY);
	ret = HAL_I2C_Master_Transmit(QT_Handler, QT2120_I2C_ADDRESS, &reg, 1, HAL_MAX_DELAY);
	if(ret!=HAL_OK){

		Error_Handler();
	}
	while(HAL_I2C_GetState(QT_Handler)!= HAL_I2C_STATE_READY);
	ret = HAL_I2C_Master_Receive(QT_Handler, QT2120_I2C_ADDRESS, &data, 1, HAL_MAX_DELAY);
	if(ret!=HAL_OK){

		Error_Handler();
	}

	return data;
}

uint16_t Appfruits_QTouch::readRegister16(uint8_t reg) 
{

	uint8_t ret=0;
	uint8_t val[2]={0};
	uint16_t data=0;
	while(HAL_I2C_GetState(QT_Handler)!= HAL_I2C_STATE_READY);
	ret = HAL_I2C_Master_Transmit(QT_Handler, QT2120_I2C_ADDRESS, &reg, 1, HAL_MAX_DELAY);
	if(ret!=HAL_OK){

		Error_Handler();
	}
	while(HAL_I2C_GetState(QT_Handler)!= HAL_I2C_STATE_READY);
	ret = HAL_I2C_Master_Receive(QT_Handler, QT2120_I2C_ADDRESS, val, 2, HAL_MAX_DELAY);
	if(ret!=HAL_OK){

		Error_Handler();
	}

	data = val[1]<<8;
	data = data | val[0];

	return data;
}

void Appfruits_QTouch::writeRegister(uint8_t reg, uint8_t value) 
{

	uint8_t ret=0;
	uint8_t val[2]={0};
	val[0]=reg;
	val[1]=value;
	while(HAL_I2C_GetState(QT_Handler)!= HAL_I2C_STATE_READY);
	ret = HAL_I2C_Master_Transmit(QT_Handler, QT2120_I2C_ADDRESS, val, 2, HAL_MAX_DELAY);
	if(ret!=HAL_OK){

		Error_Handler();
	}

}

void Appfruits_QTouch::enableWheel() 
{
	writeRegister(QT2120_SLIDER_OPTIONS,0xC0);  //Enable WHEEL Bit
}

void Appfruits_QTouch::enableSlider() 
{
	writeRegister(QT2120_SLIDER_OPTIONS,0x80);  //Enable Slider Bit
}

bool Appfruits_QTouch::eventAvailable()
{

	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6)==LOW)
	{
		return true;
	}

	this->setDragging(false);
	_lastSliderPos = 0;

	return false;
}



bool Appfruits_QTouch::isKeyPressed(uint8_t index)
{
	if (index > 11) return false;

	uint8_t address = QT2120_KEYSTATUS_1;
	if (index > 7)
	{
		address = QT2120_KEYSTATUS_2;
		index -= 8;
	}

	uint8_t keyStatus = readRegister8(address);
	return isBitSet(keyStatus,index);
}

void Appfruits_QTouch::setDetectThreshold(uint8_t index, uint8_t value)
{
	if (index > 11) return;

	uint8_t address = QT2120_DETECT_THRESHOLD + index;

	writeRegister(address,value);
}


uint8_t Appfruits_QTouch::getDetectThreshold(uint8_t index)
{
	uint8_t address = QT2120_DETECT_THRESHOLD + index;
	return readRegister8(address);
}

void Appfruits_QTouch::setDetectionIntegrator(uint8_t value)
{
	if (value <= 0) value = 1;
	if (value > 32) value = 32;

	writeRegister(QT2120_DETECTION_INTEGRATOR,value);
}


uint8_t Appfruits_QTouch::getDetectionIntegrator()
{
	return readRegister8(QT2120_DETECTION_INTEGRATOR);
}

bool Appfruits_QTouch::touchDetected()
{
	uint8_t detectionStatus = readRegister8(QT2120_DETECTION_STATUS);
	if (isBitSet(detectionStatus,0))
	{
		return true;
	}

	return false;
}

bool Appfruits_QTouch::sliderDetected()
{
	uint8_t detectionStatus = readRegister8(QT2120_DETECTION_STATUS);
	if (isBitSet(detectionStatus,1))
	{
		return true;
	}

	this->setDragging(false);

	return false;
}

bool Appfruits_QTouch::wheelDetected()
{
	return sliderDetected();
}

uint8_t Appfruits_QTouch::sliderPosition()
{
	uint8_t sliderPos = readRegister8(QT2120_SLIDERPOS);
	return sliderPos;
}

int8_t Appfruits_QTouch::sliderDelta()
{
	if (sliderDetected())
	{
		uint8_t sliderPos = sliderPosition();		
		if (this->_dragging == true)
		{
			int16_t delta = sliderPos - _lastSliderPos;

        	if (delta < -128)
        	{
            	delta += 255;
        	}
        	if (delta > 128)
        	{
           		delta -= 255; 
        	}

        	_deltas[_currentDelta++] = delta;
        	if (_currentDelta > (QT2120_NUMBER_OF_SAMPLES-1)) _currentDelta = 0;

        	delta = 0;
        	for (int i=0;i<QT2120_NUMBER_OF_SAMPLES;i++)
        	{
        		delta += _deltas[i];
        	}
        	delta /= QT2120_NUMBER_OF_SAMPLES;
        
        	_lastSliderPos = sliderPos;

			//LOG_VALUE("Slider-Position",sliderPos);
			//LOG_VALUE("Slider-Delta",delta);
			if (abs(delta) > this->_maxDelta)
			{
				this->_maxDelta = delta;
			}

			if (this->_delegate != NULL)
			{
				if (abs(this->_maxDelta) > 1)
				{
					this->_delegate->onSlider(sliderPos,delta);
				}
				else
				{
					this->_delegate->onTouchDown();
				}
			}

        	return delta;
		}

		_lastSliderPos = sliderPos;
		this->setDragging(true);
	}
	else
	{
		this->setDragging(false);
	}

	return 0;
}

uint8_t Appfruits_QTouch::wheelPosition()
{
	return sliderPosition();
}

int8_t Appfruits_QTouch::wheelDelta()
{
	return sliderDelta();
}

void Appfruits_QTouch::setDragging(bool dragging)
{
	if (_dragging == dragging) return;

	if (dragging)
	{
		this->_dragStartTime = millis();

		if (this->_delegate != NULL)
		{
			this->_delegate->onTouchDown();
		}
	}
	else
	{
		uint32_t duration = millis()-this->_dragStartTime;

		if (this->_delegate != NULL)
		{
			this->_delegate->onTouchUp(duration);
		}

		if (this->_maxDelta <= 1)
		{
			if (_delegate != NULL)
			{
				//Just tapped and not dragged
				if (abs(_lastSliderPos-140) < 10)
				{
					this->_delegate->onTouchLeftButtonPressed(duration);
				}
				else if (abs(_lastSliderPos-220) < 10)
				{
					this->_delegate->onTouchTopButtonPressed(duration);
				}
				else if (abs(_lastSliderPos-35) < 10)
				{
					this->_delegate->onTouchRightButtonPressed(duration);
				}
				else if (abs(_lastSliderPos-90) < 10)
				{
					this->_delegate->onTouchBottomButtonPressed(duration);
				}
			}
		}
	}

	this->_dragging = dragging;
	this->_maxDelta = 0;
}

bool Appfruits_QTouch::isDragging()
{
	return this->_dragging;
}

uint8_t Appfruits_QTouch::getLastSliderPos()
{
	return this->_lastSliderPos;
}

void Appfruits_QTouch::setDelegate(Appfruits_QTouch_Delegate *delegate)
{
	this->_delegate = delegate;
}

void Appfruits_QTouch ::Touch_sensitivity_set(void){

	writeRegister(0x09,0x06);  //working, do not change
	writeRegister(0x10,0x04);

//	writeRegister(0x09,0x0A);
//	writeRegister(0x10,0x05);
}
