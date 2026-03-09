/*
 * hx711.h
 *
 *  Created on: Feb 11, 2026
 *      Author: Adel Raef & Hassan Khaled
 */

#ifndef __HX711_H
#define __HX711_H

#include "main.h"

/* Public Functions */
void HX711_Init(TIM_HandleTypeDef *htim);
void HX711_SetCalibration(int32_t tare, float knownOriginal, float knownHX711);
int32_t HX711_GetWeight();

#endif
