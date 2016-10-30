/*
 * vrs_cv5.h
 *
 *  Created on: 19. okt. 2016
 *      Author: michalsadovsky
 */

#ifndef VRS_CV5_H_
#define VRS_CV5_H_

#include <string.h>
#include <stddef.h>
#include "stm32l1xx.h"

int valueOVR;
uint16_t valueEOC;
uint16_t recivedValue;
int format;
uint16_t AD_value;
char buffer[5];
int i;
char s[4];
void usart_init1(void);
void gpio_init1(void);
int adc_getMeasurement(void);
void adc_init1(void);

#endif /* VRS_CV5_H_ */
