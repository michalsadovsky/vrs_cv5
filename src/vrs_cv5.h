/*
 * vrs_cv5.h
 *
 *  Created on: 19. okt. 2016
 *      Author: michalsadovsky
 */

#ifndef VRS_CV5_H_
#define VRS_CV5_H_

void usart_init(void);
void gpio_init(void);
int adc_getMeasurement(void);
void adc_init(void);

#endif /* VRS_CV5_H_ */
