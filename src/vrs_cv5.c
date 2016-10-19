/*
 * vrs_cv5.c
 *
 *  Created on: 19. okt. 2016
 *      Author: michalsadovsky
 */
#include "vrs_cv5.h"
#include <stddef.h>
#include "stm32l1xx.h"

int valueOVR;
int valueEOC;

void adc_init(void)
{
  GPIO_InitTypeDef      GPIO_InitStructure;
  ADC_InitTypeDef       ADC_InitStructure;
  /* Enable GPIO clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);//Opraviť a upraviť
  /* Configure ADCx Channel 2 as analog input */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
/* Enable the HSI oscillator */
  RCC_HSICmd(ENABLE);
/* Check that HSI oscillator is ready */
  while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET);
  /* Enable ADC clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  /* Initialize ADC structure */
  ADC_StructInit(&ADC_InitStructure);
  /* ADC1 configuration */
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC1, &ADC_InitStructure);
/* ADCx regular channel8 configuration */
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_16Cycles);
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);


 	NVIC_InitTypeDef NVIC_InitStructure;

 	NVIC_InitStructure.NVIC_IRQChannel = ADC1_IRQn; //zoznam prerušení nájdete v súbore stm32l1xx.h


 	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;

 	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;

 	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

 	NVIC_Init(&NVIC_InitStructure);
 	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
 	ADC_ITConfig(ADC1, ADC_IT_OVR, ENABLE);
 	//ADC_ITConfig(ADC1, ADC_IT_EOC | ADC_IT_OVR , ENABLE);
  /* Enable the ADC */
  ADC_Cmd(ADC1, ENABLE);
  /* Wait until the ADC1 is ready */
  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_ADONS) == RESET)
  {
  }
  /* Start ADC Software Conversion */
  ADC_SoftwareStartConv(ADC1);



}




void ADC1_IRQHandler (void)
{
  if(ADC1->SR & ADC_SR_EOC)
{
   valueEOC = ADC1->DR;
}


  if(ADC1->SR & ADC_SR_OVR)
  {
     valueOVR = ADC1->DR;
  }
}


int adc_getMeasurement(void)
{       int value;
	       ADC_SoftwareStartConv(ADC1);
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)){}
    value =ADC_GetConversionValue(ADC1);
   return value;
}

void gpio_init(void){

	  GPIO_InitTypeDef gpioInitStruc;    //struktura pre GPIOA
	gpioInitStruc.GPIO_Mode = GPIO_Mode_OUT;
	gpioInitStruc.GPIO_OType =GPIO_OType_PP;
	gpioInitStruc.GPIO_Pin = GPIO_Pin_5;
	gpioInitStruc.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_Init(GPIOA, &gpioInitStruc);
}

void usart_init(void){

	 USART_InitTypeDef USART_InitStructure;
	  NVIC_InitTypeDef NVIC_InitStructure;
	  GPIO_InitTypeDef GPIO_InitStructure;

	  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	  RCC_APB1PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);


	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);


	    USART_InitStructure.USART_BaudRate = 115200;
	    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	    USART_InitStructure.USART_StopBits = USART_StopBits_1;
	    USART_InitStructure.USART_Parity = USART_Parity_No;
	    USART_InitStructure.USART_HardwareFlowControl= USART_HardwareFlowControl_None;
	    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	    USART_Init(USART1, &USART_InitStructure);


	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);



	  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);


	NVIC_InitTypeDef NVIC_InitStructure1;

	NVIC_InitStructure1.NVIC_IRQChannel = ADC1_IRQn; //zoznam prerušení nájdete v súbore stm32l1xx.h


	NVIC_InitStructure1.NVIC_IRQChannelPreemptionPriority = 1;

	NVIC_InitStructure1.NVIC_IRQChannelSubPriority = 0;

	NVIC_InitStructure1.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure1);

	  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	  /* Enable USART */
	  USART_Cmd(USART1, ENABLE);

}




