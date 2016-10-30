/*
 * vrs_cv5.c
 *
 *  Created on: 19. okt. 2016
 *      Author: michalsadovsky
 */
#include "vrs_cv5.h"
#include <stddef.h>
#include "stm32l1xx.h"
#include <stdio.h>




void adc_init1(void)
{
  GPIO_InitTypeDef      GPIO_InitStructure;
  ADC_InitTypeDef       ADC_InitStructure;
  /* Enable GPIO clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
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
NVIC_InitStructure.NVIC_IRQChannel = ADC1_IRQn; //zoznam preruöenŪ nŠjdete v sķbore stm32l1xx.h
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
NVIC_Init(&NVIC_InitStructure);
ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
  ADC_Cmd(ADC1, ENABLE);
  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_ADONS) == RESET)
  {
  }
}


void USART2_IRQHandler(void)         //USART1 INTERAPTION CALLBACK primanie reaguje na znak 'm' ked meni format odosielanej spravy
{
	if(USART2->SR & USART_SR_RXNE)
	{
		recivedValue=USART_ReceiveData(USART2);

		if(recivedValue=='m')
		{
			if(format==0)
			{
				format=1;
				i=0;
			}
			else if(format==1)
			{
				format=0;
			}
		}
		USART_ClearFlag(USART2,USART_FLAG_RXNE);
	}
	if(USART2->SR & USART_SR_TC)   // ked je odoslane odosle spravu znovu alebo dalsi znak zo spravy
		
	{
		if(format==0)
		{
			USART_SendData(USART2,(uint16_t) AD_value);
		}
		if(format==1)
		{
			if (i==0)
			{
				sprintf(s,"%2.2f", (float)(AD_value/4096)*3.3);

			}
			if(i<=2)
			{
				USART_SendData(USART2,(uint16_t) s[i]);
				i++;
			}
			if(i==3)
			{
				USART_SendData(USART2,(uint16_t) 'V');
				i=0;
			}
		}
		USART_ClearFlag(USART2,USART_FLAG_TC);
	}
}

void ADC1_IRQHandler (void)      //ADC1   //INTERAPTION CALLBACK   ULOHA2 citanie po preruseni EOC
{
  if(ADC1->SR & ADC_SR_EOC)
{
   AD_value =ADC1->DR;
  //ADC_ClearFlag(ADC1,ADC_FLAG_EOC);
  // USART_SendData(USART2,4);
}

/*
  if(ADC1->SR & ADC_SR_OVR)
  {
     ADC_ClearFlag(ADC1,ADC_FLAG_OVR);
  }
  */
}


int adc_getMeasurement(void)   //FUNKCIA NA zidkanie hodnoty z adc
{       int value;
	       ADC_SoftwareStartConv(ADC1);
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)){}
    value =ADC_GetConversionValue(ADC1);
   return value;
}

void gpio_init1(void){

	  GPIO_InitTypeDef gpioInitStruc;    //struktura pre GPIOA pre LED
	gpioInitStruc.GPIO_Mode = GPIO_Mode_OUT;
	gpioInitStruc.GPIO_OType =GPIO_OType_PP;
	gpioInitStruc.GPIO_Pin =  GPIO_Pin_5;
	gpioInitStruc.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_Init(GPIOA, &gpioInitStruc);
}

void usart_init1(void){   //funkcia na inicializaciu USARTu

	 USART_InitTypeDef USART_InitStructure;
	  NVIC_InitTypeDef NVIC_InitStructure;
	  GPIO_InitTypeDef GPIO_InitStructure;

	  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);

	    USART_InitStructure.USART_BaudRate = 9600*2;
	    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	    USART_InitStructure.USART_StopBits = USART_StopBits_1;
	    USART_InitStructure.USART_Parity = USART_Parity_No;
	    USART_InitStructure.USART_HardwareFlowControl= USART_HardwareFlowControl_None;
	    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	    USART_Init(USART2, &USART_InitStructure);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitTypeDef NVIC_InitStructure1;
	NVIC_InitStructure1.NVIC_IRQChannel = USART2_IRQn; //zoznam preruöenŪ nŠjdete v sķbore stm32l1xx.h
	NVIC_InitStructure1.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure1.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure1.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure1);
	  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	  USART_ITConfig(USART2, USART_IT_TC, ENABLE);
	  /* Enable USART */
	  USART_Cmd(USART2, ENABLE);
}

