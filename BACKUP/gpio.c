/*
 * gpio.c
 *
 *  Created on: Dec 25, 2022
 *      Author: 24216
 */

#include "ch32v30x.h"
#include "hardware.h"

void hardware_gpio_boot(void){
    GPIO_InitTypeDef GPIO_InitStructure={0};

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB , ENABLE );

    /*
     * ADC Pin
     * PA0  ADC0    ADC_CHANNEL_0   Injec   phase A current
     * PA1  ADC1    ADC_CHANNEL_1   Injec   phase B current
     * PA2  ADC1    ADC_CHANNEL_2   Injec   phase C current
     * PA3  ADC0    ADC_CHANNEL_3   Reg     bus voltage
     * PA4  ADC1    ADC_CHANNEL_4   Reg     GND
     */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /*
     * PWM Pin
     * PA8  TIM1_CH1    AH
     * PA9  TIM1_CH2    BH
     * PA10 TIM1_CH3    CH
     * PA11 TIM1_CH4    Trigger for ADC
     * PB13 TIM1_CH1N   AL
     * PB14 TIM1_CH2N   BL
     * PB15 TIM1_CH3N   CL
     */

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init( GPIOA, &GPIO_InitStructure );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOB, &GPIO_InitStructure );



    /*
     * USART Pin
     * PB10 USART3_TX
     * PB11 USART3_RX
     */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);





}
