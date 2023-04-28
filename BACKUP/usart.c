/*
 * usart.c
 *
 *  Created on: Dec 31, 2022
 *      Author: 24216
 */

#include "hardware.h"
#include "ch32v30x.h"

void hardware_usart3_boot(void){
    USART_InitTypeDef USART_InitStructure = {0};

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART3, &USART_InitStructure);
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    USART_Cmd(USART3, ENABLE);
}

void hardware_usart3_send(const char* data){
    uint32_t i=0;
    while(data[i]!='\0'){
        USART_SendData(USART3, (u16)data[i]);
        /* waiting for sending finish */
        while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET){}
        i++;
    }
}
