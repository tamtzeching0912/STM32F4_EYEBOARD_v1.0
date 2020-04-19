#ifndef __JDY16_H__
#define __JDY16_H__

#include "main.h"
void JDY16_init(UART_HandleTypeDef);
void EnterToAtMode(void);
void EnterToComMode(void);
void BT_Reset(void);
uint8_t BT_IsConnected(void);
void BT_Send(uint8_t*, UART_HandleTypeDef);
void BT_SetUUID(UART_HandleTypeDef);
void BT_SetNAME(UART_HandleTypeDef);



#endif