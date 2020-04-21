/*
 * stm32f446_delay.h
 *
 *  Created on: Apr 8, 2020
 *      Author: fernandoks
 */

#ifndef INC_STM32F446XX_DELAY_H_
#define INC_STM32F446XX_DELAY_H_

#include "stm32f446xx.h"


void SysTickInit (void);
void SysTick_Handler (void);
uint32_t millis (void);
void delay_ms (uint32_t t);



#endif /* INC_STM32F446XX_DELAY_H_ */
