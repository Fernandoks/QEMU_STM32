/*
 * stm32f446_delay.c
 *
 *  Created on: Apr 8, 2020
 *      Author: fernandoks
 */

#include "stm32f446xx_delay.h"


volatile uint32_t ticks;


void SysTickInit (void)
{
	//Enable Systick
	uint32_t temp = 0;
	uint32_t clock = 0;


	temp |= (3ul << 0U); // Enable counter and Exception
	temp |= (1ul << 1U); // 1 for internal clock, 0 for external
	*SYST_CSR |= temp;

	/*
	 * Get Clock - Systick receives (SYSClock*AHBPrescaler)/8
	 */


	RCC_RegDef_t* pRCC = RCC;

	if ( (pRCC->CR & (1ul << 0U)) == RESET  )
	{
		//HSE
	}
	else
	{
		//clock = ( ((pRCC->CR) & (0xFF << 8U) ) >> 8U ) ; //bit 8 to 15 are HSI cal//HSI
		clock = (16000000/8); //bit 8 to 15 are HSI cal//HSI
	}

	//set systick counter to interrupt each ms
	*SYST_RVR = ((clock/(1000))-1);



}


void SysTick_Handler (void)
 {
   ticks++;
 }


inline uint32_t millis (void)
{
   return ticks;
}




void delay_ms (uint32_t t)
{
  uint32_t start, end;
  start = millis();
  end = start + t;
  if (start < end) {
  	while ((millis() >= start) && (millis() < end)) {
  	  // do nothing
  	}
  } else {
    while ((millis() >= start) || (millis() < end)) {
      // do nothing
    };
  }


}
