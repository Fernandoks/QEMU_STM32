/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: 6 de jul de 2019
 *      Author: Fernando
 */

#include "stm32f446xx_gpio_driver.h"
#include <assert.h>

/*
 * Peripheral clock
 */

/************************************************************************
 * @fn: 					GPIO_PeriClockControl
 * @Description:			Enables or Disables Clock for GPIO port x
 * @Param1:					GPIOx base address
 * @Param2:					Enable or Disable macro
 * @Return:					-
 * @Note:					-
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnableDisable){

	if (EnableDisable == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}
		else if (pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}
		else if (pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}
		else if (pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}
		else if (pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}
		else if (pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		}
		else if (pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		}

	}
	else{
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}
		else if (pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		}
		else if (pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		}
		else if (pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		}
		else if (pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		}
		else if (pGPIOx == GPIOF){
			GPIOF_PCLK_DI();
		}
		else if (pGPIOx == GPIOH){
			GPIOH_PCLK_DI();
		}

	}
}




void __sizeof(void)
{

}

/*
 * Init and De-Init
 */
/************************************************************************
 * @fn: 					GPIO_Init
 * @Description:			Configuration of the GPIO parameters
 * @Param1:					GPIO_Handle_t contains the Base Address and the GPIO struct parameters
 * @Return:					-
 * @Note:					-
 */


void GPIO_Init(GPIO_Handle_t *pGPIOHandle){

	assert(IS_GPIO_PORT(pGPIOHandle->pGPIOX));

	assert(IS_GPIO_PIN(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	assert(IS_GPIO_MODE(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode));
	assert(IS_GPIO_SPEED(pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed));
	assert(IS_GPIO_PULL(pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl));

	GPIO_PeriClockControl(pGPIOHandle->pGPIOX, ENABLE);


	uint32_t temp = 0;
	//Configure the mode
	//This first line tests if this is a interruption mode
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp |= ( (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode )<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // Moder uses 2 bit for each position
		pGPIOHandle->pGPIOX->MODER |= temp;
	}
	else
	{
		/* Interrupt Mode*/
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//1.Configure FTSR and clear RTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //SET FTSR
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //RESET RTSR
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//1.Configure RTSR and clear FTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //SET RTSR
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //RESET FTSR
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//1.configure FTSR and RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //SET RTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //SET FTSR
		}
		//2. Configure the GPIO port in the SYSCFG_EXTICR
		uint8_t temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4); //The division by 4 will identify each EXTICR register to use - 4 because each register uses 4 bits
		uint8_t temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4); //This will identify the position in the register
		uint8_t portcode = GPIO_BASEADDR_TO_PORT(pGPIOHandle->pGPIOX);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = ( portcode << (temp2 * 4));

		//3. Enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	}
	temp = 0;

	//Configure speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOX->OSPEEDER &= ~(3ul << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //Clearing
	pGPIOHandle->pGPIOX->OSPEEDER |= temp; //setting
	temp = 0;

	//configure PuPd
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOX->PUPDR &= ~(3ul << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //Clearing
	pGPIOHandle->pGPIOX->PUPDR |= temp;
	temp = 0;

	// configure output type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType) << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOX->OTYPER  &= ~(1ul << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //Clearing
	pGPIOHandle->pGPIOX->OTYPER |= temp;
	temp = 0;

	//alternate function
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber < GPIO_PIN_8){
			temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle->pGPIOX->AFRL &= ~(15ul << ( 4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle->pGPIOX->AFRL |= temp;
		}
		else{
			temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber - 8)));
			pGPIOHandle->pGPIOX->AFRH &= ~(15ul << ( 4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle->pGPIOX->AFRH |= temp;
		}
		temp = 0;
	}

}

/************************************************************************
 * @fn: 					GPIO_DeInit
 * @Description:			Reset GPIO parameters
 * @Param1:					Base address
 * @Return:					-
 * @Note:					-
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
//for DeInit we use the RCC AHB1 peripheral reset register (RCC_AHB1RSTR)
	if(pGPIOx == GPIOA){
		GPIOA_RESET();
	}
	else if (pGPIOx == GPIOB){
		GPIOB_RESET();
	}
	else if (pGPIOx == GPIOC){
		GPIOC_RESET();
	}
	else if (pGPIOx == GPIOD){
		GPIOD_RESET();
	}
	else if (pGPIOx == GPIOE){
		GPIOE_RESET();
	}
	else if (pGPIOx == GPIOF){
		GPIOF_RESET();
	}
	else if (pGPIOx == GPIOH){
		GPIOH_RESET();
	}
}


/*
 * Data management
 */
/************************************************************************
 * @fn: 					GPIO_ReadFromInputPin
 * @Description:			Reads data from Pin PinNumber from GPIO Port x
 * @Param1:					GPIOx base address
 * @Param2:					Pin number
 * @Return:					Pin value: SET OR RESET
 * @Note:					-
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

	uint8_t value = (uint8_t)((pGPIOx->IDR >> PinNumber ) & 0x00000001);
	return value;
}

/************************************************************************
 * @fn: 					GPIO_ReadFromInputPort
 * @Description:			Reads data from GPIO Port x
 * @Param1:					GPIOx base address
 * @Return:					16 pin value
 * @Note:					-
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){

	uint16_t value = (uint16_t)(pGPIOx->IDR);
	return value;

	return 0;
}

/************************************************************************
 * @fn: 					GPIO_WriteToOutputPin
 * @Description:			Writes data from Pin PinNumber from GPIO Port x
 * @Param1:					GPIOx base address
 * @Param2:					Pin number
 * @Param3:					Value (SET or RESET)
 * @Return:					None
 * @Note:					-
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){

	if (Value == GPIO_PIN_SET){
		pGPIOx->BSSR |= (1 << PinNumber);
	}
	else{
		pGPIOx->BSSR |= (1 << (PinNumber + 0x10));
	}

}


/************************************************************************
 * @fn: 					GPIO_WriteToOutputPort
 * @Description:			Writes data from Pin PinNumber from GPIO Port x
 * @Param1:					GPIOx base address
 * @Param2:					Value (uint16_t)
 * @Return:					None
 * @Note:					-
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value){

	pGPIOx->BSSR = Value;
}


/************************************************************************
 * @fn: 					GPIO_ToggleOutputPin
 * @Description:			Toggles data from Pin PinNumber from GPIO Port x
 * @Param1:					GPIOx base address
 * @Param2:					Pin number
 * @Return:					None
 * @Note:					-
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

	if (GPIO_ReadFromInputPin(pGPIOx, PinNumber) == SET )
	{
		GPIO_WriteToOutputPin(pGPIOx,PinNumber,GPIO_PIN_RESET);
	}
	else
	{
		GPIO_WriteToOutputPin(pGPIOx,PinNumber,GPIO_PIN_SET);
	}

}

/*
 * IRQ config and handling
 */
/************************************************************************
 * @fn: 					GPIO_IRQConfig
 * @Description:			Configures GPIO interruption
 * @Param1:					IRQ number
 * @Param2:					IRQ priority
 * @Param3:					Enable or Disable
 * @Return:					None
 * @Note:					-
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnableDisable){

	if (EnableDisable == ENABLE)
	{
		if (IRQNumber < 32){
			//NVIC_ISER0
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if( IRQNumber >= 32 && IRQNumber < 64){
			//NVIC_ISER1
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber >= 64 && IRQNumber < 96){
			//NVIC_ISER2
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}
	else if (EnableDisable == DISABLE){
		if (IRQNumber < 32){
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if( IRQNumber >= 32 && IRQNumber < 64){
			//NVIC_ICER1
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber >= 64 && IRQNumber < 96){
			//NVIC_ICER2
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}

/************************************************************************
 * @fn: 					GPIO_IRQHandling
 * @Description:			Handling function for GPIO EXTI
 * @Param1:					Pin Number
 * @Return:					None
 * @Note:					-
	 * In Cortex M4 we have 60 priority registers  IPR0 to IPR59
	 *	each register is divided by 8 bits each IRQnumbers
	 *	To find which IPR register, divide by 4 (3 IRQ in each register)
 ************************************************************************/


void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority){
		//Here we find the IPR register that needs to be configured
		uint8_t iprx = IRQNumber / 4;
		//because each register is divided in 4 different IRQ, we use the MOD 4
		uint8_t iprx_section = IRQNumber % 4;
		/*Each register is 32 bits so we multiply the register number by 4
		 * Notice the 4 lower bits are inaccessible, so we need to shift by 4
		 * to configure the 4 higher bits
		 */
		uint8_t shift_IRQ = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
		*(NVIC_PR_BASEADDRESS + (iprx * 4)) = (IRQPriority << shift_IRQ);
}


/************************************************************************
 * @fn: 					GPIO_IRQHandling
 * @Description:			Handling function for GPIO EXTI
 * @Param1:					Pin Number
 * @Return:					None
 * @Note:					-
 ************************************************************************/


void GPIO_Clear_Interrupt(uint8_t PinNumber){
/*
 * Remember to use the correct implementation of the EXTI_IRQHandler
 * those functions are weak defined in Startup file
 */
	if( (EXTI->PR & (1 << PinNumber)) != 0){
		EXTI->PR |= (1 << PinNumber); // PR register is cleared with setting 1.
	}
}

