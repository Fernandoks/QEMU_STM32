/*
 * stm32f446xx_gpio_driver_course.h
 *
 *  Created on: Apr 7, 2020
 *      Author: quentin
 */

#ifdef QUENTIN

#ifndef INC_STM32F446XX_GPIO_DRIVER_COURSE_H_
#define INC_STM32F446XX_GPIO_DRIVER_COURSE_H_

#include "stm32f446xx_course.h"

/* GPIO_PIN_CONF_TypeDef */
typedef struct{
	uint8_t GPIO_Pin_Number; /* bitwise it can take multiple pin numbers */
	uint8_t GPIO_Mode;
	uint8_t GPIO_Type;
	uint8_t GPIO_Speed;
	uint8_t GPIO_PUPD;
	uint8_t GPIO_AlternateMode;
}GPIO_PIN_CONF_TypeDef;

/* GPIO_Handler_TypedDef */
typedef struct{
	GPIO_RegDef_t* pGPIO; /*holds register addresses*/
	GPIO_PIN_CONF_TypeDef GPIO_Pin_Config; /* holds pins settings */
}GPIO_Handler_TypedDef;

/* GPIO Pin Number */
#define GPIO_PIN_0			0
#define GPIO_PIN_1			1
#define GPIO_PIN_2			2
#define GPIO_PIN_3			3
#define GPIO_PIN_4			4
#define GPIO_PIN_5			5
#define GPIO_PIN_6			6
#define GPIO_PIN_7			7
#define GPIO_PIN_8			8
#define GPIO_PIN_9			9
#define GPIO_PIN_10			10
#define GPIO_PIN_11			11
#define GPIO_PIN_12			12
#define GPIO_PIN_13			13
#define GPIO_PIN_14			14
#define GPIO_PIN_15			15

/* GPIO mode */
#define GPIO_MODE_INPUT		0
#define GPIO_MODE_OUPUT		1
#define GPIO_MODE_ALTER		2
#define GPIO_MODE_ANALOG	3

/* GPIO output type */
#define GPIO_TYPE_PUSHPULL	0
#define GPIO_TYPE_OPENDRAIN	1

/* GPIO output speed */
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MED		1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3

/* GPIO pull-up pull-down */
#define GPIO_PUPD_NONE		0
#define GPIO_PUPD_PU		1
#define GPIO_PUPD_PD		2
#define GPIO_PUPD_RESERVED	3

/************** GPIO ACTIONS *********************/

/* Peripheral Clock Control */
void GPIO_PCLK(GPIO_RegDef_t* pGPIO, uint8_t status);

/* GPIO Initialization */
void GPIO_Initialize(GPIO_Handler_TypedDef* pGPIO_Handler);
void GPIO_DeInitialize(GPIO_RegDef_t* pGPIO); /* with the GPIO regdef we can use the GPIO_RESET function */

/* GPIO PIN management */
uint8_t GPIO_ReadInputPIN(GPIO_RegDef_t* pGPIO, uint8_t PinNumber); /* we don't use the handler because it might contain more than one pin number */
uint16_t GPIO_ReadInputPORT(GPIO_RegDef_t* pGPIO);
void GPIO_WriteOutputPIN(GPIO_RegDef_t* pGPIO, uint8_t PinNumber, uint8_t value);
void GPIO_WriteOutputPORT(GPIO_RegDef_t* pGPIO, uint16_t value);
void GPIO_ToggleOutputPIN(GPIO_RegDef_t* pGPIO, uint8_t PinNumber);

void GPIO_Config_IRQ(uint8_t IRQNumber, uint8_t EnableDisable);
void GPIO_Clear_IRQ(uint8_t PinNumber);
void GPIO_Config_Priority_IRQ(uint8_t IRQNumber, uint8_t IRQ_Priority);

#endif /* INC_STM32F446XX_GPIO_DRIVER_COURSE_H_ */


#endif /*QUENTIN*/

