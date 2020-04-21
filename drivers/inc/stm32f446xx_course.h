/*
 * stm32f446xx_course.h
 *
 *  Created on: Apr 5, 2020
 *      Author: quentin
 */
#ifdef QUENTIN

#ifndef STM32F446XX_COURSE_H_
#define STM32F446XX_COURSE_H_

#include <stdint.h> //For uint32_t

/* Generic macros */
#define __vo 						volatile
#define ENABLE 						1
#define DISABLE						0
#define PIN_SET_ENABLE 				1
#define PIN_SET_DISABLE				0

/* Internal peripherals */
#define INTERNALPERIPH_BASEADDRESS	0xE0000000U

/* Address Memory */
#define ALIASED_BASEADDRESS	0x00000000U
#define FLASH_BASEADDRESS	0x08000000U
#define ROM_BASEADDRESS		0x1FFF0000U
#define SRAM1_BASEADDRESS	0x20000000U
#define SRAM2_BASEADDRESS	0x2001C000U

/* Bus Peripherals addresses */
#define PERIPH_BASEADDRESS	0x40000000U
#define APB1_BASEADDRESS	PERIPH_BASEADDRESS
#define APB2_BASEADDRESS	0x40010000U
#define AHB1_BASEADDRESS	0x40020000U
#define AHB2_BASEADDRESS	0x50000000U
#define AHB3_BASEADDRESS	0x60000000U

/* Processor specific addresses */
#define NVIC_ISER0			((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1			((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2			((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3			((__vo uint32_t*)0xE000E10C)

#define NVIC_ICER0			((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1			((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2			((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3			((__vo uint32_t*)0xE000E18C)

#define NVIC_IPR_BASEADDRESS			( (__vo uint32_t*) 0xE000E400 )

#define SYST_CSR					( (__vo uint32_t*) 0xE000E010 )
#define SYST_RVR					( (__vo uint32_t*) 0xE000E014 )
#define SYST_CVR					( (__vo uint32_t*) 0xE000E018 )
#define SYST_CALIB					( (__vo uint32_t*) 0xE000E01C )

//4 bits of interrupt priority used (page 238 of reference manual)
#define PRI_BITS_IMPLEMENTED	4

/* APB1 bus peripheral address */
#define TIM2_BASEADDRESS	APB1_BASEADDRESS
#define TIM3_BASEADDRESS	(APB1_BASEADDRESS + 0x0400)
#define TIM4_BASEADDRESS	(APB1_BASEADDRESS + 0x0800)
#define TIM5_BASEADDRESS	(APB1_BASEADDRESS + 0x0C00)
#define TIM6_BASEADDRESS	(APB1_BASEADDRESS + 0x1000)
#define TIM7_BASEADDRESS	(APB1_BASEADDRESS + 0x1400)
#define TIM12_BASEADDRESS	(APB1_BASEADDRESS + 0x1800)
#define TIM13_BASEADDRESS	(APB1_BASEADDRESS + 0x1C00)
#define TIM14_BASEADDRESS	(APB1_BASEADDRESS + 0x2000)
#define RTC_BKP_BASEADDRESS	(APB1_BASEADDRESS + 0x2800)
#define WWDG_BASEADDRESS	(APB1_BASEADDRESS + 0x2C00)
#define IWDG_BASEADDRESS	(APB1_BASEADDRESS + 0x3000)
#define SPI2_BASEADDRESS	(APB1_BASEADDRESS + 0x3800)
#define SPI3_BASEADDRESS	(APB1_BASEADDRESS + 0x3C00)
#define SPDIF_BASEADDRESS	(APB1_BASEADDRESS + 0x4000)
#define USART2_BASEADDRESS	(APB1_BASEADDRESS + 0x4400)
#define USART3_BASEADDRESS	(APB1_BASEADDRESS + 0x4800)
#define UART4_BASEADDRESS	(APB1_BASEADDRESS + 0x4C00)
#define UART5_BASEADDRESS	(APB1_BASEADDRESS + 0x5000)
#define I2C1_BASEADDRESS	(APB1_BASEADDRESS + 0x5400)
#define I2C2_BASEADDRESS 	(APB1_BASEADDRESS + 0x5800)
#define I2C3_BASEADDRESS	(APB1_BASEADDRESS + 0x5C00)
#define CAN1_BASEADDRESS	(APB1_BASEADDRESS + 0x6400)
#define CAN2_BASEADDRESS	(APB1_BASEADDRESS + 0x6800)
#define HDMI_BASEADDRESS	(APB1_BASEADDRESS + 0x6C00)
#define PWR_BASEADDRESS		(APB1_BASEADDRESS + 0x7000)
#define DAC_BASEADDRESS		(APB1_BASEADDRESS + 0x7400)

/* APB2 bus peripheral address */
#define TIM1_BASEADDRESS	APB2_BASEADDRESS
#define TIM8_BASEADDRESS	(APB2_BASEADDRESS + 0x0400)
#define USART1_BASEADDRESS	(APB2_BASEADDRESS + 0x1000)
#define USART6_BASEADDRESS	(APB2_BASEADDRESS + 0x1400)
#define ADC_BASEADDRESS		(APB2_BASEADDRESS + 0x2000)
#define SDMMC_BASEADDRESS	(APB2_BASEADDRESS + 0x2C00)
#define SPI1_BASEADDRESS	(APB2_BASEADDRESS + 0x3000)
#define SPI4_BASEADDRESS	(APB2_BASEADDRESS + 0x3400)
#define SYSCFG_BASEADDRESS	(APB2_BASEADDRESS + 0x3800)
#define EXTI_BASEADDRESS	(APB2_BASEADDRESS + 0x3C00)
#define TIM9_BASEADDRESS	(APB2_BASEADDRESS + 0x4000)
#define TIM10_BASEADDRESS	(APB2_BASEADDRESS + 0x4400)
#define TIM11_BASEADDRESS	(APB2_BASEADDRESS + 0x4800)
#define SAI1_BASEADDRESS	(APB2_BASEADDRESS + 0x5800)
#define SAI2_BASEADDRESS	(APB2_BASEADDRESS + 0x5C00)

/* AHB1 bus peripheral address */
#define GPIOA_BASEADDRESS	AHB1_BASEADDRESS
#define GPIOB_BASEADDRESS	(AHB1_BASEADDRESS + 0x0400)
#define GPIOC_BASEADDRESS	(AHB1_BASEADDRESS + 0x0800)
#define GPIOD_BASEADDRESS	(AHB1_BASEADDRESS + 0x0C00)
#define GPIOE_BASEADDRESS	(AHB1_BASEADDRESS + 0x1000)
#define GPIOF_BASEADDRESS	(AHB1_BASEADDRESS + 0x1400)
#define GPIOG_BASEADDRESS	(AHB1_BASEADDRESS + 0x1800)
#define GPIOH_BASEADDRESS	(AHB1_BASEADDRESS + 0x1C00)
#define CRC_BASEADDRESS		(AHB1_BASEADDRESS + 0x3000)
#define RCC_BASEADDRESS		(AHB1_BASEADDRESS + 0x3800)
#define FLASHINTERFACE_BASEADDRESS	(AHB1_BASEADDRESS + 0x3C00)
#define BKPSRAM_BASEADDRESS	(AHB1_BASEADDRESS + 0x4000)
#define DMA1_BASEADDRESS	(AHB1_BASEADDRESS + 0x6000)
#define DMA2_BASEADDRESS	(AHB1_BASEADDRESS + 0x6400)
#define USBHS_BASEADDRESS	(0x40040000U)

/* AHB2 bus peripheral address */
#define USBFS_BASEADDRESS	AHB2_BASEADDRESS
#define DCMI_BASEADDRESS	(AHB2_BASEADDRESS + 0x50000)

/********************* Register structures *********************/

/* RCC_RegDef_t */
typedef struct{
	__vo uint32_t CR;
	__vo uint32_t PLL;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	 uint32_t RESERVED1;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	uint32_t RESERVED2;
	uint32_t RESERVED3;
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	uint32_t RESERVED4;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	uint32_t RESERVED5;
	uint32_t RESERVED6;
	__vo uint32_t AHB1_LPENR;
	__vo uint32_t AHB2_LPENR;
	__vo uint32_t AHB3_LPENR;
	uint32_t RESERVED7;
	__vo uint32_t APB1_LPENR;
	__vo uint32_t APB2_LPENR;
	uint32_t RESERVED8	;
	uint32_t RESERVED9;
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	uint32_t RESERVED10;
	uint32_t RESERVED11;
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	__vo uint32_t PLLSAICFGR;
	__vo uint32_t PLLDCKCFGR;
	__vo uint32_t CKGATENR;
	__vo uint32_t DCKCFGR2;
} RCC_RegDef_t;

/* GPIO_RegDef_t */
typedef struct{
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDER;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFRL;
	__vo uint32_t AFRH;
}GPIO_RegDef_t;

/* EXTI_RegDef_t */
typedef struct{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
} EXTI_RegDef_t;

/* SYSCFG_RegDef_t */
typedef struct{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4]; 		/*EXTICR1 -> EXTI0 to 3*/
 									/*EXTICR2 -> EXTI4 to 7*/
									/*EXTICR1 -> EXTI8 to 11*/
									/*EXTICR4 -> EXTI12 to 15*/
	uint32_t RESERVED;
	__vo uint32_t CMPCR;
	uint32_t RESERVED1[2];
	__vo uint32_t CFGR;
} SYSCFG_RegDef_t;

/* SPI_RegDef_t */
typedef struct{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;
} SPI_RegDef_t;

#define GPIOA					((GPIO_RegDef_t*)GPIOA_BASEADDRESS)
#define GPIOB					((GPIO_RegDef_t*)GPIOB_BASEADDRESS)
#define GPIOC					((GPIO_RegDef_t*)GPIOC_BASEADDRESS)
#define GPIOD					((GPIO_RegDef_t*)GPIOD_BASEADDRESS)
#define GPIOE					((GPIO_RegDef_t*)GPIOE_BASEADDRESS)
#define GPIOF					((GPIO_RegDef_t*)GPIOF_BASEADDRESS)
#define GPIOG					((GPIO_RegDef_t*)GPIOG_BASEADDRESS)
#define GPIOH					((GPIO_RegDef_t*)GPIOH_BASEADDRESS)

#define RCC						((RCC_RegDef_t*)RCC_BASEADDRESS)

#define EXTI					((EXTI_RegDef_t*)EXTI_BASEADDRESS)

#define SYSCFG					((SYSCFG_RegDef_t*)SYSCFG_BASEADDRESS)


#define SPI1					((SPI_RegDef_t*)SPI1_BASEADDRESS)
#define SPI2					((SPI_RegDef_t*)SPI2_BASEADDRESS)
#define SPI3					((SPI_RegDef_t*)SPI3_BASEADDRESS)
#define SPI4					((SPI_RegDef_t*)SPI4_BASEADDRESS)

/*
 * Clock Enable Macros for GPIOx
 */

#define GPIOA_PCLK_EN()			( RCC->AHB1ENR |= (1 << 0) )
#define GPIOB_PCLK_EN()			( RCC->AHB1ENR |= (1 << 1) )
#define GPIOC_PCLK_EN()			( RCC->AHB1ENR |= (1 << 2) )
#define GPIOD_PCLK_EN()			( RCC->AHB1ENR |= (1 << 3) )
#define GPIOE_PCLK_EN()			( RCC->AHB1ENR |= (1 << 4) )
#define GPIOF_PCLK_EN()			( RCC->AHB1ENR |= (1 << 5) )
#define GPIOG_PCLK_EN()			( RCC->AHB1ENR |= (1 << 6) )
#define GPIOH_PCLK_EN()			( RCC->AHB1ENR |= (1 << 7) )

/*
 * Clock Disable Macros for GPIOx
 */
#define GPIOA_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 0) )
#define GPIOB_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 1) )
#define GPIOC_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 2) )
#define GPIOD_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 3) )
#define GPIOE_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 4) )
#define GPIOF_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 5) )
#define GPIOG_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 6) )
#define GPIOH_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 7) )

/*
 * Reset GPIO Macros
 */
#define GPIOA_RESET()			do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0));}while(0)
#define GPIOB_RESET()			do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1));}while(0)
#define GPIOC_RESET()			do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2));}while(0)
#define GPIOD_RESET()			do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3));}while(0)
#define GPIOE_RESET()			do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4));}while(0)
#define GPIOF_RESET()			do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5));}while(0)
#define GPIOG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6));}while(0)
#define GPIOH_RESET()			do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7));}while(0)


#define GPIO_BASEADDR_TO_PORT(x)	( (x == GPIOA ) ? 0 :\
									(x == GPIOB ) ? 1 :\
									(x == GPIOC ) ? 2 :\
									(x == GPIOD ) ? 3 :\
									(x == GPIOE ) ? 4 :\
									(x == GPIOF ) ? 5 :\
									(x == GPIOG ) ? 6 :\
									(x == GPIOH ) ? 7 :0)
/*
 * Clock Enable Macros for I2Cxx
 */
#define I2C1_PCLK_EN()			( RCC->APB1ENR |= (1 << 21) )
#define I2C2_PCLK_EN()			( RCC->APB1ENR |= (1 << 22) )
#define I2C3_PCLK_EN()			( RCC->APB1ENR |= (1 << 23) )
/*
 * Clock Disable Macros for I2Cxx
 */
#define I2C1_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 21) )
#define I2C2_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 22) )
#define I2C3_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 23) )


/*
 * Clock Enable Macros for SPI
 */
#define SPI1_PCLK_EN()			( RCC->APB2ENR |= (1 << 12) )
#define SPI2_PCLK_EN()			( RCC->APB1ENR |= (1 << 14) )
#define SPI3_PCLK_EN()			( RCC->APB1ENR |= (1 << 15) )
#define SPI4_PCLK_EN()			( RCC->APB2ENR |= (1 << 13) )

/*
 * Clock Disable Macros for SPI
 */
#define SPI1_PCLK_DI()			( RCC->APB2ENR &= ~(1 << 12) )
#define SPI2_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 14) )
#define SPI3_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 15) )
#define SPI4_PCLK_DI()			( RCC->APB2ENR &= ~(1 << 13) )

/*
 * Reset GPIO Macros
 */
#define SPI1_RESET()			do{ (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12));}while(0)
#define SPI2_RESET()			do{ (RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14));}while(0)
#define SPI3_RESET()			do{ (RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15));}while(0)
#define SPI4_RESET()			do{ (RCC->APB2RSTR |= (1 << 13)); (RCC->APB2RSTR &= ~(1 << 13));}while(0)

/*
 * Clock Enable Macros for USART
 */
#define USART1_PCLK_EN()		( RCC->APB2ENR |= (1 << 4) )
#define USART2_PCLK_EN()		( RCC->APB1ENR |= (1 << 17) )
#define USART3_PCLK_EN()		( RCC->APB1ENR |= (1 << 18) )
#define UART4_PCLK_EN()			( RCC->APB1ENR |= (1 << 19) )
#define UART5_PCLK_EN()			( RCC->APB1ENR |= (1 << 20) )
#define USART6_PCLK_EN()		( RCC->APB2ENR |= (1 << 5)	)

#endif /* STM32F446RE_H_ */

#endif /*QUENTIN*/
