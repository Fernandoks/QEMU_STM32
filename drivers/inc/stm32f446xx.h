

/*
 * STM32F446xx  MCU specific header file
 * by Fernando Kaba Surjus
 * Started 02/07/2019
*/

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_


#include <stdint.h> //stdint is mandatory due uint32_t




/*Generic Macros */
#define __vo 						volatile
#define ENABLE 						1
#define DISABLE						0
#define SET							ENABLE
#define RESET						DISABLE
#define GPIO_PIN_SET				SET
#define GPIO_PIN_RESET				RESET
#define FLAG_SET					SET
#define FLAG_RESET					RESET



/****************************** Processor Specific ******************************/
/*
 * Notes:
 * e.g.
 *
 */

#define NVIC_ISER0					( (__vo uint32_t*) 0xE000E100 )
#define NVIC_ISER1					( (__vo uint32_t*) 0xE000E104 )
#define NVIC_ISER2					( (__vo uint32_t*) 0xE000E108 )
#define NVIC_ISER3					( (__vo uint32_t*) 0xE000E10C )

#define NVIC_ICER0					( (__vo uint32_t*) 0xE000E180 )
#define NVIC_ICER1					( (__vo uint32_t*) 0xE000E184 )
#define NVIC_ICER2					( (__vo uint32_t*) 0xE000E188 )
#define NVIC_ICER3					( (__vo uint32_t*) 0xE000E18C )

#define NVIC_PR_BASEADDRESS			( (__vo uint32_t*) 0xE000E400 )


#define SYST_CSR					( (__vo uint32_t*) 0xE000E010 )
#define SYST_RVR					( (__vo uint32_t*) 0xE000E014 )
#define SYST_CVR					( (__vo uint32_t*) 0xE000E018 )
#define SYST_CALIB					( (__vo uint32_t*) 0xE000E01C )

/*
 * STM32 Cortex M4 processor number of priority bits implemented in Priority register
 */
#define NO_PR_BITS_IMPLEMENTED		4





/*
 * Base Address - Memory mapping
 */
#define Flash_BASEADDRESS 			0x08000000U
#define SRAM1_BASEADDRESS			0x20000000U  //112KB
#define SRAM2_BASEADDRESS			0x20001C00U  //16KB
#define SRAM 						SRAM1_BASEADDRESS
#define ROM							0x1FFF0000U //System Memory


/*
 * AHBx and APBx Bus Peripheral address
 */

#define PERIPH_BASE					0x40000000U
#define APB1PERIPH_BASE				PERIPH_BASE
#define APB2PERIPH_BASE				0x40010000U
#define AHB1PERIPH_BASE				0x40020000U
#define AHB2PERIPH_BASE				0x50000000U

/*
 * AHB1 bus peripheral addresses
 */
#define GPIOA_BASEADDRESS			(AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDRESS			(AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDRESS			(AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDRESS			(AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDRESS			(AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASEADDRESS			(AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASEADDRESS			(AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASEADDRESS			(AHB1PERIPH_BASE + 0x1C00)
#define RCC_BASEADDRESS				(AHB1PERIPH_BASE + 0x3800)

/*
 * APB1 bus peripheral addresses
 */

#define I2C1_BASEADDRESS			(APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDRESS			(APB1PERIPH_BASE + 0x5800)
#define I2C3_BASEADDRESS			(APB1PERIPH_BASE + 0x5C00)
#define SPI2_BASEADDRESS			(APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDRESS			(APB1PERIPH_BASE + 0x3C00)
#define UART4_BASEADDRESS			(APB1PERIPH_BASE + 0x4C00)
#define UART5_BASEADDRESS			(APB1PERIPH_BASE + 0x5000)
#define USART2_BASEADDRESS			(APB1PERIPH_BASE + 0x4400)
#define USART3_BASEADDRESS			(APB1PERIPH_BASE + 0x4800)

/*
 * APB2 bus peripheral addresses
 */
#define SPI1_BASEADDRESS			(APB2PERIPH_BASE + 0x3000)
#define SPI4_BASEADDRESS			(APB2PERIPH_BASE + 0x3400)
#define EXTI_BASEADDRESS			(APB2PERIPH_BASE + 0x3C00)
#define SYSCFG_BASEADDRESS			(APB2PERIPH_BASE + 0x3800)
#define USART1_BASEADDRESS			(APB2PERIPH_BASE + 0x1000)
#define USART6_BASEADDRESS			(APB2PERIPH_BASE + 0x1400)




/****************************** IRQ numbers ******************************/
/*
 * Notes:
 * e.g.
 *
 */

typedef enum
{
/******  Cortex-M4 Processor Exceptions Numbers ****************************************************************/
  NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                                          */
  MemoryManagement_IRQn       = -12,    /*!< 4 Cortex-M4 Memory Management Interrupt                           */
  BusFault_IRQn               = -11,    /*!< 5 Cortex-M4 Bus Fault Interrupt                                   */
  UsageFault_IRQn             = -10,    /*!< 6 Cortex-M4 Usage Fault Interrupt                                 */
  SVCall_IRQn                 = -5,     /*!< 11 Cortex-M4 SV Call Interrupt                                    */
  DebugMonitor_IRQn           = -4,     /*!< 12 Cortex-M4 Debug Monitor Interrupt                              */
  PendSV_IRQn                 = -2,     /*!< 14 Cortex-M4 Pend SV Interrupt                                    */
  SysTick_IRQn                = -1,     /*!< 15 Cortex-M4 System Tick Interrupt                                */
/******  STM32 specific Interrupt Numbers **********************************************************************/
  WWDG_IRQn                   = 0,      /*!< Window WatchDog Interrupt                                         */
  PVD_IRQn                    = 1,      /*!< PVD through EXTI Line detection Interrupt                         */
  TAMP_STAMP_IRQn             = 2,      /*!< Tamper and TimeStamp interrupts through the EXTI line             */
  RTC_WKUP_IRQn               = 3,      /*!< RTC Wakeup interrupt through the EXTI line                        */
  FLASH_IRQn                  = 4,      /*!< FLASH global Interrupt                                            */
  RCC_IRQn                    = 5,      /*!< RCC global Interrupt                                              */
  EXTI0_IRQn                  = 6,      /*!< EXTI Line0 Interrupt                                              */
  EXTI1_IRQn                  = 7,      /*!< EXTI Line1 Interrupt                                              */
  EXTI2_IRQn                  = 8,      /*!< EXTI Line2 Interrupt                                              */
  EXTI3_IRQn                  = 9,      /*!< EXTI Line3 Interrupt                                              */
  EXTI4_IRQn                  = 10,     /*!< EXTI Line4 Interrupt                                              */
  DMA1_Stream0_IRQn           = 11,     /*!< DMA1 Stream 0 global Interrupt                                    */
  DMA1_Stream1_IRQn           = 12,     /*!< DMA1 Stream 1 global Interrupt                                    */
  DMA1_Stream2_IRQn           = 13,     /*!< DMA1 Stream 2 global Interrupt                                    */
  DMA1_Stream3_IRQn           = 14,     /*!< DMA1 Stream 3 global Interrupt                                    */
  DMA1_Stream4_IRQn           = 15,     /*!< DMA1 Stream 4 global Interrupt                                    */
  DMA1_Stream5_IRQn           = 16,     /*!< DMA1 Stream 5 global Interrupt                                    */
  DMA1_Stream6_IRQn           = 17,     /*!< DMA1 Stream 6 global Interrupt                                    */
  ADC_IRQn                    = 18,     /*!< ADC1, ADC2 and ADC3 global Interrupts                             */
  CAN1_TX_IRQn                = 19,     /*!< CAN1 TX Interrupt                                                 */
  CAN1_RX0_IRQn               = 20,     /*!< CAN1 RX0 Interrupt                                                */
  CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                                */
  CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                                */
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                                     */
  TIM1_BRK_TIM9_IRQn          = 24,     /*!< TIM1 Break interrupt and TIM9 global interrupt                    */
  TIM1_UP_TIM10_IRQn          = 25,     /*!< TIM1 Update Interrupt and TIM10 global interrupt                  */
  TIM1_TRG_COM_TIM11_IRQn     = 26,     /*!< TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt */
  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                                    */
  TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                             */
  TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                             */
  TIM4_IRQn                   = 30,     /*!< TIM4 global Interrupt                                             */
  I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                              */
  I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                              */
  I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                              */
  I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                              */
  SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                             */
  SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                             */
  USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                                           */
  USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                                           */
  USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                                           */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                                   */
  RTC_Alarm_IRQn              = 41,     /*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
  OTG_FS_WKUP_IRQn            = 42,     /*!< USB OTG FS Wakeup through EXTI line interrupt                     */
  TIM8_BRK_TIM12_IRQn         = 43,     /*!< TIM8 Break Interrupt and TIM12 global interrupt                   */
  TIM8_UP_TIM13_IRQn          = 44,     /*!< TIM8 Update Interrupt and TIM13 global interrupt                  */
  TIM8_TRG_COM_TIM14_IRQn     = 45,     /*!< TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt */
  TIM8_CC_IRQn                = 46,     /*!< TIM8 Capture Compare global interrupt                             */
  DMA1_Stream7_IRQn           = 47,     /*!< DMA1 Stream7 Interrupt                                            */
  FMC_IRQn                    = 48,     /*!< FMC global Interrupt                                              */
  SDIO_IRQn                   = 49,     /*!< SDIO global Interrupt                                             */
  TIM5_IRQn                   = 50,     /*!< TIM5 global Interrupt                                             */
  SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                             */
  UART4_IRQn                  = 52,     /*!< UART4 global Interrupt                                            */
  UART5_IRQn                  = 53,     /*!< UART5 global Interrupt                                            */
  TIM6_DAC_IRQn               = 54,     /*!< TIM6 global and DAC1&2 underrun error  interrupts                 */
  TIM7_IRQn                   = 55,     /*!< TIM7 global interrupt                                             */
  DMA2_Stream0_IRQn           = 56,     /*!< DMA2 Stream 0 global Interrupt                                    */
  DMA2_Stream1_IRQn           = 57,     /*!< DMA2 Stream 1 global Interrupt                                    */
  DMA2_Stream2_IRQn           = 58,     /*!< DMA2 Stream 2 global Interrupt                                    */
  DMA2_Stream3_IRQn           = 59,     /*!< DMA2 Stream 3 global Interrupt                                    */
  DMA2_Stream4_IRQn           = 60,     /*!< DMA2 Stream 4 global Interrupt                                    */
  CAN2_TX_IRQn                = 63,     /*!< CAN2 TX Interrupt                                                 */
  CAN2_RX0_IRQn               = 64,     /*!< CAN2 RX0 Interrupt                                                */
  CAN2_RX1_IRQn               = 65,     /*!< CAN2 RX1 Interrupt                                                */
  CAN2_SCE_IRQn               = 66,     /*!< CAN2 SCE Interrupt                                                */
  OTG_FS_IRQn                 = 67,     /*!< USB OTG FS global Interrupt                                       */
  DMA2_Stream5_IRQn           = 68,     /*!< DMA2 Stream 5 global interrupt                                    */
  DMA2_Stream6_IRQn           = 69,     /*!< DMA2 Stream 6 global interrupt                                    */
  DMA2_Stream7_IRQn           = 70,     /*!< DMA2 Stream 7 global interrupt                                    */
  USART6_IRQn                 = 71,     /*!< USART6 global interrupt                                           */
  I2C3_EV_IRQn                = 72,     /*!< I2C3 event interrupt                                              */
  I2C3_ER_IRQn                = 73,     /*!< I2C3 error interrupt                                              */
  OTG_HS_EP1_OUT_IRQn         = 74,     /*!< USB OTG HS End Point 1 Out global interrupt                       */
  OTG_HS_EP1_IN_IRQn          = 75,     /*!< USB OTG HS End Point 1 In global interrupt                        */
  OTG_HS_WKUP_IRQn            = 76,     /*!< USB OTG HS Wakeup through EXTI interrupt                          */
  OTG_HS_IRQn                 = 77,     /*!< USB OTG HS global interrupt                                       */
  DCMI_IRQn                   = 78,     /*!< DCMI global interrupt                                             */
  FPU_IRQn                    = 81,     /*!< FPU global interrupt                                              */
  SPI4_IRQn                   = 84,     /*!< SPI4 global Interrupt                                             */
  SAI1_IRQn                   = 87,     /*!< SAI1 global Interrupt                                             */
  SAI2_IRQn                   = 91,     /*!< SAI2 global Interrupt                                             */
  QUADSPI_IRQn                = 92,     /*!< QuadSPI global Interrupt                                          */
  CEC_IRQn                    = 93,     /*!< CEC global Interrupt                                              */
  SPDIF_RX_IRQn               = 94,     /*!< SPDIF-RX global Interrupt                                          */
  FMPI2C1_EV_IRQn             = 95,     /*!< FMPI2C1 Event Interrupt                                           */
  FMPI2C1_ER_IRQn             = 96      /*!< FMPI2C1 Error Interrupt                                           */
} IRQn_Type;




/****************************** Peripheral register structures ******************************/
/*
 * Notes:
 * e.g.
 *
 *********************************************************************************/


/*********************************************************************************
 * RCC_RegDef_t
 *********************************************************************************/
typedef struct{
	__vo uint32_t CR; 				//Address offset:0x00
	__vo uint32_t PLL; 				//Address offset:0x04
	__vo uint32_t CFGR; 			//Address offset:0x08
	__vo uint32_t CIR; 				//Address offset:0x0C
	__vo uint32_t AHB1RSTR;			//Address offset:0x10
	__vo uint32_t AHB2RSTR;			//Address offset:0x14
	__vo uint32_t AHB3RSTR;			//Address offset:0x18
	 uint32_t RESERVED1;			//Address offset:0x1C
	__vo uint32_t APB1RSTR;			//Address offset:0x20
	__vo uint32_t APB2RSTR;			//Address offset:0x24
	uint32_t RESERVED2;				//Address offset:0x28
	uint32_t RESERVED3;				//Address offset:0x2C
	__vo uint32_t AHB1ENR;			//Address offset:0x30
	__vo uint32_t AHB2ENR;			//Address offset:0x34
	__vo uint32_t AHB3ENR;			//Address offset:0x38
	uint32_t RESERVED4;				//Address offset:0x3C
	__vo uint32_t APB1ENR;			//Address offset:0x40
	__vo uint32_t APB2ENR;			//Address offset:0x44
	uint32_t RESERVED5;				//Address offset:0x48
	uint32_t RESERVED6;				//Address offset:0x4C
	__vo uint32_t AHB1_LPENR;		//Address offset:0x50
	__vo uint32_t AHB2_LPENR;		//Address offset:0x54
	__vo uint32_t AHB3_LPENR;		//Address offset:0x58
	uint32_t RESERVED7;				//Address offset:0x5C
	__vo uint32_t APB1_LPENR;		//Address offset:0x60
	__vo uint32_t APB2_LPENR;		//Address offset:0x64
	uint32_t RESERVED8	;			//Address offset:0x68
	uint32_t RESERVED9;				//Address offset:0x6C
	__vo uint32_t BDCR;				//Address offset:0x70
	__vo uint32_t CSR;				//Address offset:0x74
	uint32_t RESERVED10;			//Address offset:0x48
	uint32_t RESERVED11;			//Address offset:0x7C
	__vo uint32_t SSCGR;			//Address offset:0x80
	__vo uint32_t PLLI2SCFGR;		//Address offset:0x84
	__vo uint32_t PLLSAICFGR;		//Address offset:0x88
	__vo uint32_t PLLDCKCFGR;		//Address offset:0x8C
	__vo uint32_t CKGATENR;			//Address offset:0x90
	__vo uint32_t DCKCFGR2;			//Address offset:0x94

} RCC_RegDef_t;

/*********************************************************************************
 * GPIO_RegDef_t
 *********************************************************************************/

typedef struct{
	__vo uint32_t MODER; 			//Address offset:0x00 <= 0x40020000U
	__vo uint32_t OTYPER; 			//Address offset:0x04 <=0x40020004U
	__vo uint32_t OSPEEDER; 		//Address offset:0x08 <=0x40020008U
	__vo uint32_t PUPDR; 			//Address offset:0x0C
	__vo uint32_t IDR;				//Address offset:0x10
	__vo uint32_t ODR;				//Address offset:0x14
	__vo uint32_t BSSR;				//Address offset:0x18
	__vo uint32_t LCKR;				//Address offset:0x1C
	__vo uint32_t AFRL;				//Address offset:0x20
	__vo uint32_t AFRH;				//Address offset:0x24
} GPIO_RegDef_t;




/********************************************************************************
 * EXTI_RegDef_t
 ********************************************************************************/

typedef struct{
	__vo uint32_t IMR; 				//Address offset:0x00
	__vo uint32_t EMR; 				//Address offset:0x04
	__vo uint32_t RTSR; 			//Address offset:0x08
	__vo uint32_t FTSR; 			//Address offset:0x0C
	__vo uint32_t SWIER;			//Address offset:0x10
	__vo uint32_t PR;				//Address offset:0x14
} EXTI_RegDef_t;


/*********************************************************************************
 * SYSCFG_RegDef_t
 *********************************************************************************/

typedef struct{
	__vo uint32_t MEMRMP; 			//Address offset:0x00
	__vo uint32_t PMC; 				//Address offset:0x04
	__vo uint32_t EXTICR[4]; 		/*Defines GPIO port used in EXTI0 to 3		Address offset:0x08*/
 									/*Defines GPIO port used in EXTI4 to 7		Address offset:0x0C*/
									/*Defines GPIO port used in EXTI8 to 11		Address offset:0x10*/
									/*Defines GPIO port used in EXTI12 to 15	Address offset:0x14*/
	uint32_t RESERVED;				//Address offset:0x18
	__vo uint32_t CMPCR;			//Address offset:0x20
	uint32_t RESERVED1[2];			//Address offset:0x24
									//Address offset:0x28
	__vo uint32_t CFGR;				//Address offset:0x2C
} SYSCFG_RegDef_t;

/*********************************************************************************
 * SPI_RegDef_t
 *********************************************************************************/

typedef struct{
	__vo uint32_t CR1; 				//Address offset:0x00
	__vo uint32_t CR2; 				//Address offset:0x04
	__vo uint32_t SR; 				//Address offset:0x08
	__vo uint32_t DR; 				//Address offset:0x0C
	__vo uint32_t CRCPR;			//Address offset:0x10
	__vo uint32_t RXCRCR;			//Address offset:0x14
	__vo uint32_t TXCRCR;			//Address offset:0x18
	__vo uint32_t I2SCFGR;			//Address offset:0x1C
	__vo uint32_t I2SPR;			//Address offset:0x20

} SPI_RegDef_t;



/*********************************************************************************
 * Peripheral definitions (Peripheral base address typecasted to XXX_RegDef_t
 * This is created to easily use with the structure
 *********************************************************************************/

#define GPIOA					((GPIO_RegDef_t*)GPIOA_BASEADDRESS)
#define GPIOB					((GPIO_RegDef_t*)GPIOB_BASEADDRESS)
#define GPIOC					((GPIO_RegDef_t*)GPIOC_BASEADDRESS)
#define GPIOD					((GPIO_RegDef_t*)GPIOD_BASEADDRESS)
#define GPIOE					((GPIO_RegDef_t*)GPIOE_BASEADDRESS)
#define GPIOF					((GPIO_RegDef_t*)GPIOF_BASEADDRESS)
#define GPIOG					((GPIO_RegDef_t*)GPIOG_BASEADDRESS)
#define GPIOH					((GPIO_RegDef_t*)GPIOA_BASEADDRESS)

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
#define GPIOB_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 2) )
#define GPIOC_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 3) )
#define GPIOD_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 4) )
#define GPIOE_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 5) )
#define GPIOF_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 6) )
#define GPIOG_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 7) )
#define GPIOH_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 8) )

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
#define USART6_PCLK_EN()		( RCC->APB2ENR |= (1 <<) 5)

/*
 * Clock Disable Macros for USART
 */
#define USART1_PCLK_DI()		( RCC->APB2ENR &= ~(1 << 4) )
#define USART2_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 17) )
#define USART3_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 18) )
#define UART4_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 19) )
#define UART5_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 20) )
#define USART6_PCLK_DI()		( RCC->APB2ENR &= ~(1 <<) 5)


/*
 * Clock Enable Macros for SYSCFG
 */
#define SYSCFG_PCLK_EN()		( RCC->APB2ENR |= (1 << 14) )

/*
 * Clock Enable Macros for SYSCFG
 */
#define SYSCFG_PCLK_DI()		( RCC->APB2ENR &= ~(1 << 14) )

/*********************************************************************************
 * Bit position definition SPI Peripheral
 *********************************************************************************/

//TODO: Inform the register in comments.

#define SPI_CR1_CPHA 			0
#define SPI_CR1_CPOL 			1
#define SPI_CR1_MSTR 			2
#define SPI_CR1_BR	 			3
#define SPI_CR1_SPE 			6
#define SPI_CR1_LSBFIRST		7
#define SPI_CR1_SSI 			8
#define SPI_CR1_SSM 			9
#define SPI_CR1_RXONLY 			10
#define SPI_CR1_DFF 			11
#define SPI_CR1_CRCNEXT 		12
#define SPI_CR1_CRCEN 			13
#define SPI_CR1_BIDIOE 			14
#define SPI_CR1_BIDIMODE		15


#define SPI_CR2_RXDMAEN 		0
#define SPI_CR2_TXDMAEN 		1
#define SPI_CR2_SSOE	 		2
#define SPI_CR2_FRF 			4
#define SPI_CR2_ERRIE 			5
#define SPI_CR2_RXNEIE 			6
#define SPI_CR2_TXEIE 			7


#define SPI_SR_RXNE				0
#define SPI_SR_TXE				1
#define SPI_SR_CHSIDE			2
#define SPI_SR_UDR				3
#define SPI_SR_CRCERR			4
#define SPI_SR_MODF				5
#define SPI_SR_OVR				6
#define SPI_SR_BSY				7
#define SPI_SR_FRE				8



#endif /* INC_STM32F446XX_H_ */



