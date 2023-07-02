/*
 * stm32f407xx.h
 *
 *  Created on: Feb 4, 2023
 *      Author: sara
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

#define __vo volatile


/*
 * base addresses of Flash memory and SRAM memories
 *
 * @detail: Compiler by default treat any number as signed integer
 * That is why we should tell the compiler that they are addresses
 * and cannot be signed. We can either typecast them (uint32_t) 0x08000000
 * or add U right after the number
 */
#define	FLASH_BASEADDR				0x08000000U
#define SRAM1_BASEADDR				0x02000000U  		/*112 KB*/
#define SRAM2_BASEADDR           	0x02001C0000		/*112 * 1024 = 1C0000, 1C0000 + SRAM1_BASEADDR = SRAM2_BASEADDR */
#define ROM                         0x1FF0000       	/*ROM is system memory*/
#define SRAM 						SRAM1_BASEADDR

/*
 * AHBx and APBx Bus Peripheral base addresses
 *
 */
#define PERIPH_BASE					0x40000000U
#define	APB1PERIPH_BASE             PERIPH_BASE
#define	APB2PERIPH_BASE				0x40010000U
#define	AHB1PERIPH_BASE				0x40020000U
#define	AHB2PERIPH_BASE				0x50000000U

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 * TODO: Complete for all other peripherals
 *
 */

#define GPIOA_BASEADDR              (AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR              (AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR				(AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR				(AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR				(AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASEADDR				(AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASEADDR				(AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASEADDR				(AHB1PERIPH_BASE + 0x1C00)
#define GPIOI_BASEADDR				(AHB1PERIPH_BASE + 0x2000)
#define RCC_BASEADDR				(AHB1PERIPH_BASE + 0x3800)



/*
 * Base addresses of peripherals which are hanging on APB1 bus
 * TODO: Complete for all other peripherals
 *
 */

#define I2C1_BASEADD    			(APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADD				(APB1PERIPH_BASE + 0x5800)
#define I2C3_BASEADD				(APB1PERIPH_BASE + 0x5C00)


#define SPI2_BASEADD				(APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADD                (APB1PERIPH_BASE + 0x3C00)


#define USART2_BASEADD				(APB1PERIPH_BASE + 0x4400)
#define USART3_BASEADD				(APB1PERIPH_BASE + 0x4800)
#define USART4_BASEADD				(APB1PERIPH_BASE + 0x4C00)
#define USART5_BASEADD				(APB1PERIPH_BASE + 0x5000)



/*
 * Base addresses of peripherals which are hanging on APB2 bus
 * TODO: Complete for all other peripherals
 *
 */

#define EXTI_BASEADD				(AHB2PERIPH_BASE + 0x3C00)
#define SPI1_BASEADD				(AHB2PERIPH_BASE + 0x3000)
#define SYSCFG_BASEADD				(AHB2PERIPH_BASE + 0x3800)
#define USART1_BASEADD				(AHB2PERIPH_BASE + 0x1000)
#define USART6_BASEADD				(AHB2PERIPH_BASE + 0x1400)





/**************************************Peripheral register definition structures***************************************************/

/*
 * Note: Registers of a peripherals specific to MCU
 * e.g Number of registers of SPI peripheral of STM32F4x family of MCUs may be different (more or less)
 * compared to number of registers of SPI peripheral of STM32Lx or STM32F0x family of MCUs
 *Check your Device RM
 *
 * Note data type for each register in uint32_t because each register is 32 bits.
 * */

typedef struct
{
	__vo uint32_t MODER;                 /*Address offset: 0x00*/
	__vo uint32_t OTYPER;				/*Address offset: 0x04*/
	__vo uint32_t OSPEEDR;				/*Address offset: 0x08*/
	__vo uint32_t PUPDR;				/*Address offset: 0x0C*/
	__vo uint32_t IDR;					/*Address offset: 0x10*/
	__vo uint32_t ODR;					/*Address offset: 0x14*/
	__vo uint32_t BSRRL;				/*Address offset: 0x18*/
	__vo uint32_t BSRRH;				/*Address offset: 0x1A*/
	__vo uint32_t LCKR;					/*Address offset: 0x1C*/
	__vo uint32_t AFRL[2];				/*Address offset: 0x20- 0x24*/ /* AFR[0] : GPIO alternate function low register, AFR[1] : GPIO alternate function high register,*/
}GPIO_RegDef_t;



/*
 * Peripheral register definition structure for RCC
 *
 * */
typedef struct
{
	__vo uint32_t  CR;                       /*Address offset: 0x00*/
	__vo uint32_t  PLLCFGR;					/*Address offset: 0x00*/
	__vo uint32_t  CFGR;					/*Address offset: 0x00*/
	__vo uint32_t  CIR;                     /*Address offset: 0x00*/
	__vo uint32_t  AHB1RSTR; 				/*Address offset: 0x00*/
	__vo uint32_t  AHB2RSTR;				/*Address offset: 0x00*/
	__vo uint32_t  AHB3RSTR;				/*Address offset: 0x00*/
	uint32_t       RESERVED0;				/*Address offset: 0x00*/
	__vo uint32_t  APB1RSTR;				/*Address offset: 0x00*/
	__vo uint32_t  APB2RSTR;				/*Address offset: 0x00*/
	uint32_t       RESERVED1[2]; 			/*Address offset: 0x00*/
	__vo uint32_t  AHB1ENR; 				/*Address offset: 0x00*/
	__vo uint32_t  AHB2ENR;  				/*Address offset: 0x00*/
	__vo uint32_t  AHB3ENR;					/*Address offset: 0x00*/
	uint32_t       RESERVED2;				/*Address offset: 0x00*/
	__vo uint32_t  APB1ENR;					/*Address offset: 0x00*/
	__vo uint32_t  APB2ENR;					/*Address offset: 0x00*/
	uint32_t       RESERVED3[2];			/*Address offset: 0x00*/
	__vo uint32_t  AHB1LPENR;				/*Address offset: 0x00*/
	__vo uint32_t  AHB2LPENR;				/*Address offset: 0x00*/
	__vo uint32_t  AHB3LPENR;				/*Address offset: 0x00*/
	uint32_t       RESERVED4;				/*Address offset: 0x00*/
	__vo uint32_t  APB1LPENR;				/*Address offset: 0x00*/
	__vo uint32_t  APB2LPENR;				/*Address offset: 0x00*/
	uint32_t       RESERVED5[2];			/*Address offset: 0x00*/
	__vo uint32_t  BDCR;					/*Address offset: 0x00*/
	__vo uint32_t  CSR;						/*Address offset: 0x00*/
	uint32_t       RESERVED6[2];			/*Address offset: 0x00*/
	__vo uint32_t  SSCGR;					/*Address offset: 0x00*/
	__vo uint32_t  PLLI2SCFGR;				/*Address offset: 0x00*/
	__vo uint32_t  PLLSAICFGR;				/*Address offset: 0x00*/
	__vo uint32_t  DCKCFGR;					/*Address offset: 0x00*/
	__vo uint32_t  CKGATENR;				/*Address offset: 0x00*/
	__vo uint32_t  DCKCFGR2;				/*Address offset: 0x00*/
}RCC_RegDef_t;


#define  GPIOA              (GPIO_RegDef_t*)GPIOA_BASEADDR
#define  GPIOB              (GPIO_RegDef_t*)GPIOB_BASEADDR
#define  GPIOC              (GPIO_RegDef_t*)GPIOC_BASEADDR
#define  GPIOD              (GPIO_RegDef_t*)GPIOD_BASEADDR
#define  GPIOE              (GPIO_RegDef_t*)GPIOE_BASEADDR
#define  GPIOF              (GPIO_RegDef_t*)GPIOF_BASEADDR
#define  GPIOG              (GPIO_RegDef_t*)GPIOG_BASEADDR
#define  GPIOH              (GPIO_RegDef_t*)GPIOH_BASEADDR
#define  GPIOI              (GPIO_RegDef_t*)GPIOI_BASEADDR
#define  RCC                (RCC_RegDef_t*)RCC_BASEADDR



/*
 *  Clock enable for macros GPIOx peripherals
 *
*/
#define GPIOA_PLCK_ENABLE   ( RCC->AHB1ENR |= (1 << 0) )
#define GPIOB_PLCK_ENABLE   ( RCC->AHB1ENR |= (1 << 1) )



/*
 *  Clock enable for macros I2Cx peripherals
 *
*/
#define I2C1_PLCK_ENABLE   ( RCC->AHB1ENR |= (1 << 0) )



/*
 *  Clock enable for macros SPIx peripherals
 *
*/
#define SPI1_PLCK_ENABLE   ( RCC->AHB1ENR |= (1 << 0) )



/*
 *  Clock enable for macros USARTx peripherals
 *
*/
#define USART1_PLCK_ENABLE   ( RCC->AHB1ENR |= (1 << 0) )





#endif /* INC_STM32F407XX_H_ */
