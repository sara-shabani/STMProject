/*
 * stm32f407xx.h
 *
 *  Created on: Jun 23, 2023
 *      Author: sara
 */

#ifndef STM32F407XX_H_
#define STM32F407XX_H_

#include <stdint.h>

#define __vo volatile


/*************************Processor specific details*****************************************/
/*
 * ARM cortex Mx processor NVIC ISER register addresses
 */


#define NVIC_ISER0  	((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1  	((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2  	((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3  	((__vo uint32_t*)0xE000E10C)



/*
 * ARM cortex Mx processor NVIC ICER register addresses
 */


#define NVIC_ICER0  	((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1  	((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2  	((__vo uint32_t*)0XE000E188)
#define NVIC_ICSER3  	((__vo uint32_t*)0XE000E18C)



/*
 * ARM cortex Mx processor priority register addresses
 */

#define NVIC_PR_BASE_ADDR  	((__vo uint32_t*)0xE000E400)




#define NO_PR_BITS_IMPLEMENTED      4

#define NVIC_IRQ_PRIO15				15


/*
 * base addresses of Flash memory and SRAM memories
 *
 * @detail: Compiler by default treat any number as signed integer
 * That is why we should tell the compiler that they are addresses
 * and cannot be signed. We can either type cast them (uint32_t) 0x08000000
 * or add U right after the number
 * EX : Either 0x8000000U or (uint32_t)0x8000000
 */
#define	FLASH_BASEADDR				0x08000000U			/*Main memory*/
#define SRAM1_BASEADDR				0x02000000U  		/*112 KB*/
#define SRAM2_BASEADDR           	0x02001C0000U		/*112 * 1024 = 1C0000, 1C0000 + SRAM1_BASEADDR = SRAM2_BASEADDR */
#define ROM                         0x1FF0000U      	/*ROM is system memory*/
#define SRAM 						SRAM1_BASEADDR

/*
 * AHBx and APBx Bus Peripheral base addresses
 * Note: Fist Step : Different peripherals are hanging on different buses
 * AHB: AHB bus is used for those peripherals which need high speed data communication
 * (EX. Camera interfaces, GPIOs)
 * APB: APB bus is used for those peripherals for which low speed communication would suffice.
 *
 */
#define PERIPH_BASE					0x40000000U
#define	APB1PERIPH_BASE             PERIPH_BASE  //See: UserMAnual Memory MAp GPIOA, GPIOB, GPIOC ...
#define	APB2PERIPH_BASE				0x40010000U
#define	AHB1PERIPH_BASE				0x40020000U  //See: UserMAnual  I2C1, I2C2, I2C3, SPI2 ...
#define	AHB2PERIPH_BASE				0x50000000U  //See: UserMAnual  SPI1, USART1EXTI, SYSCF, ...

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
#define UART4_BASEADD				(APB1PERIPH_BASE + 0x4C00)
#define UART5_BASEADD				(APB1PERIPH_BASE + 0x5000)



/*
 * Base addresses of peripherals which are hanging on APB2 bus
 * TODO: Complete for all other peripherals
 *
 */

#define EXTI_BASEADD				(APB2PERIPH_BASE + 0x3C00)
#define SPI1_BASEADD				(APB2PERIPH_BASE + 0x3000)
#define SYSCFG_BASEADD				(APB2PERIPH_BASE + 0x3800)
#define USART1_BASEADD				(APB2PERIPH_BASE + 0x1000)
#define USART6_BASEADD				(APB2PERIPH_BASE + 0x1400)




/********************************Peripheral register definition structures***************************************************/

/*
 * Note: Second Step:Structuring peripheral register details:
 * Registers of a peripherals specific to MCU
 * e.g Number of registers of SPI peripheral of STM32F4x family of MCUs may be different (more or less)
 * compared to number of registers of SPI peripheral of STM32Lx or STM32F0x family of MCUs
 *Check your Device RM
 *
 * Note data type for each register in uint32_t because each register is 32 bits.
 * Generic structure for all GPIO ports
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


#define  GPIOA              (GPIO_RegDef_t*)GPIOA_BASEADDR
#define  GPIOB              (GPIO_RegDef_t*)GPIOB_BASEADDR
#define  GPIOC              (GPIO_RegDef_t*)GPIOC_BASEADDR
#define  GPIOD              (GPIO_RegDef_t*)GPIOD_BASEADDR
#define  GPIOE              (GPIO_RegDef_t*)GPIOE_BASEADDR
#define  GPIOF              (GPIO_RegDef_t*)GPIOF_BASEADDR
#define  GPIOG              (GPIO_RegDef_t*)GPIOG_BASEADDR
#define  GPIOH              (GPIO_RegDef_t*)GPIOH_BASEADDR
#define  GPIOI              (GPIO_RegDef_t*)GPIOI_BASEADDR

/*
 * Peripheral register definition structure for RCC
 *
 * */
typedef struct
{
	__vo uint32_t  CR;                       /*Address offset: 0x00*/
	__vo uint32_t  PLLCFGR;					/*Address offset: 0x04*/
	__vo uint32_t  CFGR;					/*Address offset: 0x08*/
	__vo uint32_t  CIR;                     /*Address offset: 0x0C*/
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

#define  RCC                ((RCC_RegDef_t*)RCC_BASEADDR)





/*
 * Peripheral register definition structure for EXTI
 *
 * */
typedef struct
{
	__vo uint32_t IMR;               	/*Address offset: 0x00*/
	__vo uint32_t EMR;					/*Address offset: 0x04*/
	__vo uint32_t RTSR;					/*Address offset: 0x08*/
	__vo uint32_t FTSR;					/*Address offset: 0x0C*/
	__vo uint32_t SWIER;				/*Address offset: 0x10*/
	__vo uint32_t PR;					/*Address offset: 0x14*/
}EXTI_RegDef_t;

#define EXTI                			((EXTI_RegDef_t*)EXTI_BASEADD)





/*
 * Peripheral register definition structure for SYSCFG
 *
 * */
typedef struct
{
	__vo uint32_t MEMRMP;               	 /*Address offset: 0x00*/
	__vo uint32_t PMC;					    /*Address offset: 0x04*/
	__vo uint32_t EXTICR[4];				/*Address offset: 0x08 - 0x14*/
	__vo uint32_t CMPCR;				   /*Address offset: 0x20*/

}SYSCFG_RegDef_t;

#define SYSCFG                        ((SYSCFG_RegDef_t*)SYSCFG_BASEADD)







/*
 * Peripheral register definition structure for SPI
 *
 * */
typedef struct
{
	__vo uint32_t CR1;               	    /*Address offset: 0x00*/
	__vo uint32_t CR2;					    /*Address offset: 0x04*/
	__vo uint32_t SR;				        /*Address offset: 0x08*/
	__vo uint32_t DR;				       /*Address offset: 0x20*/
	__vo uint32_t CRCPR;				   /*Address offset: 0x20*/
	__vo uint32_t RXCRCR;				   /*Address offset: 0x20*/
	__vo uint32_t TXCRCR;				   /*Address offset: 0x20*/
	__vo uint32_t I2SCFGR;				   /*Address offset: 0x20*/
	__vo uint32_t I2SPR;				   /*Address offset: 0x20*/
}SPI_RegDef_t;

#define SPI1                        ((SPI_RegDef_t*)SPI1_BASEADD)
#define SPI2                        ((SPI_RegDef_t*)SPI2_BASEADD)
#define SPI3                        ((SPI_RegDef_t*)SPI3_BASEADD)



/*******************************Bit position macros of SPI_CR1 peripheral********************************************/
#define SPI_CR1_CPHA          0
#define SPI_CR1_CPOL          1
#define SPI_CR1_MSTR          2
#define SPI_CR1_BR            3
#define SPI_CR1_SPE           6
#define SPI_CR1_LSBFIRST      7
#define SPI_CR1_SSI      	  8
#define SPI_CR1_SSM      	  9
#define SPI_CR1_RXONLY       10
#define SPI_CR1_DFF          11
#define SPI_CR1_CRCNEXT      12
#define SPI_CR1_CRCEN        13
#define SPI_CR1_BIDIOE       14
#define SPI_CR1_BIDIMODE     15


/*******************************Bit position macros of SPI_CR2 peripheral***********************************************/

#define SPI_CR2_RXMAED          0
#define SPI_CR2_TXMAED          1
#define SPI_CR2_SSOE            2
#define SPI_CR2_FRF             4
#define SPI_CR2_ERRIE           5
#define SPI_CR2_RXNEIE          6
#define SPI_CR2_TXNEIE          7




#define SPI_SR_RXNE            0
#define SPI_SR_TXE             1
#define SPI_SR_CHSIDE          2
#define SPI_SR_UDR             3
#define SPI_SR_CRCERR          4
#define SPI_SR_MODF            5
#define SPI_SR_OVR             6
#define SPI_SR_BSY             7
#define SPI_SR_FRE             8



/********************************Clock Enable Macros********************************/


/*
 *  Clock enable for macros GPIOx peripherals
 *
*/
#define GPIOA_PLCK_EN()   ( RCC->AHB1ENR |= (1 << 0) )
#define GPIOB_PLCK_EN()   ( RCC->AHB1ENR |= (1 << 1) )
#define GPIOC_PLCK_EN()   ( RCC->AHB1ENR |= (1 << 2) )
#define GPIOD_PLCK_EN()   ( RCC->AHB1ENR |= (1 << 3) )



/*
 *  Clock enable for macros I2Cx peripherals
*/
#define I2C1_PLCK_EN()      ( RCC->AHB1ENR |= (1 << 21) )



/*
 *  Clock enable for macros SPIx peripherals
*/
#define SPI1_PLCK_EN()      ( RCC->APB1ENR |= (1 << 12) )
#define SPI2_PLCK_EN()      ( RCC->APB1ENR |= (1 << 14) )
#define SPI3_PLCK_EN()      ( RCC->APB1ENR |= (1 << 15) )
#define SPI4_PLCK_EN()      ( RCC->APB1ENR |= (1 << 13) )



/*
 *  Clock enable for macros USARTx peripherals
*/
#define USART1_PLCK_EN()   ( RCC->APB2ENR |= (1 << 12) )





/*
 *  Clock enable for macros SYSCFG
*/
#define SYSCFG_PLCK_EN()    (RCC->APB2ENR |= (1<< 14))




/*
 *  Clock Disable for macros GPIOx peripherals
*/






#define GPIO_BASEADD_TO_CODE(x)    ( (x == GPIOA) ? 0: \
									 (x == GPIOB) ? 1: \
									 (x == GPIOC) ? 2: \
									 (x == GPIOD) ? 3: \
									 (x == GPIOE) ? 4: \
									 (x == GPIOF) ? 5: \
									 (x == GPIOG) ? 6:0  )




#define IRQ_NO_EXTI0         6
#define IRQ_NO_EXTI1         7
#define IRQ_NO_EXTI2         8
#define IRQ_NO_EXTI3         9
#define IRQ_NO_EXTI4         10
#define IRQ_NO_EXTI9_5       23
#define IRQ_NO_EXTI15_10     40




//some generic macros
#define ENABLE 	1
#define DISABLE 0
#define SET   ENABLE
#define RESET DISABLE
#define GPIO_PIN_SET SET
#define GPIO_PIN_RESET RESET
#define FLAG_RESET     RESET
#define FLAG_SET       SET


#endif /* STM32F407XX_H_ */
