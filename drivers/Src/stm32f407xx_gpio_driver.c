/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Jun 25, 2023
 *      Author: sara
 */

#include "stm32f407xx_gpio_driver.h"

/*
 * Peripheral clock setup
 *
 */
/****************************************************************
 * @fn     				- GPIO_PriclockCotrol
 *
 *
 * @brief				- This function enables and disables peripheral clock for the given GPIO port
 *
 * @param[in]			- base address of the GPIO peripheral
 * @param[in]			- ENABLE and DISABLE macros
 *
 * @return				- none
 *
 * @Note				- none
 *
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t ENDi)
{
	if ( ENABLE == ENDi)
	{
		if ( GPIOA == pGPIOx)
		{
			GPIOA_PLCK_EN();
		}
		else if (GPIOD == pGPIOx)
		{
			GPIOD_PLCK_EN();
		}
		else if (GPIOB == pGPIOx)
		{
			GPIOB_PLCK_EN();
		}
		/*TODO : complete for other ports*/
	}
	else
	{
		if (GPIOA == pGPIOx)
		{
			//GPIOA_PLCK_DI();
		}

	}

}
/*
 * Init and De-init
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;

	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//1. Configure the mode of GPIO pin
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{/*Start of: If the mode is less than analog , then it is non interrupt mode*/
			temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
			pGPIOHandle->pGPIOx->MODER |= temp; //setting
	}/*End of: The mode is less than analog , non interrupt is configured*/
	else
	{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//1.Configure FTSR
			EXTI->FTSR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;

			//Clear corresponding RTSR
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RI)
		{
			//1.COnfigure the RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			//Clear corresponding FTSR
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//1.COnfigure the RTSR and FTSR

			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}

		//2. Configure the GPIO port selection in SYSCFG_EXTICR

		//uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4;
		//uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%4;
		//uint8_t portcode = GPIO_BASEADD_TO_CODE(pGPIOHandle->pGPIOx);

		SYSCFG_PLCK_EN();

		//SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);


		//3. Enable the exti interrupt delivery using IMR(Interrupt mask register)
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	temp = 0;
	//2. Configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;
	//3. Configure pull up pull down resistor

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;
	//4. Configure output type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << ( pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp = 0;
	//5. Configure alternate functionality
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		// configure alternate functionality register
		uint8_t temp1, temp2;
		// Decide alternate high or low register, divide by 8
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber /8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber %8;
		pGPIOHandle->pGPIOx->AFRL[temp1] &= ~(0xF << (4 * temp2)); //clearing
		pGPIOHandle->pGPIOx->AFRL[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}

}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)  //send back register to its reset state
{

	if ( GPIOA == pGPIOx)
	{
		//GPIOA_REG_RESET();
	}
	else if (GPIOB == pGPIOx)
	{
		//GPIOA_REG_RESET();
	}


}
/****************************************************************
 * @fn     				- GPIO_PriclockCotrol
 *
 *
 * @brief				- This function Data read write
 *
 * @param[in]			- base address of the GPIO peripheral
 * @param[in]			- ENABLE and DISABLE macros
 *
 * @return				- 0 or 1
 *
 * @Note				- none
 *
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{

	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber ) & 0x00000001);
	return value;


}
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{

	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;

}
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}


}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{

	pGPIOx->ODR = Value;

}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{

	pGPIOx->ODR ^= (1 << PinNumber);

}

/*
 * IRQ Configuration and LSR handling
 */
void GPIO_IRQITConfig(uint8_t IRQNumber, uint8_t EnDi)
{
	if(ENABLE == EnDi)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			(*NVIC_ISER0) |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 &&  IRQNumber < 64)
		{
			//program ISER1 register
			(*NVIC_ISER1) |= (1 << (IRQNumber %32));
		}
		else if(IRQNumber >= 64 &&  IRQNumber < 96)
		{
			//program ISER2 register
			(*NVIC_ISER2) |= 1 << (IRQNumber %64);
		}
		else
		{
			if(IRQNumber <= 31)
			{
				//program ICER20 register
				(*NVIC_ICER0) |= 1 << IRQNumber;
			}
			else if(IRQNumber > 31 &&  IRQNumber < 64)
			{
				//program ICER1 register
				(*NVIC_ICER1) |= 1 << (IRQNumber %32);
			}
			else if(IRQNumber >= 64 &&  IRQNumber < 96)
			{
				//program ICER2 register
				(*NVIC_ICER2) |= 1 << (IRQNumber %64);
			}

		}

	}

}


void GPIO_IRQITPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority)
{
	//1. find out the irq register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber %4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx * 4) |= ( IRQPriority << shift_amount);

}

void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the exti pr register corresponding to the pn number
	if(EXTI->PR & (1 << PinNumber))
	{
		//clear
		EXTI->PR |= (1 << PinNumber);
	}

}

