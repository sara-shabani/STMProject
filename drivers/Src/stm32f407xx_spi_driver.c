/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Jul 4, 2023
 *      Author: sara
 */

#include "stm32f407xx_spi_driver.h"


void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t ENDi)
{

	if ( ENABLE == ENDi)
	{
		if ( SPI1 == pSPIx)
		{
			SPI1_PLCK_EN();
		}
		else if (SPI2 == pSPIx)
		{
			SPI2_PLCK_EN();
		}
		/*TODO : complete for other ports*/
	}
	else
	{
		if (SPI1 == pSPIx)
		{
			//GPIOA_PLCK_DI();
		}

	}

}



void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//1. configure the SPI_CR1 register

	uint32_t tempreg = 0;

	//1.1 configure the device mode
	tempreg |= pSPIHandle->SPI_Config.SPI_DeviceMode << 2;

	//2.2 configure the bus config
	if( SPI_BUS_CONFIG_FD == pSPIHandle->SPI_Config.SPI_BusConfig)
	{
			//bidi mode should be cleared
			tempreg &= ~(1<< 15);

	}else if(SPI_BUS_CONFIG_HD == pSPIHandle->SPI_Config.SPI_BusConfig)
	{
			//bidi mode should be set
			tempreg |= (1<< 15);

	}else if(SPI_BUS_CONFIG_SIMPLEX_RXONLY == pSPIHandle->SPI_Config.SPI_BusConfig)
	{
		//bidi should be cleared
		//RxONLY bit must be set
		tempreg &= ~(1<< 15);
		tempreg |= (1<< 10);
	}

   //3. configure the spi serial clock speed (baud rate)

	tempreg |= pSPIHandle->SPI_Config.SPI_SclkSpeed << SPI_CR1_BR;

   //4. configure DFF

	tempreg |= pSPIHandle->SPI_Config.SPI_DFF << 11;


   //5. configure CPOL
	tempreg |= pSPIHandle->SPI_Config.SPI_CPOL << 1;


  //6. configure CPHA

	tempreg |= pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA;

	tempreg |= pSPIHandle->SPI_Config.SPI_SSM << SPI_CR1_SSM;


	pSPIHandle->pSPIx->CR1 = tempreg;



}
void SPI_DeInit(SPI_RegDef_t *pSPIx)  //send back register to its reset state
{


}
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flagName)
{
	if(pSPIx->SR & flagName)
	{
		return FLAG_SET;

	}
	return FLAG_RESET;
}



void SPI_sendData(SPI_RegDef_t *pSPIx, uint8_t* pTxBuffer, uint32_t len)
{

	while(len > 0)
	{
		//1. wait until TXEA is set
		while ( SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG ) == FLAG_RESET);

		//2. check the DFF bit in CR1
		if ( pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			// 16 bit DFF

			//1. Load the data in to the data register (DR)
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			len --;
			len --;
			(uint16_t*)pTxBuffer++;
		}
		else
		{
			// 8 bit DFF
			//1. Load the data in to the data register (DR)
			pSPIx->DR = *pTxBuffer;
			len --;
			pTxBuffer++;
		}
	}

}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t* pRxBuffer, uint32_t len)
{


}


void SPI_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi)
{


}
void SPI_IRQITPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority)
{


}
void SPI_IRQHandling(SPI_Handle_t* pHandle)
{


}


void SPI_PheripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{

	if(ENABLE  == EnOrDi)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{

	if(ENABLE  == EnOrDi)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}


}
