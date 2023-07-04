/*
 * 006spi_tx_testing.c
 *
 *  Created on: Jul 4, 2023
 *      Author: sara
 */


#include <stdint.h>
#include <string.h>
#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"

/*
 * PB14 --> SP2_MISO
 * PB15 --> SP2_MOSI
 * PB13 --> SP2_SCLK
 * PB12 --> SP2_NSS
 * ALT function mode : 5

 */

int main(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl  = GPIO_NO_PUPD;


	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber  = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber  = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);


	//MISO
//	SPIPins.GPIO_PinConfig.GPIO_PinNumber  = GPIO_PIN_NO_14;
//	GPIO_Init(&SPIPins);
//
//
//	//NSS
//	SPIPins.GPIO_PinConfig.GPIO_PinNumber  = GPIO_PIN_NO_12;
//	GPIO_Init(&SPIPins);


	///////////////////////////////////////////////////////////////////

	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2; //8 MHz
	SPI2Handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPI_Config.SPI_CPHA = SPI_CPOL_LOW;
	SPI2Handle.SPI_Config.SPI_SSM = SPI_SSM_EN; // Software slave management enable



	SPI_Init(&SPI2Handle);

	// This makes NSS signal internally high and avoids MODF error

	SPI_SSIConfig(SPI2, ENABLE);


	char userData[] = "Hello_world";


	SPI_PheripheralControl(SPI2, ENABLE);


	SPI_sendData(SPI2, (uint8_t*)userData, strlen(userData));

	while(1);


	return 0;

}
