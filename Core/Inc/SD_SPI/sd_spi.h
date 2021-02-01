/*
 * sd_spi.h
 *
 *  Created on: 26.11.2017
 *      Author: jaras
 */

#ifndef SD_SPI_H_
#define SD_SPI_H_

#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_gpio.h"
#include "main.h"

#define SDSPI_CSPORT CS_SD_GPIO_Port
#define SDSPI_CSPIN CS_SD_Pin

uint8_t SDSPI_Init(SPI_HandleTypeDef *phandle);
uint8_t SDSPI_ReadInfo(SPI_HandleTypeDef *phandle, uint16_t *sector, uint32_t *capacity);
uint8_t SDSPI_ReadBlock(SPI_HandleTypeDef *phandle, uint32_t lba, uint8_t *buf, uint16_t size);
uint8_t SDSPI_WriteBlock(SPI_HandleTypeDef *phandle, uint32_t lba, uint8_t *buf, uint16_t size);


#endif /* SD_SPI_H_ */
