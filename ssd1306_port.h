/*
 * ssd1306_port.h
 *
 *  Created on: 22-Feb-2024
 *      Author: lenovo
 */

#ifndef SSD1306_PORT_H_
#define SSD1306_PORT_H_
#ifdef __cplusplus
extern "C" {
#endif

/************************************
 * INCLUDES
 ************************************/
#include "cyhal.h"
#include "cybsp.h"
/************************************
 * MACROS AND DEFINES
 ************************************/
#define SSD1306_DC_Port         DIS_DC_GPIO_Port
#define SSD1306_DC_Pin          DIS_DC_Pin

#define SSD1306_Reset_Port         DIS_RST_GPIO_Port
#define SSD1306_Reset_Pin          DIS_RST_Pin

#define SSD1306_SPI_PORT        hspi4
/************************************
 * TYPEDEFS
 ************************************/

/************************************
 * EXPORTED VARIABLES
 ************************************/

/************************************
 * GLOBAL FUNCTION PROTOTYPES
 ************************************/
void ssd1306_Reset(void);
void ssd1306_WriteCommand(uint8_t byte);
void ssd1306_WriteData(uint8_t* buffer, size_t buff_size);

#ifdef __cplusplus
}
#endif




#endif /* SSD1306_PORT_H_ */
