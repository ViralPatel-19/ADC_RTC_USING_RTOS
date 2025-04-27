/*
 * ssd1306_fonts.h
 *
 *  Created on: 22-Feb-2024
 *      Author: lenovo
 */

#ifndef SSD1306_FONTS_H_
#define SSD1306_FONTS_H_
#ifdef __cplusplus
extern "C" {
#endif

/************************************
 * INCLUDES
 ************************************/
#include <stdint.h>
/************************************
 * MACROS AND DEFINES
 ************************************/

#define SSD1306_INCLUDE_FONT_6x8
#define SSD1306_INCLUDE_FONT_11x18
#define SSD1306_INCLUDE_FONT_16x26
#define SSD1306_INCLUDE_FONT_16x16
#define SSD1306_INCLUDE_FONT_7x10
//#define SSD1306_INCLUDE_FONT_16x24

/************************************
 * TYPEDEFS
 ************************************/
typedef struct {
	const uint8_t FontWidth;    /*!< Font width in pixels */
	uint8_t FontHeight;   /*!< Font height in pixels */
	const uint16_t *data; /*!< Pointer to data font data array */
} FontDef;
/************************************
 * EXPORTED VARIABLES
 ************************************/
#ifdef SSD1306_INCLUDE_FONT_6x8
extern FontDef Font_6x8;
#endif
#ifdef SSD1306_INCLUDE_FONT_7x10
extern FontDef Font_7x10;
#endif
#ifdef SSD1306_INCLUDE_FONT_11x18
extern FontDef Font_11x18;
#endif
#ifdef SSD1306_INCLUDE_FONT_16x26
extern FontDef Font_16x26;
#endif
#ifdef SSD1306_INCLUDE_FONT_16x24
extern FontDef Font_16x24;
#endif

#ifdef SSD1306_INCLUDE_FONT_7x10
extern FontDef Font_7x10;
#endif

#ifdef SSD1306_INCLUDE_FONT_16x16
extern FontDef Font_16x16;
#endif


extern FontDef Font_20x32 ;
extern const uint8_t comic_sans_font24x32_123[];
extern const uint8_t comic_sans_font24x32_dot[];
extern const uint8_t ssd1306xled_font8x16[];
extern const uint8_t Vengeance17x16[];
extern const uint8_t LiberationSans_Bold17x16[];

extern const uint8_t FuturaCondensedMedium21x32[];
extern const uint8_t Dustismo_Roman_Bold23x32[];
extern const uint8_t Targa_MS16x32[] ;
extern const uint8_t Targa15x32[] ;
extern const uint8_t FuturaCondensedMedium27x32[] ;
extern const uint8_t Dustismo_Roman_Bold27x32[] ;
extern const uint8_t Dustismo_Roman_Bold24x32[] ;
extern const uint8_t Bignoodletitling24x32[] ;

extern const uint8_t Vera17x16[] ;
/************************************
 * GLOBAL FUNCTION PROTOTYPES
 ************************************/

#ifdef __cplusplus
}
#endif




#endif /* SSD1306_FONTS_H_ */
