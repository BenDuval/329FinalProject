/*
 * lcd.h
 *
 *  Created on: Oct 11, 2023
 *      Author: esteb
 */

#ifndef SRC_LCD_H_
#define SRC_LCD_H_
#include "main.h"
#include <stdint.h>

// Define constants and macros
#define LCD_PORT GPIOC
#define LCD_EN GPIO_PIN_2
#define LCD_DATA_BITS (GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 )
#define LCD_RS GPIO_PIN_0
#define LCD_RW GPIO_PIN_1



#define CONFIG_MODE_INPUT 0
#define CONFIG_MODE_OUTPUT 1
#define CONFIG_MODE_ALT 2
#define CONFIG_MODE_ANALOG 3
#define CONFIG_TYPE_PUSHPULL 0
#define CONFIG_TYPE_OPENDRAIN 1
#define CONFIG_SPEED_LOW 0
#define CONFIG_SPEED_MED 1
#define CONFIG_SPEED_HIGH 2
#define CONFIG_SPEED_VHIGH 3
#define CONFIG_PUPD_NONE 0
#define CONFIG_PUPD_PULLUP 1
#define CONFIG_PUPD_PULLDOWN 2



// Function prototypes
void LCD_init(void);   	              // initialize LCD
void LCD_command(uint8_t command);   // send LCD a single 8-bit command
void LCD_write_char(uint8_t letter); // write a character to the LCD
void SysTick_Init(void);
void delay_us(const uint32_t time_us);
void LCD_pulse_ENA( void );
void LCD_4b_command( uint8_t command );
void delay_ms(uint32_t time_ms);
void delay_seconds(uint32_t seconds);

#endif /* SRC_LCD_H_ */
