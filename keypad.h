/**
 ******************************************************************************
 * @file           : keypad.c
 * @brief          : Keypad source file
 ******************************************************************************
 * @attention
 *
 * wiring    :
 * PC0 - CN9-3 - Row 0
 * PC1 - CN9-7 - Row 1
 * PC2 - CN10-9 - Row 2
 * PC3 - CN9-5 - Row 3
 * PC4 - CN9-9 - Col 0
 * PC5 - CN9-11 - Col 1
 * PC6 - CN7-1 - Col 2
 * Description: This contains the function for detecting any key presses and
 * 				the function for determining which key was pressed.
 ******************************************************************************
 */

#ifndef SRC_KEYPAD_H_
#define SRC_KEYPAD_H_

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "configurePin.h"

/* Definitions ---------------------------------------------------------------*/
#define COL_PORT GPIOB
#define ROW_PORT GPIOB
#define COL_PINS (GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6)
#define ROW_PINS (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 )
#define NUM_ROWS 4
#define NUM_COLS 3
#define TRUE 1
#define FALSE 0
#define BIT0 1
#define KEY_ZERO 11
#define CODE_ZERO 0
#define NO_KEYPRESS -1

#define DEBOUNCE_THRESHOLD 5  // Number of consecutive reads for a stable state

/* Private function prototypes -----------------------------------------------*/
void Keypad_Config(void);
int Keypad_IsAnyKeyPressed(void);
int Keypad_WhichKeyIsPressed(void);

#endif /* SRC_KEYPAD_H_ */


