/**
 ******************************************************************************
 * @file           : configurePin.h
 * @brief          : configurePin header file
 * @author		   : Matthew P.
 ******************************************************************************
 * @attention
 *
 * Description:
 * 	INPUTS:
 *    1. pin       (integer representation, not a bitmask)
 *    2. GPIO_port (GPIO[A-G])
 *    3. MODE      (0=input, 1=output, 2=alternate function, 3=analog)
 *    4. OTYPE     (0=push-pull, 1=open-drain)
 *    5. OSPEED    (0=low, 1=medium, 2=high, 3=very high)
 *    6. PUPD      (0=none, 1=pull-up, 2=pull-down, 3=reserved)
 *
 ******************************************************************************
 */

#ifndef SRC_CONFIGUREPIN_H_

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Definitions ---------------------------------------------------------------*/
#define SRC_CONFIGUREPIN_H_
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

/* Private function prototypes -----------------------------------------------*/
void configurePin(uint8_t pin, GPIO_TypeDef *GPIO_port,
        uint8_t MODE, uint8_t OTYPE,uint8_t OSPEED,uint8_t PUPD);

#endif /* SRC_CONFIGUREPIN_H_ */
