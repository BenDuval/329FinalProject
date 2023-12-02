/**
 ******************************************************************************
 * @file           : configurePin.c
 * @brief          : configurePin source file
 ******************************************************************************
 * @attention
 *
 * Description: Contains the Matthew P. configurePin function to make
 * 				configuring pins much simpler.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "configurePin.h"

/*
 * Function: configurePin
 * --------------------
 *	Configures specified GPIO port according to inputs.
 *
 *  returns: nothing
 */
void configurePin(uint8_t pin, GPIO_TypeDef *GPIO_port,
                  uint8_t MODE, uint8_t OTYPE,uint8_t OSPEED,uint8_t PUPD)
{
  // set field to 0, shifting mask by pinNumber*fieldWidth
  GPIO_port->MODER   &= ~(GPIO_MODER_MODE0 << (pin * GPIO_MODER_MODE1_Pos));
  // OR in value ANDed with mask
  GPIO_port->MODER   |= (MODE & GPIO_MODER_MODE0) << (pin * GPIO_MODER_MODE1_Pos);
  GPIO_port->PUPDR   &= ~(GPIO_PUPDR_PUPD0 << (pin * GPIO_PUPDR_PUPD1_Pos));
  GPIO_port->PUPDR   |= (PUPD & GPIO_PUPDR_PUPD0) << (pin * GPIO_PUPDR_PUPD1_Pos);
  GPIO_port->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED0 << (pin * GPIO_OSPEEDR_OSPEED1_Pos));
  GPIO_port->OSPEEDR |= (OSPEED & GPIO_OSPEEDR_OSPEED0) << (pin * GPIO_OSPEEDR_OSPEED1_Pos);
  GPIO_port->OTYPER  &= ~(GPIO_OTYPER_OT0 << (pin * GPIO_OTYPER_OT1_Pos));
  GPIO_port->OTYPER  |= (OTYPE & GPIO_OTYPER_OT0) << (pin * GPIO_OTYPER_OT1_Pos);
}
