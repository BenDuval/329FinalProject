/*
 * ADC.c
 *
 *  Created on: Nov 17, 2023
 *      Author: Ben Duval
 */
#include "ADC.h"
#include "stm32l4xx.h"
#include "LCD.h"
void ADC_init(void);

 uint16_t ADCValues[20]; // Array to hold 20 samples
 uint8_t CurrentSample = 0; // Index of the current sample
 uint8_t ADCConversionComplete = 0; // Flag for conversion complete

void ADC_init(void) {
	// Enable clock for ADC
	RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;

	// Power up and calibrate ADC
	ADC123_COMMON->CCR |= (1 << ADC_CCR_CKMODE_Pos);  // Clock source = HCLK/1
	ADC1->CR &= ~(ADC_CR_DEEPPWD);                    // Disable deep-power-down
	ADC1->CR |= ADC_CR_ADVREGEN;                     // Enable voltage regulator
	delay_us(20);                                    // Wait for ADC to power up

	// Configure ADC channel
	ADC1->DIFSEL &= ~(ADC_DIFSEL_DIFSEL_5);        // PA0=ADC1_IN5, single-ended
	ADC1->CR &= ~(ADC_CR_ADEN | ADC_CR_ADCALDIF); // Disable ADC, single-end calibration
	ADC1->CR |= ADC_CR_ADCAL;                         // Start calibration
	while (ADC1->CR & ADC_CR_ADCAL)
		;                  // Wait for calibration to finish

	// Enable ADC
	ADC1->ISR |= ADC_ISR_ADRDY;                       // Clear ADC Ready flag
	ADC1->CR |= ADC_CR_ADEN;                          // Enable ADC
	while (!(ADC1->ISR & ADC_ISR_ADRDY))
		;             // Wait for ADC Ready flag

	// Configure ADC sampling and sequencing
	ADC1->SQR1 |= (5 << ADC_SQR1_SQ1_Pos); // Sequence = 1 conversion, channel 5

//	ADC1->SMPR1 &= ~(7 << ADC_SMPR1_SMP5_Pos); // Clear current setting
//	ADC1->SMPR1 |= (4 << ADC_SMPR1_SMP5_Pos); // Set sample time to 47.5 clock cycles

	ADC1->SMPR1 &= ~(7 << ADC_SMPR1_SMP5_Pos); // Clear current setting
	ADC1->SMPR1 |= (7 << ADC_SMPR1_SMP5_Pos); // Set sample time to 640.5 clock cycles


//	ADC1->SMPR1 |= (1 << ADC_SMPR1_SMP5_Pos); // Channel 5 sample time = 6.5 clocks
	ADC1->CFGR &= ~(ADC_CFGR_CONT | ADC_CFGR_EXTEN | ADC_CFGR_RES); // Single conversion mode, 12-bit resolution

	// Configure and enable ADC interrupt
	ADC1->IER |= ADC_IER_EOCIE;            // Enable end-of-conversion interrupt
	NVIC->ISER[0] = (1 << (ADC1_2_IRQn & 0x1F)); // Enable ADC interrupt in NVIC
	__enable_irq();                                  // Enable global interrupts

	// Configure GPIO pin PA0 for ADC
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;              // Enable clock for GPIOA
	GPIOA->MODER |= GPIO_MODER_MODE0;                 // Set PA0 to analog mode
}

//void ADC1_2_IRQHandler() {
//	if (ADC1->ISR & ADC_ISR_EOC) {
//		// Store the ADC value in the array at the current index
//		ADCValues[CurrentSample] = ADC1->DR;
//
//		// Increment the index
//		CurrentSample++;
//
//		// Check if 20 samples have been collected
//		if (CurrentSample >= 20) {
//			CurrentSample = 0;  // Reset the index
//			ADCConversionComplete = 1; // Indicate that a batch of samples is ready
//		}
//
//		// Clear the EOC flag by writing 1 to it
//		ADC1->ISR |= ADC_ISR_EOC;
//	}
//}

void ADC1_2_IRQHandler() {
	if (ADC1->ISR & ADC_ISR_EOC) {
		// Store the ADC value in the array at the current index
		ADCValues[CurrentSample] = ADC1->DR;

		// Increment the index
		CurrentSample++;

		// Check if 50 samples have been collected
		if (CurrentSample >= 50) {
			CurrentSample = 0;  // Reset the index
			ADCConversionComplete = 1; // Indicate that a batch of samples is ready
		}

		// Clear the EOC flag by writing 1 to it
		ADC1->ISR |= ADC_ISR_EOC;
	}
}
