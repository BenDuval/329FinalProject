/*
 * ADC.h
 *
 *  Created on: Nov 17, 2023
 *      Author: Ben Duval
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_
#include <stdint.h>
void ADC_init(void);

extern  uint16_t ADCValues[20]; // Array to hold 20 samples
extern  uint8_t CurrentSample; // Index of the current sample
extern  uint8_t ADCConversionComplete; // Flag for conversion complete
void ADC1_2_IRQHandler(void);
#endif /* INC_ADC_H_ */
