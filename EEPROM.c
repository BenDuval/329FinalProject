///*
// * EEPROM.c
// *
// *  Created on: Nov 30, 2023
// *      Author: bcstu
// */
//
#include "EEPROM.h"
#include "stm32l4a6xx.h"
#include "main.h"
#include "LCD.h"
#include "configurePin.h"

void EEPROM_init(void);
void EEPROM_Write(uint16_t mem_address, int data);
uint16_t EEPROM_Read(uint16_t addr);

void EEPROM_init(void) {

//    // Enable the I2C bus clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOGEN;

    // Configure GPIO pins for I2C SCL and SDA with AF4
        configurePin(14, GPIOG, CONFIG_MODE_ALT, CONFIG_TYPE_OPENDRAIN, CONFIG_SPEED_HIGH, CONFIG_PUPD_PULLUP);
        configurePin(13, GPIOG, CONFIG_MODE_ALT, CONFIG_TYPE_OPENDRAIN, CONFIG_SPEED_HIGH, CONFIG_PUPD_PULLUP);
        // Configure PG14 for I2C1_SCL
        GPIOG->AFR[1] &= ~(0xF << ((14 - 8) * 4)); // Clear the previous value for PG14
        GPIOG->AFR[1] |= (0x4 << ((14 - 8) * 4));  // Set AF4 for PG14 (I2C1_SCL)

        // Configure PG13 for I2C1_SDA
        GPIOG->AFR[1] &= ~(0xF << ((13 - 8) * 4)); // Clear the previous value for PG13
        GPIOG->AFR[1] |= (0x4 << ((13 - 8) * 4));  // Set AF4 for PG13 (I2C1_SDA)

        I2C1->CR1   &= ~( I2C_CR1_PE );        // put I2C into reset (release SDA, SCL)
        I2C1->CR1   &= ~( I2C_CR1_ANFOFF );    // filters: enable analog
        I2C1->CR1   &= ~( I2C_CR1_DNF );       // filters: disable digital
        I2C1->TIMINGR = 0x00303D5B;            // 16 MHz SYSCLK timing from CubeMX
        I2C1->CR2   |=  ( I2C_CR2_AUTOEND );   // auto send STOP after transmission
        I2C1->CR2   &= ~( I2C_CR2_ADD10 );     // 7-bit address mode
        I2C1->CR1   |=  ( I2C_CR1_PE );        // enable I2C


}

void EEPROM_Write(uint16_t mem_address, int data) {

	// build EEPROM transaction
	I2C1->CR2   &= ~( I2C_CR2_RD_WRN );    // set WRITE mode
	I2C1->CR2   &= ~( I2C_CR2_NBYTES );    // clear Byte count
	I2C1->CR2   |=  ( 3 << I2C_CR2_NBYTES_Pos); // write 3 bytes (2 addr, 1 data)
	I2C1->CR2   &= ~( I2C_CR2_SADD );      // clear device address
	I2C1->CR2   |=  ( EEPROM_ADDRESS << (I2C_CR2_SADD_Pos+1) ); // device addr SHL 1
	I2C1->CR2   |=    I2C_CR2_START;       // start I2C WRITE op

	while(!(I2C1->ISR & I2C_ISR_TXIS));
	delay_ms(5);
	I2C1->TXDR = (mem_address >> 8); // Load MSByte of address into TXDR
	delay_ms(5);

	 while (!(I2C1->ISR & I2C_ISR_TXIS));
	 I2C1->TXDR = mem_address & 0xFF;
	 delay_ms(5);
	 while (!(I2C1->ISR & I2C_ISR_TXIS));
	 I2C1->TXDR = data;
	 delay_ms(5);
}


uint16_t EEPROM_Read(uint16_t addr) {
    uint16_t data = 0;

//    // Start condition and send the memory address to read from
//    I2C1->CR2 = (I2C1->CR2 & ~(I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_RD_WRN | I2C_CR2_AUTOEND))
//                | (EEPROM_ADDRESS << 1)     // Set EEPROM address and clear the R/W bit
//                | (2 << I2C_CR2_NBYTES_Pos) // Set number of bytes to transmit
//                | I2C_CR2_START;            // Generate START condition
    // Clear the necessary bits in CR2
    I2C1->CR2 &= ~(I2C_CR2_SADD);
    I2C1->CR2 &= ~(I2C_CR2_NBYTES);
    I2C1->CR2 &= ~(I2C_CR2_RD_WRN);
    I2C1->CR2 &= ~(I2C_CR2_AUTOEND);

    // Now set each bit or field individually
    I2C1->CR2 |= (EEPROM_ADDRESS << 1); // Set the EEPROM address
    I2C1->CR2 |= (2 << I2C_CR2_NBYTES_Pos); // Set number of bytes to receive
    I2C1->CR2 |= I2C_CR2_START;             // Generate the START condition

    while (!(I2C1->ISR & I2C_ISR_TXIS)); // isr goes busy
    I2C1->TXDR = (addr >> 8); // Send high byte of the memory address
    while (!(I2C1->ISR & I2C_ISR_TXIS));
    I2C1->TXDR = (addr & 0xFF); // Send low byte of the memory address

    // Wait for the end of the address transmission
    while (!(I2C1->ISR & I2C_ISR_TC));

    I2C1->CR1 &= ~( I2C_CR1_PE ); // reset I2C (release SDA, SCL) clears STOP bit
    delay_ms(5);
    I2C1->CR1 |=  ( I2C_CR1_PE ); 	// enable I2C

//    // Reconfigure CR2 for the read operation, 1 byte to read
//    I2C1->CR2 = (I2C1->CR2 & ~(I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_RD_WRN | I2C_CR2_AUTOEND))
//                | ((EEPROM_ADDRESS << 1))      // Set EEPROM address
//                | I2C_CR2_RD_WRN            // Read mode
//                | (1 << I2C_CR2_NBYTES_Pos) // Set number of bytes to receive
//                | I2C_CR2_START;            // Generate Re-START condition

    // Clear the necessary bits in CR2
    I2C1->CR2 &= ~(I2C_CR2_SADD);
    I2C1->CR2 &= ~(I2C_CR2_NBYTES);
    I2C1->CR2 &= ~(I2C_CR2_RD_WRN);
    I2C1->CR2 &= ~(I2C_CR2_AUTOEND);

    // Now set each bit or field individually
    I2C1->CR2 |= (EEPROM_ADDRESS << 1); // Set the EEPROM address
    I2C1->CR2 |= I2C_CR2_RD_WRN;        // Set the read mode
    I2C1->CR2 |= (1 << I2C_CR2_NBYTES_Pos); // Set number of bytes to receive
    I2C1->CR2 |= I2C_CR2_START;             // Generate the START condition

    // Wait for the data to be received
    while (!(I2C1->ISR & I2C_ISR_RXNE));
    data = I2C1->RXDR;

    // Wait for the transfer to complete
    while (!(I2C1->ISR & I2C_ISR_TC));

    // Send stop condition
    I2C1->CR2 |= I2C_CR2_STOP;

    // Wait for the stop flag to be set indicating a stop condition has been sent
    while (!(I2C1->ISR & I2C_ISR_STOPF));

    // Clear the stop flag for next operation
    I2C1->ICR |= I2C_ICR_STOPCF;

    return data;
}
