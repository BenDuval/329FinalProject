/*
 * lcd.c
 *
 *  Created on: Oct 11, 2023
 *      Author: esteb
 */

#include "lcd.h"

// --------------------------------------------------- delay.c w/o #includes ---
// configure SysTick timer for use with delay_us().
// warning: breaks HAL_delay() by disabling interrupts for shorter delay timing.
void SysTick_Init(void) {
	SysTick->CTRL |= (SysTick_CTRL_ENABLE_Msk |     	// enable SysTick Timer
                      SysTick_CTRL_CLKSOURCE_Msk); 	// select CPU clock
	SysTick->CTRL &= ~(SysTick_CTRL_TICKINT_Msk);  	// disable interrupt
}

// delay in microseconds using SysTick timer to count CPU clock cycles
// do not call with 0 : error, maximum delay.
// careful calling with small nums : results in longer delays than specified:
//	   e.g. @4MHz, delay_us(1) = 10=15 us delay.
void delay_us(const uint32_t time_us) {
	// set the counts for the specified delay
	SysTick->LOAD = (uint32_t)((time_us * (SystemCoreClock / 1000000)) - 1);
	SysTick->VAL = 0;                                  	 // clear timer count
	SysTick->CTRL &= ~(SysTick_CTRL_COUNTFLAG_Msk);    	 // clear count flag
	while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)); // wait for flag
}
void delay_ms(uint32_t time_ms) {
    for(uint32_t i = 0; i < time_ms; i++) {
        delay_us(1000); // delay for 1 millisecond
    }
}

void delay_seconds(uint32_t seconds) {
    for(uint32_t i = 0; i < seconds; i++) {
        delay_ms(1000); // delay for 1 second
    }
}
void LCD_init( void )  {   // RCC & GPIO config removed - leverage A1, A2 code
   delay_us( 400000 );                     // power-up wait 40 ms
   for ( int idx = 0; idx < 3; idx++ ) {  // wake up 1,2,3: DATA = 0011 XXXX
      LCD_4b_command( 0x30 );             // HI 4b of 8b cmd, low nibble = X

      delay_us( 5000 );
   }

   LCD_4b_command( 0x20 ); // fcn set #4: 4b cmd set 4b mode - next 0x28:2-line
   delay_us( 400 );         // remainder of LCD init removed - see LCD datasheets
   LCD_command (0x01);
   LCD_command(0x28);
   delay_us(1000);
   LCD_command(0x10);
   delay_us(1000);
   LCD_command(0x0F);
   delay_us(1000);
   LCD_command(0x01);
   delay_us(1500);
   LCD_command(0x06);
   delay_us(1000);

}

void LCD_pulse_ENA( void )  {
// ENAble line sends command on falling edge
// set to restore default then clear to trigger
   LCD_PORT->ODR   |= ( LCD_EN );         	// ENABLE = HI

   delay_us( 50 );                         // TDDR > 320 ns

   LCD_PORT->ODR   &= ~( LCD_EN );        // ENABLE = LOW
   delay_us( 50 );                         // low values flakey, see A3:p.1
}

void LCD_4b_command( uint8_t command )  {
// LCD command using high nibble only - used for 'wake-up' 0x30 commands
   LCD_PORT->ODR   &= ~( LCD_DATA_BITS ); 	// clear DATA bits
   LCD_PORT->ODR   |= ( command << 4 );   // DATA = command

   delay_us( 50 );

   LCD_pulse_ENA( );

}

void LCD_command( uint8_t command )  {
// send command to LCD in 4-bit instruction mode
// HIGH nibble then LOW nibble, timing sensitive
   LCD_PORT->ODR   &= ~( LCD_DATA_BITS );               // isolate cmd bits
   LCD_PORT->ODR   |= ( (command<<4) & LCD_DATA_BITS ); // shift higher bits to data: C*-C11
   delay_us( 50 );
   LCD_pulse_ENA( );                                    // latch HIGH NIBBLE

   LCD_PORT->ODR   &= ~( LCD_DATA_BITS );               // isolate cmd bits
   LCD_PORT->ODR   |= ( (command<<8) & LCD_DATA_BITS );      // shifts lower bits to data: C8 - C11
   delay_us( 50 );
   LCD_pulse_ENA( );                                    // latch LOW NIBBLE
}

void LCD_write_char( uint8_t letter )  {
// calls LCD_command() w/char data; assumes all ctrl bits set LO in LCD_init()
   LCD_PORT->ODR   |= (LCD_RS);       // RS = HI for data to address
   delay_us( 50 );
   LCD_command( letter );             // character to print
   LCD_PORT->ODR   &= ~(LCD_RS);      // RS = LO
}
