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
/* Includes ------------------------------------------------------------------*/
#include "keypad.h"
#include <stdint.h>
#include <stdbool.h>

// Settle time after columns are driven high
uint16_t SETTLE = 10;

// Implement the Keypad_Config function if needed
void Keypad_Config(void) {
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOCEN);
	// configure GPIO pins
	// input mode, push-pull, pull down, high speed
	for (int i = 0; i < 4; i++) {
		configurePin(i, ROW_PORT, CONFIG_MODE_INPUT, CONFIG_TYPE_PUSHPULL,
				CONFIG_SPEED_LOW, CONFIG_PUPD_PULLDOWN);
	}

	// configure GPIO pins
	// output mode, push-pull, no push up or pull down, high speed
	for (int i = 4; i < 7; i++) {
		configurePin(i, COL_PORT, CONFIG_MODE_OUTPUT, CONFIG_TYPE_PUSHPULL,
				CONFIG_SPEED_LOW, CONFIG_PUPD_NONE);
	}
}

/*
 * Function: Keypad_IsAnyKeyPressed
 * --------------------
 *	Drives all COLUMNS HI; sees if any ROWS are HI.
 *	Contains a debounce, which only considers a key pressed
 *	if the same state is detected for a (DEBOUNCE_THRESHOLD)
 *	number of consecutive reads.
 *
 *  returns: true - if a key is pressed
 *  		 false - if no key is pressed
 */
int Keypad_IsAnyKeyPressed(void) {
   COL_PORT->BSRR = COL_PINS;				// set all columns HI
   for ( uint16_t idx=0; idx<SETTLE; idx++ )   	// let it settle
      ;

   // Debounce mechanism
   // author: ChatGPT
   uint8_t consecutiveReads = 0;
   bool lastState = (ROW_PORT->IDR & ROW_PINS) != 0;

   while (consecutiveReads < DEBOUNCE_THRESHOLD) {
	   bool currentState = (ROW_PORT->IDR & ROW_PINS) != 0;

	   if (currentState == lastState) {
		   // If the current state is the same as the last state, increment the counter
		   consecutiveReads++;
	   } else {
		   // If the current state is different, reset the counter
		   consecutiveReads = 0;
	   }

	   // Update the last state for the next iteration
	   lastState = currentState;

	   // Introduce a delay between consecutive reads (adjust as needed)
	   for (volatile uint32_t delay = 0; delay < 10000; delay++);
   }

   if ((ROW_PORT->IDR & ROW_PINS) != 0 )        // got a keypress!
      return( TRUE );
   else
      return( FALSE );                          // nope.
}


int Keypad_WhichKeyIsPressed(void) {
// detect and encode a pressed key at {row,col}
// assumes a previous call to Keypad_IsAnyKeyPressed() returned TRUE
// verifies the Keypad_IsAnyKeyPressed() result (no debounce here),
// determines which key is pressed and returns the encoded key ID

   uint8_t iRow=0, iCol=0, iKey=0;  // keypad row & col index, key ID result
   uint8_t bGotKey = 0;             // bool for keypress, 0 = no press

   COL_PORT->BSRR = COL_PINS;                       	 // set all columns HI
   for ( iRow = 0; iRow < NUM_ROWS; iRow++ ) {      	 // check all ROWS
      if (((ROW_PORT->IDR) & (BIT0 << iRow)) != 0) {     // keypress in iRow!!
         COL_PORT->BRR = ( COL_PINS );            	    // set all cols LO
         for ( iCol = 0; iCol < NUM_COLS; iCol++ ) {   // 1 col at a time
            COL_PORT->BSRR = ( BIT0 << (4+iCol) );     // set this col HI
            if ((ROW_PORT->IDR & (BIT0 << iRow)) != 0)// keypress in iCol!!
            {
               bGotKey = 1;
               break;                                  // exit for iCol loop
            }
         }
         if ( bGotKey )
            break;
      }
   }
   if ( bGotKey ) {

      iKey = ( iRow * NUM_COLS ) + iCol + 1;       // handle numeric keys ...

 	   switch (iKey)
      {
         case (1):
            return(iKey);
         case (2):
            return(iKey);
         case (3):
            return(iKey);
         case (4):
            iKey = 4; //A
            return(iKey);
         case (5):
            iKey = 5;
            return(iKey);
         case (6):
            iKey = 6;
            return(iKey);
         case (7):
            iKey = 7;
            return(iKey);
         case (8):
            iKey = 8; // B
            return(iKey);
         case (9):
            iKey = 9;
            return(iKey);
         case (10):
            iKey = 10;//*
            return(iKey);
         case (11):
            iKey = 0; // 0
            return(iKey);
         case (12):
            iKey = 12; // #
            return(iKey);
         case (13):
            iKey = 11;
            return(iKey);
         case (14):
            iKey = 14;
            return(iKey);
         case (15):
            iKey = 15;
            return(iKey);
         case (16):
            iKey = 13;
            return(iKey);
         default:
            return( NO_KEYPRESS );
      }
   }
   else
   return( NO_KEYPRESS );
}
