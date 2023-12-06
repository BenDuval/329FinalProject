/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 *
 * wiring    :
 * LCD Configuration
 * PC0 - RS
 * PC1 - R/W
 * PC2 - E
 * PC8 - DB4
 * PC9 - DB5
 * PC10 - DB6
 * PC11 - DB7
 * PC1 - DB8
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "LCD.h"
#include "configurePin.h"
#include "ADC.h"

#include "stdio.h"
#include <string.h>
#include "stm32l4xx.h"

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void LCD_display_sentence(const char *sentence);
void LCD_config(void);
void config_Piezo(void);
void lock_Box(void);
void led_on(uint16_t pin, GPIO_TypeDef *port);
void led_off(uint16_t pin, GPIO_TypeDef *port);


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN; // Enable LCD clock
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN; // Enabled Clock for Port B
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();


  /* Initialize all configured peripherals */

  /* USER CODE BEGIN 2 */
	led_off(7, GPIOB);
	led_off(14, GPIOB);
	led_off(7, GPIOC);
	program_State current_State; // defined current State to hold current state of program
	current_State = wait_State; // set current state to wait State
	ADC_init(); // Initialize ADC
	LCD_config(); // Configure LCD
	LCD_init(); // initialize LCD
	config_Piezo(); // configure Piezo for PB0

    // Configure PB5 as input with internal pull-up for reed sensor
    configurePin(5, GPIOB, CONFIG_MODE_INPUT, CONFIG_TYPE_PUSHPULL,
                 CONFIG_SPEED_LOW, CONFIG_PUPD_PULLUP);

    // Configure PC7, PB7, and PB14 as output
    configurePin(7, GPIOC, CONFIG_MODE_OUTPUT, CONFIG_TYPE_PUSHPULL,
                 CONFIG_SPEED_LOW, CONFIG_PUPD_NONE);

    configurePin(7, GPIOB, CONFIG_MODE_OUTPUT, CONFIG_TYPE_PUSHPULL,
                 CONFIG_SPEED_LOW, CONFIG_PUPD_NONE);

    configurePin(14, GPIOB, CONFIG_MODE_OUTPUT, CONFIG_TYPE_PUSHPULL,
                 CONFIG_SPEED_LOW, CONFIG_PUPD_NONE);

    //lock_Box(); // test for reed sensor

	switch (current_State) {
	case wait_State:
		LCD_command(0x02); // go home
		delay_us(50); // delay 50 microseconds
		LCD_display_sentence("Knock Lock Box:"); // Display start screen "Knock Lock Box"
		delay_us(50); // delay 50 microseconds
		LCD_display_sentence("Welcome"); // Display Welcome
		delay_ms(10); // delay 10 milliseconds
		// wait for something to transition states? first knock? turn on led to indicate state transition
		// if first knock detected, transition
		current_State = password_Check;
		break;
	case password_Check:
		led_on(7,GPIOB);
		LCD_command(0x01); // clear screen
		delay_us(50); // delay for 50 microseconds
		LCD_command(0x02); // go home
		delay_us(50); // delay for 50 microseconds
		LCD_display_sentence("Checking passwords...");
		// Check EEPROM for saved data?
		// if saved data, current_State = unlock state; else setPassword state;

		break;
	case set_Password:
		// begin process of setting password
		// wait for user to knock 1 time indicating the next knock will be start of password.
		// Check for first knock
		// start counter and stop on next knock
		// repeat 2x and store 3 times as password
		break;
	case unlock_State:
		led_on(7,GPIOB); // is this right? PB7 for user led?
		// the password is determined by the amount of time between knocks (with a percent of error allowed)
		// compare captured password against EEPROM stored password
		// if true, unlock box
		//unlock_Box();
		break;
	case lock_State:
		//lock_Box();
		break;
	case reset_password:
		// clear EEPROM
		current_State = set_Password;
		break;
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  }

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_8;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

void led_on(uint16_t pin, GPIO_TypeDef *port) {
	port->BSRR = (1 << pin);
}

void led_off(uint16_t pin, GPIO_TypeDef *port) {
	port->BSRR = (1 << (pin + 16));
}

// Function to display a sentence on the LCD
void LCD_display_sentence(const char *sentence) {
	for (int i = 0; i < strlen(sentence); ++i) {
		LCD_write_char(sentence[i]);
	}
}

void LCD_config(void) {
	// configure GPIO pins for LCD
	// output mode, push-pull,no pullup-pulldown, low speed
	for (int i = 0; i < 3; i++) {
		configurePin(i, LCD_PORT, CONFIG_MODE_OUTPUT,
		CONFIG_TYPE_PUSHPULL,
		CONFIG_SPEED_LOW,
		CONFIG_PUPD_NONE);
	}

	for (int i = 8; i < 12; i++) {
		configurePin(i, LCD_PORT, CONFIG_MODE_OUTPUT,
		CONFIG_TYPE_PUSHPULL,
		CONFIG_SPEED_LOW,
		CONFIG_PUPD_NONE);
	}
}

void unlock_Box(void) {
	// something special
}
void lock_Box(void) {
	while(1) {

	    // Read the state of PB5
	    if (GPIOB->IDR & (1 << 5)) {
	    	led_on(14,GPIOB);
	         //PB5 is HIGH, so the box is open
			LCD_command(0x01); // clear screen
			delay_us(50); // delay for 50 microseconds
			LCD_command(0x02); // go home
			delay_us(50); // delay for 50 microseconds
			LCD_display_sentence("Close box");
	    } else {
	    	led_on(14,GPIOB);
	        // PB5 is LOW, so the box is closed
	    	activateSolenoid();
	    	delay_ms(5);
	    	deactivateSolenoid();

	    }
	}
}


void comparator_init() {
    // Configure PB2 as analog input for COMP1_INP
    configurePin(2, GPIOB, CONFIG_MODE_ANALOG, CONFIG_TYPE_PUSHPULL, CONFIG_SPEED_LOW, CONFIG_PUPD_NONE);

    // Configure PB1 as analog input for COMP1_INM
    configurePin(1, GPIOB, CONFIG_MODE_ANALOG, CONFIG_TYPE_PUSHPULL, CONFIG_SPEED_LOW, CONFIG_PUPD_NONE);

    // Configure PB4 as analog input for COMP2_INP
    configurePin(4, GPIOB, CONFIG_MODE_ANALOG, CONFIG_TYPE_PUSHPULL, CONFIG_SPEED_LOW, CONFIG_PUPD_NONE);

    // Configure PB3 as analog input for COMP2_INM
    configurePin(3, GPIOB, CONFIG_MODE_ANALOG, CONFIG_TYPE_PUSHPULL, CONFIG_SPEED_LOW, CONFIG_PUPD_NONE);
}
void config_Solenoid(void) {
    // Configure PC6 as output to drive relay controlling solenoid
    configurePin(6, GPIOC, CONFIG_MODE_OUTPUT, CONFIG_TYPE_PUSHPULL,
    			CONFIG_SPEED_LOW, CONFIG_PUPD_NONE);
}

void activateSolenoid(void) {
	// Set PC6 high to activate the solenoid
	 GPIOC->BSRR = (1 << 6); // set PC6
}

void deactivateSolenoid(void) {
    // Set PC6 low to deactivate the solenoid
    GPIOC->BRR = (1 << 6); // Reset PC6
}
#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

