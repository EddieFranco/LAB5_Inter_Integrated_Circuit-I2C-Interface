/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* 5.2 Setting the GPIO Modes */
	
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // Enable GPIOB in the RCC
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Enable GPIOC in the RCC
	
	//Set PB11 to alternate function mode
	GPIOB->MODER &= ~(GPIO_MODER_MODER11); // clearing bits 
	// Set the selected pins into alternate function mode
	GPIOB->MODER |= GPIO_MODER_MODER11_1 ; 
	// Set the pin to open-drain output type in the OTYPER register
	GPIOB->OTYPER |= GPIO_OTYPER_OT_11 ;
	//select I2C2_SDA as its alternate function.
	GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL11); // Clear alternate function bits 
	GPIOB->AFR[1] |= (1 <<GPIO_AFRH_AFSEL11_Pos);
	
	//Set PB13 to alternate function mode
	GPIOB->MODER &= ~(GPIO_MODER_MODER13); // clearing bits 
	// Set the selected pins into alternate function mode
	GPIOB->MODER |= GPIO_MODER_MODER13_1 ; 
	// Set the pin to open-drain output type in the OTYPER register
	GPIOB->OTYPER |= GPIO_OTYPER_OT_13 ;
	//select I2C2_SDA as its alternate function.
	GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL13); // Clear alternate function bits 
	GPIOB->AFR[1] |= (0101<<GPIO_AFRH_AFSEL13_Pos);////////
	
	//Set PB14 to output mode, push-pull output type, and initialize/set the pin high.
	GPIOB->MODER|=(GPIO_MODER_MODER14_0);
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_14 );
	//Initialize one pin logic high.
	GPIOB->BSRR  = GPIO_BSRR_BS_14 ;
	
	//Set PC0 to output mode, push-pull output type, and initialize/set the pin high.
	GPIOC->MODER|=(GPIO_MODER_MODER0_0);
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_0 );
	//Initialize one pin logic high.
	GPIOC->BSRR  = GPIO_BSRR_BS_0 ;
	
  /* USER CODE END SysInit */
	
	/* 5.3 Initializing the I2C Peripheral */
	RCC->APB1ENR|= RCC_APB1ENR_I2C2EN; //Enable the I2C2 peripheral in the RCC
	//Set the parameters in the TIMINGR register to use 100 kHz standard-mode I2C.
	I2C2->TIMINGR |= (1<<I2C_TIMINGR_PRESC_Pos);
	I2C2->TIMINGR |= (0X13<<I2C_TIMINGR_SCLL_Pos );
	I2C2->TIMINGR |= (0XF<<I2C_TIMINGR_SCLH_Pos);
	I2C2->TIMINGR |= (0X2<<I2C_TIMINGR_SDADEL_Pos);
	I2C2->TIMINGR |= (0X4<<I2C_TIMINGR_SCLDEL_Pos);
	//Enable the I2C peripheral using the PE bit in the CR1 register.
	I2C2->CR1 = I2C_CR1_PE;
	
	/* USER CODE END SysInit */
	
	
	/* 5.4 Reading the Register */
	//Set the transaction parameters in the CR2 register.
	I2C2->CR2 = 0x6B;   // slave address = 0x6B
	I2C2->CR2 &= ~(I2C_CR2_NBYTES_Msk);
	I2C2->CR2 |= (1<<I2C_CR2_NBYTES_Pos); //Set the number of bytes to transmit = 1.
	I2C2->CR2 &= ~(I2C_CR2_RD_WRN_Pos);  // Set the RD_WRN bit to indicate a write operation
	I2C2->CR2 |= I2C_CR2_START_Pos; //Set the START bit.
	
	//Wait until either of the TXIS (Transmit Register Empty/Ready) or NACKF (Slave Not-
  // Acknowledge) flags are set.
	while (1) {
        if((I2C2->ISR & I2C_ISR_NACKF)!=0){
					break;
				}
				if((I2C2->ISR & I2C_ISR_TXIS)!=0){
					break;
				}
    }
	
		if(I2C2->ISR & I2C_ISR_TXIS){
			
		}
	
		
	
	/* USER CODE END SysInit */
	
  /* Initialize all configured peripherals */
	
	
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */





  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

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
