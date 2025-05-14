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
#include "Keypad4X4.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "fonts.h"
#include "ssd1306.h"

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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
extern char key;
char hold[4];
int flag = 0;

// Green LED on PA5
#define GREEN_LED_PORT GPIOA
#define GREEN_LED_PIN GPIO_PIN_5

// Red LED on PC13
#define RED_LED_PORT GPIOC
#define RED_LED_PIN GPIO_PIN_13
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE BEGIN 2 */
    SSD1306_Init();
    SSD1306_GotoXY (0,0);
    //SSD1306_Puts ("Voltage:", &Font_11x18, 1);
    SSD1306_Puts ("Enter code:", &Font_11x18, 1);
    SSD1306_GotoXY (0, 30);
    SSD1306_UpdateScreen();
    SSD1306_UpdateScreen();
    HAL_Delay (500);


        HAL_GPIO_WritePin (KC0_GPIO_Port, KC0_Pin, GPIO_PIN_SET);   // Pull the C0 LOW
        		HAL_GPIO_WritePin (KC1_GPIO_Port, KC1_Pin, GPIO_PIN_SET);   // Pull the C1 LOW
        		HAL_GPIO_WritePin (KC2_GPIO_Port, KC2_Pin, GPIO_PIN_SET);   // Pull the C2 LOW
        		HAL_GPIO_WritePin (KC3_GPIO_Port, KC3_Pin, GPIO_PIN_SET);




      SSD1306_GotoXY (0, 30);
      SSD1306_Puts ("4 ou 6?", &Font_11x18, 1);
      SSD1306_UpdateScreen();
      HAL_Delay (500);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  int cursor_x = 0;
  int etat = -1;
  int i = 0;
  int passwordLength = 0;
  char motDePasse[7] = "";
  char armMode;
  int timerEcoule = 0;

  volatile uint32_t start_time = 0;


  while (1)
  {
	 /* SSD1306_Init();
	      SSD1306_GotoXY (0,0);
	      //SSD1306_Puts ("Voltage:", &Font_11x18, 1);
	      SSD1306_Puts ("test:", &Font_11x18, 1);
	      SSD1306_GotoXY (0, 30);
	      SSD1306_UpdateScreen();
	      SSD1306_UpdateScreen();
	      HAL_Delay (500);
	  */
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	/* D10 to D7 as input pins for row 0 to row 3. D6 to D3 as output for column pins C1 to C3*/
	  //testing*****
	 /* if(flag){
	  	  key = Get_Key();
	  	  sprintf(hold, "%c", key);
	  	  HAL_UART_Transmit(&huart2, (uint8_t *)hold, strlen(hold), 100);
	  	  SSD1306_GotoXY (0, 30);
	  	  SSD1306_UpdateScreen();
	  	  SSD1306_Puts (hold, &Font_11x18, 1);
	  	  SSD1306_UpdateScreen();
	  	  HAL_Delay (500);
	  	  flag = 0;
	  } */


	  /*if(HAL_GPIO_ReadPin(GPIOA, SENSOR_Pin)){
		  start_time = HAL_GetTick();

		  SSD1306_GotoXY(0, 30);
		  	        		  	        		  	        	  // Display the error message
		   SSD1306_Puts("Detecte", &Font_11x18, 1);
		  	        		  	        	  	        	  // Update the screen
		  SSD1306_UpdateScreen();
		  HAL_Delay (500);

		  SSD1306_GotoXY(0, 30);
		  		  	        		  	        		  	        	  // Display the error message
		  		   SSD1306_Puts("            ", &Font_11x18, 1);
		  		  	        		  	        	  	        	  // Update the screen
		  		  SSD1306_UpdateScreen();
		  		  HAL_Delay (500);



		  		while(HAL_GetTick() - start_time <= 10000){

		  		}

		  		HAL_GPIO_WritePin(GPIOA, Son_Alarme_Pin, GPIO_PIN_SET);

		  		    // Wait for 2 seconds
		  		    HAL_Delay(2000);

		  		    // Turn off the buzzer
		  		  HAL_GPIO_WritePin(GPIOA, Son_Alarme_Pin, GPIO_PIN_RESET);

	  }*/


	  if (flag) {
	          //key = Get_Key(); // Get the key pressed

	          if(etat == -1){


	        	  if(key == '4'){
	        		  passwordLength = 4;
	        		  SSD1306_GotoXY(0, 30);
	        		  	        		  	        	  // Display the error message
	        		  SSD1306_Puts("               ", &Font_11x18, 1);
	        		  	        	  	        	  // Update the screen
	        		  SSD1306_UpdateScreen();
	        		  HAL_Delay (500);


	        		  etat = 0;
	        		  flag = 0;

	        	  }
	        	  else if(key == '6'){
	        		  passwordLength = 6;
	        		  SSD1306_GotoXY(0, 30);
	        		  	        		  	        	  // Display the error message
	        		  SSD1306_Puts("               ", &Font_11x18, 1);
	        		  	        	  	        	  // Update the screen
	        		  SSD1306_UpdateScreen();
	        		  HAL_Delay (500);


	        		  etat = 0;
	        		  flag = 0;

	        	  }

	          }
	          else if(etat == 0){
	        	  sprintf(hold, "%c", '*');
	        	  motDePasse[i] = key;
	        	  i++;

	        	  HAL_UART_Transmit(&huart2, (uint8_t *)hold, strlen(hold), 100); // Debug output via UART

	        	  // Display '*' at the current cursor position
	        	  SSD1306_GotoXY(cursor_x, 30); // Set position for the character
	        	  SSD1306_Puts(hold, &Font_11x18, 1);
	        	  SSD1306_UpdateScreen();

	        	  // Increment cursor_x to move to the next position
	        	  cursor_x += 12; // Adjust spacing as needed based on your font width

        	  	  HAL_Delay(500); // Small delay
	        	  flag = 0;       // Reset the flag

	        	  if(i == passwordLength){
	        		  etat = 1;
	        		  i =0;
	        		  cursor_x = 0;
	        	  }

	          }

	          else if(etat == 1){

	        	  SSD1306_GotoXY(0, 30);
	        	  // Display the error message
	        	  SSD1306_Puts("             ", &Font_11x18, 1);
	        	  // Update the screen
	        	  SSD1306_UpdateScreen();
	        	  //HAL_Delay(500);  // Small delay before continuing

	        	  if(key == 'A' || key == 'B'){
	        		  armMode = key;
	        		  etat = 2;
	        	  }
	        	  i = 0;
	        	  flag = 0;
	          }
	          else if(etat == 2){
	        	  sprintf(hold, "%c", key);


	        	  	        	  HAL_UART_Transmit(&huart2, (uint8_t *)hold, strlen(hold), 100); // Debug output via UART

	        	  	        	  // Display '*' at the current cursor position
	        	  	        	  SSD1306_GotoXY(cursor_x, 30); // Set position for the character
	        	  	        	  SSD1306_Puts(hold, &Font_11x18, 1);
	        	  	        	  SSD1306_UpdateScreen();

	        	  	        	  // Increment cursor_x to move to the next position
	        	  	        	  cursor_x += 12; // Adjust spacing as needed based on your font width

	        	          	  	  //HAL_Delay(500); // Small delay
	        	  	        	  //flag = 0;

	        	  if(key != motDePasse[i]){
	        		  etat = 1;
	        		  	  SSD1306_GotoXY (0, 30);
	        		      SSD1306_Puts ("Incorrecte", &Font_11x18, 1);
	        		      SSD1306_UpdateScreen();
	        		      HAL_Delay (500);
	        	  }

	        	  else if(i == (passwordLength -1)){
	        		  etat = 1;
	        		  cursor_x = 0;
	        		  SSD1306_GotoXY (0, 30);

	        		  if(armMode == 'A'){
	        		    SSD1306_Puts ("Arme", &Font_11x18, 1);

	        		    HAL_GPIO_WritePin(GPIOC, ledRouge_Pin, GPIO_PIN_SET);
	        		    HAL_GPIO_WritePin(GPIOC, ledVert_Pin, GPIO_PIN_RESET);

	        		    start_time = HAL_GetTick();

	        		   while(HAL_GetTick() - start_time <= 10000){}

	        		   while(1){

	        		    if(HAL_GPIO_ReadPin(GPIOA, SENSOR_Pin)){
	        		    	HAL_GPIO_WritePin(GPIOA, Son_Alarme_Pin, GPIO_PIN_SET);
	        		    	//timerEcoule = 1;
	        		    	break;
	        		    }
	        		   }

	        		  }
	        		  else if(armMode == 'B'){

	        			  HAL_GPIO_WritePin(GPIOC, ledVert_Pin, GPIO_PIN_SET);
	        			  HAL_GPIO_WritePin(GPIOC, ledRouge_Pin, GPIO_PIN_RESET);
	        			  HAL_GPIO_WritePin(GPIOA, Son_Alarme_Pin, GPIO_PIN_RESET);
	        			  SSD1306_Puts ("Desarme", &Font_11x18, 1);

	        		  }
	        	  }
	        	  SSD1306_UpdateScreen();
	              HAL_Delay (500);
	        	  i++;
	        	  flag = 0;

	          }
	          else if(etat == 3){

	          }
	          else{

	        	  sprintf(hold, "%c", key);

	        	  	          HAL_UART_Transmit(&huart2, (uint8_t *)hold, strlen(hold), 100); // Debug output via UART

	        	  	          // Display the key at the current cursor position
	        	  	          SSD1306_GotoXY(cursor_x, 30); // Set position for the character
	        	  	          SSD1306_Puts(hold, &Font_11x18, 1);
	        	  	          SSD1306_UpdateScreen();

	        	  	          // Move cursor to the right for the next character
	        	  	          cursor_x += 11; // Advance by character width
	        	  	          if (cursor_x >= 128) { // Reset position if at the end of the line
	        	  	              cursor_x = 0; // Start from the left again
	        	  	              SSD1306_Clear(); // Clear the screen
	        	  	          }

	        	  	          HAL_Delay(500); // Small delay
	        	  	          flag = 0;       // Reset the flag

	          }

	      }


	     //test
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */


  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, ledRouge_Pin|ledVert_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|Son_Alarme_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, KC0_Pin|KC3_Pin|KC1_Pin|KC2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ledRouge_Pin ledVert_Pin */
  GPIO_InitStruct.Pin = ledRouge_Pin|ledVert_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 Son_Alarme_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_5|Son_Alarme_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SENSOR_Pin */
  GPIO_InitStruct.Pin = SENSOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SENSOR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : KC0_Pin KC3_Pin KC1_Pin KC2_Pin */
  GPIO_InitStruct.Pin = KC0_Pin|KC3_Pin|KC1_Pin|KC2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : KR1_Pin */
  GPIO_InitStruct.Pin = KR1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(KR1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : KR3_Pin KR2_Pin */
  GPIO_InitStruct.Pin = KR3_Pin|KR2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : KR0_Pin */
  GPIO_InitStruct.Pin = KR0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(KR0_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	flag = 1;

	if (GPIO_Pin == KR0_Pin || GPIO_Pin == KR1_Pin || GPIO_Pin == KR2_Pin || GPIO_Pin == KR3_Pin) {
	        //COLUMN 3
		HAL_GPIO_WritePin (KC0_GPIO_Port, KC0_Pin, GPIO_PIN_SET);     // Pull the C0 HIGH
		HAL_GPIO_WritePin (KC1_GPIO_Port, KC1_Pin, GPIO_PIN_RESET);   // Pull the C1 LOW
		HAL_GPIO_WritePin (KC2_GPIO_Port, KC2_Pin, GPIO_PIN_RESET);   // Pull the C2 LOW
		HAL_GPIO_WritePin (KC3_GPIO_Port, KC3_Pin, GPIO_PIN_RESET);   // Pull the C3 LOW


	        if (HAL_GPIO_ReadPin(KR0_GPIO_Port, KR0_Pin)){
	        	key = '1';
	        }
	        else if (HAL_GPIO_ReadPin(KR1_GPIO_Port, KR1_Pin)){
	        	key = '4';
	        }
	        else if (HAL_GPIO_ReadPin(KR2_GPIO_Port, KR2_Pin)){
	        	key = '7';
	        }
	        else if (HAL_GPIO_ReadPin(KR3_GPIO_Port, KR3_Pin)){
	        	key = '*';
	        }


	        //COLUMN 2
	        	HAL_GPIO_WritePin (KC0_GPIO_Port, KC0_Pin, GPIO_PIN_RESET);   // Pull the C0 LOW
	        	HAL_GPIO_WritePin (KC1_GPIO_Port, KC1_Pin, GPIO_PIN_SET);     // Pull the C1 HIGH
	        	HAL_GPIO_WritePin (KC2_GPIO_Port, KC2_Pin, GPIO_PIN_RESET);   // Pull the C2 LOW
	        	HAL_GPIO_WritePin (KC3_GPIO_Port, KC3_Pin, GPIO_PIN_RESET);   // Pull the C3 LOW

	        if (HAL_GPIO_ReadPin(KR0_GPIO_Port, KR0_Pin)){
	        	key = '2';
	        }
	        else if (HAL_GPIO_ReadPin(KR1_GPIO_Port, KR1_Pin)){
	        	key = '5';
	        }
	        else if (HAL_GPIO_ReadPin(KR2_GPIO_Port, KR2_Pin)){
	        	key = '8';
	        }
	        else if (HAL_GPIO_ReadPin(KR3_GPIO_Port, KR3_Pin)){
	        	key = '0';
	        }

	        //COLUMN 1
	        	HAL_GPIO_WritePin (KC0_GPIO_Port, KC0_Pin, GPIO_PIN_RESET);  // Pull the C0 LOW
	        	HAL_GPIO_WritePin (KC1_GPIO_Port, KC1_Pin, GPIO_PIN_RESET);  // Pull the C1 LOW
	        	HAL_GPIO_WritePin (KC2_GPIO_Port, KC2_Pin, GPIO_PIN_SET);    // Pull the C2 HIGH
	        	HAL_GPIO_WritePin (KC3_GPIO_Port, KC3_Pin, GPIO_PIN_RESET);  // Pull the C3 LOW

	        if (HAL_GPIO_ReadPin(KR0_GPIO_Port, KR0_Pin)){
	        	key = '3';
	        }
	        else if (HAL_GPIO_ReadPin(KR1_GPIO_Port, KR1_Pin)){
	        	key = '6';
	        }
	        else if (HAL_GPIO_ReadPin(KR2_GPIO_Port, KR2_Pin)){
	        	key = '9';

	        }
	        else if (HAL_GPIO_ReadPin(KR3_GPIO_Port, KR3_Pin)){
	        	key = '#';
	        }

	        //COLUMN 0
	        	HAL_GPIO_WritePin (KC0_GPIO_Port, KC0_Pin, GPIO_PIN_RESET);   // Pull the C0 LOW
	        	HAL_GPIO_WritePin (KC1_GPIO_Port, KC1_Pin, GPIO_PIN_RESET);   // Pull the C1 LOW
	        	HAL_GPIO_WritePin (KC2_GPIO_Port, KC2_Pin, GPIO_PIN_RESET);   // Pull the C2 LOW
	        	HAL_GPIO_WritePin (KC3_GPIO_Port, KC3_Pin, GPIO_PIN_SET);     // Pull the C3 HIGH

	        if (HAL_GPIO_ReadPin(KR0_GPIO_Port, KR0_Pin)){
	        	key = 'A';
	        }
	        else if (HAL_GPIO_ReadPin(KR1_GPIO_Port, KR1_Pin)){
	        	key = 'B';
	        }
	        else if (HAL_GPIO_ReadPin(KR2_GPIO_Port, KR2_Pin)){
	        	key = 'C';
	        }
	        else if (HAL_GPIO_ReadPin(KR3_GPIO_Port, KR3_Pin)){
	        	key = 'D';
	        }

	        HAL_GPIO_WritePin (KC0_GPIO_Port, KC0_Pin, GPIO_PIN_SET);   // Pull the C0 LOW
	        		HAL_GPIO_WritePin (KC1_GPIO_Port, KC1_Pin, GPIO_PIN_SET);   // Pull the C1 LOW
	        		HAL_GPIO_WritePin (KC2_GPIO_Port, KC2_Pin, GPIO_PIN_SET);   // Pull the C2 LOW
	        		HAL_GPIO_WritePin (KC3_GPIO_Port, KC3_Pin, GPIO_PIN_SET);

	    }

  /* Prevent unused argument(s) compilation warning */
  //UNUSED(GPIO_Pin);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */


  /*if(GPIO_Pin == KR0_Pin && (HAL_GPIO_ReadPin (KC0_GPIO_Port, KC0_Pin))){ //&& (HAL_GPIO_ReadPin (KC0_GPIO_Port, KC0_Pin))
	  key = '1';
  }*/

	/*if(GPIO_Pin == KR0_Pin){

		if ((HAL_GPIO_ReadPin (KC0_GPIO_Port, KC0_Pin)))   // if R0 is HIGH
				{
					key = '1';
				}
		else if ((HAL_GPIO_ReadPin (KC1_GPIO_Port, KC1_Pin)))   // if R0 is HIGH
						{
							key = '4';
						}
		else if ((HAL_GPIO_ReadPin (KC2_GPIO_Port, KC2_Pin)))   // if R0 is HIGH
								{
									key = '7';
								}
		else if ((HAL_GPIO_ReadPin (KC3_GPIO_Port, KC3_Pin)))   // if R0 is HIGH
										{
											key = '*';
										}
		//key = '1';
	}*/
	/*else if(GPIO_Pin == KR1_Pin){
		if ((HAL_GPIO_ReadPin (KC0_GPIO_Port, KC0_Pin)))   // if R0 is HIGH
				{
					key = '2';
				}
		else if ((HAL_GPIO_ReadPin (KC1_GPIO_Port, KC1_Pin)))   // if R0 is HIGH
				{
				         key = '5';
				}
		else if ((HAL_GPIO_ReadPin (KC2_GPIO_Port, KC2_Pin)))   // if R0 is HIGH
				{
			key = '8';
										}
		else if ((HAL_GPIO_ReadPin (KC3_GPIO_Port, KC3_Pin)))   // if R0 is HIGH
				{
				key = '0';
				}
	}
	else if(GPIO_Pin == KR2_Pin){
			if ((HAL_GPIO_ReadPin (KC0_GPIO_Port, KC0_Pin)))   // if R0 is HIGH
					{
						key = '3';
					}
			else if ((HAL_GPIO_ReadPin (KC1_GPIO_Port, KC1_Pin)))   // if R0 is HIGH
							{
								key = '6';
							}
			else if ((HAL_GPIO_ReadPin (KC2_GPIO_Port, KC2_Pin)))   // if R0 is HIGH
									{
										key = '9';
									}
			else if ((HAL_GPIO_ReadPin (KC3_GPIO_Port, KC3_Pin)))   // if R0 is HIGH
											{
												key = '#';
											}
		}
	else if (GPIO_Pin == KR3_Pin){
			if ((HAL_GPIO_ReadPin (KC0_GPIO_Port, KC0_Pin)))   // if R0 is HIGH
					{
						key = 'A';
					}
			else if ((HAL_GPIO_ReadPin (KC1_GPIO_Port, KC1_Pin)))   // if R0 is HIGH
							{
								key = 'B';
							}
			else if ((HAL_GPIO_ReadPin (KC2_GPIO_Port, KC2_Pin)))   // if R0 is HIGH
									{
										key = 'C';
									}
			else if ((HAL_GPIO_ReadPin (KC3_GPIO_Port, KC3_Pin)))   // if R0 is HIGH
											{
												key = 'D';
											}
		}*/
}


/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
