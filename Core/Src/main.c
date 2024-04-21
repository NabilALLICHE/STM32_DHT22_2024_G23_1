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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "calcule.h" // Inclusion de la bibliothèque pour la communication avec le capteur
#include "delay.h" // Inclusion de la bibliothèque pour les retards en microsecondes
#include<lib_lcd.h>
#include"stdio.h"
#include <string.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
static rgb_lcd lcdData;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float Temperature = 0, Humidite = 0; // Variables pour stocker la température et l'humidité
uint16_t RH = 0, TEMP = 0; // Variables pour les valeurs brutes du capteur
uint8_t dataH1;
uint8_t dataH2;
uint8_t dataT1;
uint8_t dataT2;
uint8_t SUM;
uint8_t check;
char bufRH[30]; // Pour stocker la valeur de l'humidité
char bufT[30]; // Pour stocker la valeur de la température
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void UART_TransmitTemperature(float Temperature);
void UART_TransmitHummidity(float Humidite );
void UART_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void UART_TransmitTemperature(float Temperature ) {
        sprintf(bufT, "Temperature: %.2f C\t", Temperature);
        HAL_UART_Transmit(&huart2, (uint8_t*)bufT, strlen(bufT), HAL_MAX_DELAY);
        //HAL_UART_Transmit(&huart1, (uint8_t*)bufT, strlen(bufT), HAL_MAX_DELAY);
    }
    void UART_TransmitHummidity(float Humidite ) {
            sprintf(bufRH, "Humidity: %.2f %%\r\n", Humidite);
            HAL_UART_Transmit(&huart2, (uint8_t*)bufRH, strlen(bufRH), HAL_MAX_DELAY);
          //  HAL_UART_Transmit(&huart1, (uint8_t*)bufRH, strlen(bufRH), HAL_MAX_DELAY);
        }
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
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  lcd_init(&hi2c1, &lcdData); // Initialisation du LCD
   DWT_Delay_Init(); // Initialisation des retards

   lcd_position(&hi2c1,0,0); // Positionnement du curseur LCD
   lcd_print(&hi2c1,"Bonjour!!"); // Affichage d'un message sur le LCD
   reglagecouleur(0,0,255); // Réglage de la couleur

   HAL_Delay(3000); // Pause de 3 secondes

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	        /*commence la communication avec le capteur*/
	        	  //HAL_TIM_Base_Start(&htim2); // Démarre le timer (commenté)
	        	  	  HAL_Delay(1000); // Attend 1 seconde
	        	  	  Data_Output(GPIOA, GPIO_PIN_4); // Configure la broche GPIOA_Pin_4 en mode sortie (info vers le capteur)
	        	  	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // Met la broche GPIOA_Pin_4 à l'état bas (remet à l'état bas)
	        	  	  DWT_Delay_us(1200); // Attend un délai de 1200 µs (signal de commande > 1ms)
	        	  	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // Met la broche GPIOA_Pin_4 à l'état haut (état haut)
	        	  	  DWT_Delay_us(30); // Attend un délai de 30 µs (signal de commande)
	        	  	  Data_Input(GPIOA, GPIO_PIN_4); // Configure la broche GPIOA_Pin_4 en mode entrée (info vers le microcontrôleur)

	        	  	  /*commence la réception de données*/

	        	  	  while(!(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4))); // Attend que la broche GPIOA_Pin_4 soit à l'état haut

	        	  	  for (int k=0;k<1000;k++) // Boucle pour attendre jusqu'à 1000 fois
	        	  	  {
	        	  		  if (HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_4) == GPIO_PIN_RESET) // Vérifie si la broche GPIOA_Pin_4 est à l'état bas
	        	  		  {
	        	  	  	  	break; // Sort de la boucle si la condition est vraie
	        	  	  	  }
	        	  	  }

	        	  	  while(!(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4))); // Attend que la broche GPIOA_Pin_4 soit à l'état haut
	        	  	  DWT_Delay_us(40); // Attend un délai de 40 µs

	        	  	  Read_data(&dataH1); // Lit les données du capteur (dans la bibliothèque HT.c)
	        	  	  Read_data(&dataH2);
	        	  	  Read_data(&dataT1);
	        	  	  Read_data(&dataT2);
	        	  	  Read_data(&SUM);

	        	  	  check = dataH1 + dataH2 + dataT1 + dataT2; // Calcule la somme des données pour vérification

	        	  	 if(check == (SUM)) // Vérifie si la somme est égale au checksum
	        	  	  {

	        	  	  RH = (dataH1<<8) | dataH2; // Calcule l'humidité
	        	  	  TEMP = (dataT1<<8) | dataT2; // Calcule la température

	        	  	  }


	        	  	  Humidite = RH / 10.0; // Calcule l'humidité finale
	        	  	  Temperature = TEMP / 10.0; // Calcule la température finale

	        	  	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // Met la broche GPIOA_Pin_4 à l'état haut pour la prochaine lecture
	        	  	 clearlcd(); // Efface l'affichage LCD
	        	  		  	  sprintf(bufRH,"Humidite: %.1f", Humidite); // Formatage de l'humidité pour l'affichage
	        	  		  	  sprintf(bufT, "Temp.: %.1f C", Temperature); // Formatage de la température pour l'affichage
	        	  		  	 lcd_position(&hi2c1,1,0); // Positionne le curseur LCD
	        	  		  	  lcd_print(&hi2c1,bufRH); // Affiche l'humidité sur le LCD
	        	  		  	  lcd_print(&hi2c1,"%"); // Affiche le symbole "%" sur le LCD
	        	  		  	  lcd_position(&hi2c1,1,1); // Positionne le curseur LCD
	        	  		  	  lcd_print(&hi2c1,bufT); // Affiche la température sur le LCD
	        	  		  UART_TransmitTemperature( Temperature); // Transmet la température via UART
	        	  		 	  UART_TransmitHummidity( Humidite ); // Transmet l'humidité via UART
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
