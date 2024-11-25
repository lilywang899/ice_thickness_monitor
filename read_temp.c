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
#include "usart.h"
#include "gpio.h"
#include <string.h>
#include <stdio.h>

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
#define DHT11_GPIO_PORT GPIOA
#define DHT11_GPIO_PIN GPIO_PIN_0
uint8_t data[5];

//#include "stm32f4xx_hal.h" // Replace with your specific STM32 series HAL header

UART_HandleTypeDef huart2;

// Function to initialize UART2
void UART2_Init(void) {
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;  // PA2: TX, PA3: RX
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);
}


// function to read data from DHT11
void DHT11_ReadData(float *temperature, float *humidity)
{
    uint8_t i;

    // send start signal
    HAL_GPIO_WritePin(DHT11_GPIO_PORT, DHT11_GPIO_PIN, GPIO_PIN_RESET);
    HAL_Delay(18);
    HAL_GPIO_WritePin(DHT11_GPIO_PORT, DHT11_GPIO_PIN, GPIO_PIN_SET);

    // wait for response
    HAL_Delay(40);

    // initialize data array
    memset(data, 0, sizeof(data));

    // read 40 bits of data
    for(i=0; i<40; i++)
    {
        // wait for low pulse
        while(!HAL_GPIO_ReadPin(DHT11_GPIO_PORT, DHT11_GPIO_PIN));
//        HAL_UART_Transmit(&huart2, (uint8_t *)"waiting for LOW\r\n", 16, HAL_MAX_DELAY);

        // wait for high pulse
        uint32_t t = 0;
        while(HAL_GPIO_ReadPin(DHT11_GPIO_PORT, DHT11_GPIO_PIN))
        {
            t++;
            HAL_Delay(1);
            //HAL_UART_Transmit(&huart2, (uint8_t *)"Timeout waiting for HIGH\r\n", 25, HAL_MAX_DELAY);
            if (t>3){
            	break;
            }

        }
        HAL_UART_Transmit(&huart2, (uint8_t *)"Step X reached\r\n", 16, HAL_MAX_DELAY);

        // store bit value in data array
        if(t > 30)
            data[i/8] |= (1 << (7 - (i % 8)));
    }

    // verify checksum
    if(data[4] == (data[0] + data[1] + data[2] + data[3]))
    {
        // convert temperature and humidity values
        *humidity = (data[0] << 8 | data[1]) / 10.0;
        *temperature = ((data[2] & 0x7F) << 8 | data[3]) / 10.0;
        if (data[2] & 0x80) *temperature *= -1;
    }

}
//void DHT11_ReadData(float *temperature, float *humidity)
//{
//    uint8_t i;
//    uint32_t timeout;
//    GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//    // Set pin as output
//    GPIO_InitStruct.Pin = DHT11_GPIO_PIN;
//    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//    HAL_GPIO_Init(DHT11_GPIO_PORT, &GPIO_InitStruct);
//
//    // Send start signal
//    HAL_GPIO_WritePin(DHT11_GPIO_PORT, DHT11_GPIO_PIN, GPIO_PIN_RESET);
//    HAL_Delay(18); // Keep the pin low for at least 18ms
//    HAL_GPIO_WritePin(DHT11_GPIO_PORT, DHT11_GPIO_PIN, GPIO_PIN_SET);
//    HAL_Delay(40); // Keep the pin high for 20-40µs
//
//    // Set pin as input to read data
//    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//    HAL_GPIO_Init(DHT11_GPIO_PORT, &GPIO_InitStruct);
//
//    // Wait for DHT11 response
//    timeout = 1000;
//    while (!HAL_GPIO_ReadPin(DHT11_GPIO_PORT, DHT11_GPIO_PIN) && timeout--);
//    if (timeout == 0) return; // Timeout waiting for response
//
//    timeout = 1000;
//    while (HAL_GPIO_ReadPin(DHT11_GPIO_PORT, DHT11_GPIO_PIN) && timeout--);
//    if (timeout == 0) return; // Timeout waiting for response
//
//    // Initialize data array
//    memset(data, 0, sizeof(data));
//
//    // Read 40 bits of data
//    for (i = 0; i < 40; i++) {
////        timeout = 1000;
////        while (!HAL_GPIO_ReadPin(DHT11_GPIO_PORT, DHT11_GPIO_PIN) && timeout--);
////        if (timeout == 0) return;
////
////        uint32_t t = 0;
////        //edit
////        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
////        HAL_GPIO_Init(DHT11_GPIO_PORT, &GPIO_InitStruct);
////        while (HAL_GPIO_ReadPin(DHT11_GPIO_PORT, DHT11_GPIO_PIN)) {
////        	HAL_UART_Transmit(&huart2, (uint8_t *)"Step X reached\r\n", 16, HAL_MAX_DELAY);
////            t++;
////            HAL_Delay(1); // Adjust timing if necessary
////        }
//    	uint32_t timeout = 1000;
//    	while (!HAL_GPIO_ReadPin(DHT11_GPIO_PORT, DHT11_GPIO_PIN) && timeout--) {
//    	    if (timeout == 0) {
//    	        HAL_UART_Transmit(&huart2, (uint8_t *)"Timeout waiting for LOW\r\n", 25, HAL_MAX_DELAY);
//    	        return; // Timeout reached, exit function
//    	    }
////    	    HAL_Delay(1);
//    	}
//
//    	timeout = 1000;
//    	uint32_t t = 0;
//    	while (HAL_GPIO_ReadPin(DHT11_GPIO_PORT, DHT11_GPIO_PIN) && timeout--) {
//    		t++;
//    	    if (timeout == 0) {
//    	        HAL_UART_Transmit(&huart2, (uint8_t *)"Timeout waiting for HIGH\r\n", 26, HAL_MAX_DELAY);
//    	        return; // Timeout reached, exit function
//    	    }
//    	    HAL_Delay(1);
//    	}
//
//    	HAL_UART_Transmit(&huart2, (uint8_t *)"Step X reached\r\n", 16, HAL_MAX_DELAY);
//
//
//        if (t > 30) {
//            data[i / 8] |= (1 << (7 - (i % 8)));
//        }
//    }
//
//    // Verify checksum
//    if (data[4] == (data[0] + data[1] + data[2] + data[3])) {
//        *humidity = (data[0] << 8 | data[1]) / 10.0;
//        *temperature = ((data[2] & 0x7F) << 8 | data[3]) / 10.0;
//        if (data[2] & 0x80) *temperature *= -1; // Handle negative temperatures
//    }
//}



int _write(int file, uintptr_t *data, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)data, len, HAL_MAX_DELAY);
    return len;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  float temperature;
  float humidity;


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
  UART2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  // read data from DHT11

	  uint8_t dataToSend[] = "0";
	  HAL_UART_Transmit(&huart2, dataToSend, sizeof(dataToSend) - 1, HAL_MAX_DELAY);

//	  char test_message[] = "50";
//	  HAL_UART_Transmit(&huart2, (uint8_t *)test_message, strlen(test_message), HAL_MAX_DELAY);
	  HAL_Delay(1000);
	  char result[50];
	  float num = 23.34;
	  sprintf(result, "%f", num);
	  printf("\n The string for the num is %s", result);
	  getchar();
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	  DHT11_ReadData(&temperature, &humidity);
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	  // print temperature and humidity values
	  printf("Temperature: %.1fC\r\n", temperature);
	  printf("Humidity: %.1f%%\r\n", humidity);
	  // wait for 1 second
	  //HAL_UART_Transmit(&huart2, (uint8_t *)50, sizeof(50), HAL_MAX_DELAY);

	  HAL_Delay(1000);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
 