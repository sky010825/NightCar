/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32l0xx_hal.h"
#include <stdio.h>
#include <string.h>

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
ADC_HandleTypeDef hadc;


UART_HandleTypeDef huart2;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_USART5_UART_Init(void);
static void MX_USART4_UART_Init(void);
/* USER CODE BEGIN PFP */
//Server
volatile uint8_t Start_State = 0;
uint8_t rxData;


void Read_Sensors(void);
void Send_UART(char* msg);
void sendDFPlayerCommand(uint8_t command, uint16_t parameter);
void TransmitState();

float getSOCFromVoltage(float voltage);
float getSOHFromVoltage(float voltage);

char uart_buf[50];

volatile int uartserver_buf_len;
volatile int check[5]={0};


volatile int soh ,soc;


#define R1 1000.0f         
#define R2 (220.0f + 100.0f) 
#define ADC_MAX 4095.0f
#define VREF 3.0f 

uint16_t adc_raw;
float vin;
float vbat;

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
  MX_ADC_Init();
  MX_USART5_UART_Init();
  MX_USART4_UART_Init();
  /* USER CODE BEGIN 2 */
	
	
	
	
	
	
	
	sendDFPlayerCommand(0x06, 30); // Alarm volume check
	Send_UART("UART Initialized!\r\n");
	

	//sound test
	//sendDFPlayerCommand(0x06, 30);
	//sendDFPlayerCommand(0x03, 0x0001);
	//HAL_Delay(10000);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */		
		
		Start_State = 1;

		HAL_ADC_Start(&hadc);
		HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
		adc_raw = HAL_ADC_GetValue(&hadc);
		HAL_ADC_Stop(&hadc);
		
		vin = (adc_raw / ADC_MAX) * VREF;
		vbat = vin * ((R1 + R2) / R2);

		soc = getSOCFromVoltage(vbat);
		soh = getSOHFromVoltage(vbat);
		
		
		sprintf(uart_buf, "SOC: %d%%, SOH: %d%%\r\n",soc,soh);
    Send_UART(uart_buf);
		TransmitState(); //Transmit to ESP32

		// If SOH is below 70%, trigger alarm sound
		if(soh<=70&&check[4]==0){
			 check[4] = 1;
       sendDFPlayerCommand(0x06, 30);  // Set alarm volume
       sendDFPlayerCommand(0x03, 0x0005);  // Play alarm sound
			 HAL_Delay(5000);
			 
		 }
      else if(soc<=20&&check[0]==0){
			 check[0] = 1;
            sendDFPlayerCommand(0x06, 30);
            sendDFPlayerCommand(0x03, 0x0001);
				HAL_Delay(5000);
			 
		 }
    else if(soc<=15&&check[1]==0){
			 check[1] = 1;
            sendDFPlayerCommand(0x06, 30);
            sendDFPlayerCommand(0x03, 0x0002);
			HAL_Delay(5000);
			 
		 }
    
		 else if(soc<=10&&check[2]==0){
			 check[2] = 1;
            sendDFPlayerCommand(0x06, 30);
            sendDFPlayerCommand(0x03, 0x0003);
			 HAL_Delay(5000);
			 
		 }
        
    else if(soc<=5&&check[3]==0){
			 check[3] = 1;
            sendDFPlayerCommand(0x06, 30);
            sendDFPlayerCommand(0x03, 0x0004);
			HAL_Delay(5000);
			 
		 }


		HAL_Delay(1000);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_160CYCLES_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART4_UART_Init(void)
{

  /* USER CODE BEGIN USART4_Init 0 */

  /* USER CODE END USART4_Init 0 */

  /* USER CODE BEGIN USART4_Init 1 */

  /* USER CODE END USART4_Init 1 */
  huart4.Instance = USART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART4_Init 2 */

  /* USER CODE END USART4_Init 2 */

}

/**
  * @brief USART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART5_UART_Init(void)
{

  /* USER CODE BEGIN USART5_Init 0 */

  /* USER CODE END USART5_Init 0 */

  /* USER CODE BEGIN USART5_Init 1 */

  /* USER CODE END USART5_Init 1 */
  huart5.Instance = USART5;
  huart5.Init.BaudRate = 9600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART5_Init 2 */

  /* USER CODE END USART5_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
	//UART4  <- ESP32
	if (huart->Instance == USART4) {
		if (rxData >= '0' && rxData <= '9') {
			Start_State = rxData - '0';
		}
		HAL_UART_Receive_IT(&huart4, &rxData, 1);
	}
}




void sendDFPlayerCommand(uint8_t command, uint16_t parameter)
{
	uint8_t buffer[10];
	buffer[0] = 0x7E;
	buffer[1] = 0xFF;
	buffer[2] = 0x06;
	buffer[3] = command;
	buffer[4] = 0x00;
	buffer[5] = (parameter >> 8) & 0xFF;
	buffer[6] = parameter & 0xFF;

	uint16_t checksum = 0 - (buffer[1] + buffer[2] + buffer[3] +
		buffer[4] + buffer[5] + buffer[6]);

	buffer[7] = (checksum >> 8) & 0xFF;
	buffer[8] = checksum & 0xFF;
	buffer[9] = 0xEF;

	HAL_UART_Transmit(&huart5, buffer, 10, HAL_MAX_DELAY);
}



void Send_UART(char* msg)
{
	if (msg != NULL) {
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	}


}


void SendDataToESP32(uint8_t* data, uint16_t size) {
	if (HAL_UART_Transmit(&huart4, data, size, 1000) != HAL_OK) {
		Error_Handler();
	}
}


void TransmitState() {
	char data[50];

   
   
   sprintf(data, "SOC : %d%%, SOH : %d%%\r\n", soc, soh);
   
   
   SendDataToESP32((uint8_t*)data, strlen(data));
}

float getSOCFromVoltage(float voltage) {
    const float voltageTable[] = { 12.60, 12.45, 12.35, 12.25, 12.15, 12.05, 11.95, 11.85, 11.75, 11.65, 11.55, 11.45, 11.35, 11.25, 11.15, 10.95, 10.80, 10.65, 10.50, 10.20, 9.00 };
    const float socTable[] = { 100,   95,    90,    85,    80,    75,    70,    65,    60,    55,    50,    45,    40,    35,    30,    25,    20,    15,    10,    5,    0 };
    int i;
    for (i = 0; i < 20; i++) {
        if (voltage >= voltageTable[i + 1] && voltage <= voltageTable[i]) {
            float slope = (socTable[i + 1] - socTable[i]) / (voltageTable[i + 1] - voltageTable[i]);
            return socTable[i] + slope * (voltage - voltageTable[i]);
        }
    }
    if (voltage >= voltageTable[0]) return 100.0f;
    if (voltage <= voltageTable[20]) return 0.0f;
    return -1.0f;
}

float getSOHFromVoltage(float voltage) {
    const float nominal_voltage = 12.6f; // 100% SOH ??
    if (voltage >= nominal_voltage) return 100.0f;
    float soh = (voltage / nominal_voltage) * 100.0f;
    if (soh < 0.0f) soh = 0.0f;
    if (soh > 100.0f) soh = 100.0f;
    return soh;
}


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
