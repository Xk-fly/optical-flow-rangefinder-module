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
#include "PMW3901MB/pmw_3901.h"
#include "VL53L1X/vl53l1x.h"
#include "Adapter/massage_adapter.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PMW_READ_DELAY_MS               1U
#define PMW_INIT_RETRY_INTERVAL_MS      1000U
#define PMW_SCALE_PER_MILLE             3000U
#define FLOW_PERIOD_MS                  20U
#define DISTANCE_PERIOD_MS              100U
#define HEARTBEAT_PERIOD_MS             1000U
#define PMW3901_ENABLE_PERIODIC_DIAGNOSTICS 0U
#if PMW3901_ENABLE_PERIODIC_DIAGNOSTICS
#define PMW_DIAG_PERIOD_MS              1000U
#endif

#if PMW3901_ACQUISITION_MODE == PMW3901_ACQ_DIRECT
#define PMW_MODE_STATUS_TEXT             "PMW mode direct"
#else
#define PMW_MODE_STATUS_TEXT             "PMW mode burst"
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM14_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
static void PMW_SendInitFailed(uint8_t err);
#if PMW3901_ENABLE_PERIODIC_DIAGNOSTICS
static void PMW_SendRawDiagnostics(void);
static void PMW_SendDirectDiagnostics(const PMW3901_Diagnostics *diagnostics);
static void PMW_SendDirectStatistics(const PMW3901_Diagnostics *diagnostics);
static void PMW_SendBurstStatistics(const PMW3901_Diagnostics *diagnostics);
#endif
static void PMW_SendConfigDiagnostics(void);
static uint8_t PMW_AppendChar(char *message, uint8_t index, char value);
static uint8_t PMW_AppendText(char *message, uint8_t index, const char *text);
static uint8_t PMW_AppendHex8(char *message, uint8_t index, uint8_t value);
#if PMW3901_ENABLE_PERIODIC_DIAGNOSTICS
static uint8_t PMW_AppendHex16(char *message, uint8_t index, uint16_t value);
#endif
static uint8_t PMW_AppendUnsigned(char *message, uint8_t index, uint32_t value);
#if PMW3901_ENABLE_PERIODIC_DIAGNOSTICS
static uint8_t PMW_AppendSigned(char *message, uint8_t index, int16_t value);
#endif

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void PMW_SendInitFailed(uint8_t err)
{
	char msg[] = "PMW init failed err=000";
	uint8_t index = 20U;

	if (err >= 100U) {
		msg[index++] = (char)('0' + (err / 100U));
		msg[index++] = (char)('0' + ((err / 10U) % 10U));
		msg[index++] = (char)('0' + (err % 10U));
		msg[index] = '\0';
	} else if (err >= 10U) {
		msg[index++] = (char)('0' + (err / 10U));
		msg[index++] = (char)('0' + (err % 10U));
		msg[index] = '\0';
	} else {
		msg[index++] = (char)('0' + err);
		msg[index] = '\0';
	}

	send_status_text(&huart2, MAV_SEVERITY_WARNING, msg);
}

static uint8_t PMW_AppendChar(char *message, uint8_t index, char value)
{
	if (index < 49U) {
		message[index++] = value;
	}
	message[index] = '\0';
	return index;
}

static uint8_t PMW_AppendText(char *message, uint8_t index, const char *text)
{
	while ((*text != '\0') && (index < 49U)) {
		message[index++] = *text++;
	}
	message[index] = '\0';
	return index;
}

static uint8_t PMW_AppendHex8(char *message, uint8_t index, uint8_t value)
{
	static const char hex[] = "0123456789ABCDEF";
	index = PMW_AppendChar(message, index, hex[(value >> 4) & 0x0FU]);
	return PMW_AppendChar(message, index, hex[value & 0x0FU]);
}

#if PMW3901_ENABLE_PERIODIC_DIAGNOSTICS
static uint8_t PMW_AppendHex16(char *message, uint8_t index, uint16_t value)
{
	index = PMW_AppendHex8(message, index, (uint8_t)(value >> 8));
	return PMW_AppendHex8(message, index, (uint8_t)value);
}
#endif

static uint8_t PMW_AppendUnsigned(char *message, uint8_t index, uint32_t value)
{
	char digits[3];
	uint8_t count = 0U;

	if (value > 999U) {
		value = 999U;
	}
	do {
		digits[count++] = (char)('0' + (value % 10U));
		value /= 10U;
	} while ((value != 0U) && (count < sizeof(digits)));

	while (count > 0U) {
		index = PMW_AppendChar(message, index, digits[--count]);
	}
	return index;
}

#if PMW3901_ENABLE_PERIODIC_DIAGNOSTICS
static uint8_t PMW_AppendSigned(char *message, uint8_t index, int16_t value)
{
	int32_t signed_value = value;
	uint32_t magnitude;
	char digits[5];
	uint8_t count = 0U;

	if (signed_value < 0) {
		index = PMW_AppendChar(message, index, '-');
		magnitude = (uint32_t)(-signed_value);
	} else {
		magnitude = (uint32_t)signed_value;
	}

	do {
		digits[count++] = (char)('0' + (magnitude % 10U));
		magnitude /= 10U;
	} while ((magnitude != 0U) && (count < sizeof(digits)));

	while (count > 0U) {
		index = PMW_AppendChar(message, index, digits[--count]);
	}
	return index;
}
#endif

#if PMW3901_ENABLE_PERIODIC_DIAGNOSTICS
static void PMW_SendRawDiagnostics(void)
{
	PMW3901_Diagnostics diagnostics;
	char message[50] = {0};
	uint8_t index = 0U;

	PMW3901_GetDiagnostics(&diagnostics);
	index = PMW_AppendText(message, index, "P S");
	index = PMW_AppendUnsigned(message, index, diagnostics.last_status);
	index = PMW_AppendText(message, index, " M");
	index = PMW_AppendHex8(message, index, diagnostics.last_raw.motion);
	index = PMW_AppendText(message, index, " O");
	index = PMW_AppendHex8(message, index, diagnostics.last_raw.observation);
	index = PMW_AppendText(message, index, " X");
	index = PMW_AppendSigned(message, index, diagnostics.last_raw.delta_x);
	index = PMW_AppendText(message, index, " Y");
	index = PMW_AppendSigned(message, index, diagnostics.last_raw.delta_y);
	index = PMW_AppendText(message, index, " Q");
	index = PMW_AppendUnsigned(message, index, diagnostics.last_raw.squal);
	index = PMW_AppendText(message, index, " H");
	index = PMW_AppendHex16(message, index, diagnostics.last_raw.shutter);
	index = PMW_AppendText(message, index, " E");
	index = PMW_AppendUnsigned(message, index, diagnostics.spi_error_count);
	index = PMW_AppendText(message, index, " R");
	(void)PMW_AppendUnsigned(message, index, diagnostics.reject_count);
	send_status_text(&huart2, MAV_SEVERITY_DEBUG, message);
	PMW_SendDirectDiagnostics(&diagnostics);
	PMW_SendDirectStatistics(&diagnostics);
	PMW_SendBurstStatistics(&diagnostics);
}

static void PMW_SendDirectDiagnostics(const PMW3901_Diagnostics *diagnostics)
{
	char message[50] = {0};
	uint8_t index = 0U;

	index = PMW_AppendText(message, index, "D S");
	index = PMW_AppendUnsigned(message, index, diagnostics->direct_status);
	index = PMW_AppendText(message, index, " M");
	index = PMW_AppendHex8(message, index, diagnostics->last_direct.motion);
	index = PMW_AppendText(message, index, " O");
	index = PMW_AppendHex8(message, index, diagnostics->last_direct.observation);
	index = PMW_AppendText(message, index, " X");
	index = PMW_AppendSigned(message, index, diagnostics->last_direct.delta_x);
	index = PMW_AppendText(message, index, " Y");
	index = PMW_AppendSigned(message, index, diagnostics->last_direct.delta_y);
	index = PMW_AppendText(message, index, " Q");
	index = PMW_AppendUnsigned(message, index, diagnostics->last_direct.squal);
	index = PMW_AppendText(message, index, " H");
	index = PMW_AppendHex16(message, index, diagnostics->last_direct.shutter);
	index = PMW_AppendText(message, index, " E");
	(void)PMW_AppendUnsigned(message, index, diagnostics->spi_error_count);
	send_status_text(&huart2, MAV_SEVERITY_DEBUG, message);
}

static void PMW_SendDirectStatistics(const PMW3901_Diagnostics *diagnostics)
{
	char message[50] = {0};
	uint8_t index = 0U;

	index = PMW_AppendText(message, index, "T C");
	index = PMW_AppendUnsigned(message, index, diagnostics->direct_sample_count);
	index = PMW_AppendText(message, index, " V");
	index = PMW_AppendUnsigned(message, index, diagnostics->direct_motion_count);
	index = PMW_AppendText(message, index, " Q");
	(void)PMW_AppendUnsigned(message, index, diagnostics->direct_squal_nonzero_count);
	send_status_text(&huart2, MAV_SEVERITY_DEBUG, message);
}

static void PMW_SendBurstStatistics(const PMW3901_Diagnostics *diagnostics)
{
	char message[50] = {0};
	uint8_t index = 0U;

	index = PMW_AppendText(message, index, "R U");
	index = PMW_AppendUnsigned(message, index, diagnostics->last_raw.raw_sum);
	index = PMW_AppendText(message, index, " A");
	index = PMW_AppendUnsigned(message, index, diagnostics->last_raw.raw_max);
	index = PMW_AppendText(message, index, " I");
	index = PMW_AppendUnsigned(message, index, diagnostics->last_raw.raw_min);
	index = PMW_AppendText(message, index, " C");
	index = PMW_AppendUnsigned(message, index, diagnostics->sample_count);
	index = PMW_AppendText(message, index, " V");
	index = PMW_AppendUnsigned(message, index, diagnostics->motion_count);
	index = PMW_AppendText(message, index, " Q");
	(void)PMW_AppendUnsigned(message, index, diagnostics->squal_nonzero_count);
	send_status_text(&huart2, MAV_SEVERITY_DEBUG, message);
}
#endif

static void PMW_SendConfigDiagnostics(void)
{
	PMW3901_Diagnostics diagnostics;
	char message[50] = {0};
	uint8_t index = 0U;

	PMW3901_GetDiagnostics(&diagnostics);
	index = PMW_AppendText(message, index, "C S");
	index = PMW_AppendUnsigned(message, index, diagnostics.config_status);
	index = PMW_AppendText(message, index, " D");
	index = PMW_AppendHex8(message, index, diagnostics.config_bank0_4d);
	index = PMW_AppendText(message, index, " A");
	index = PMW_AppendHex8(message, index, diagnostics.config_bank14_65);
	index = PMW_AppendText(message, index, " F");
	index = PMW_AppendHex8(message, index, diagnostics.config_bank15_48);
	index = PMW_AppendText(message, index, " N");
	(void)PMW_AppendUnsigned(message, index, diagnostics.config_mismatch_count);
	send_status_text(&huart2, MAV_SEVERITY_INFO, message);
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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
	  MX_TIM14_Init();
	  MX_I2C2_Init();
	  /* USER CODE BEGIN 2 */
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	  uint8_t pmw_err = PMW3901_init(hspi1, PMW_READ_DELAY_MS);
	  uint8_t pmw_ok = (pmw_err == 0U);
	  uint32_t pmw_last_retry_ms = HAL_GetTick();

	  if (pmw_ok) {
		  send_status_text(&huart2, MAV_SEVERITY_INFO, "PMW init ok");
		  send_status_text(&huart2, MAV_SEVERITY_INFO, PMW_MODE_STATUS_TEXT);
		  PMW_SendConfigDiagnostics();
	  } else {
		  PMW_SendInitFailed(pmw_err);
	  }
	  uint8_t vl53_ok = vl53_Init();
	  send_status_text(&huart2, vl53_ok ? MAV_SEVERITY_INFO : MAV_SEVERITY_WARNING,
	                   vl53_ok ? "VL53 init ok" : "VL53 init failed");
	  send_heart_beat(&huart2);
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t scheduler_start_ms = HAL_GetTick();
  uint32_t last_flow_ms = scheduler_start_ms;
  uint32_t last_distance_ms = scheduler_start_ms;
  uint32_t last_heartbeat_ms = scheduler_start_ms;
#if PMW3901_ENABLE_PERIODIC_DIAGNOSTICS
  uint32_t last_diag_ms = scheduler_start_ms;
#endif
  uint16_t distance = 0U;
  uint8_t distance_valid = 0U;

  while (1) {
	  uint32_t now = HAL_GetTick();

	  if ((!pmw_ok) && (now - pmw_last_retry_ms >= PMW_INIT_RETRY_INTERVAL_MS)) {
		  pmw_last_retry_ms = now;
		  pmw_err = PMW3901_init(hspi1, PMW_READ_DELAY_MS);
		  pmw_ok = (pmw_err == 0U);
		  if (pmw_ok) {
			  send_status_text(&huart2, MAV_SEVERITY_INFO, "PMW init ok");
			  send_status_text(&huart2, MAV_SEVERITY_INFO, PMW_MODE_STATUS_TEXT);
			  PMW_SendConfigDiagnostics();
		  } else {
			  PMW_SendInitFailed(pmw_err);
		  }
	  }

#if PMW3901_ENABLE_PERIODIC_DIAGNOSTICS
	  if (now - last_diag_ms >= PMW_DIAG_PERIOD_MS) {
		  last_diag_ms = now;
		  PMW_SendRawDiagnostics();
	  }
#endif

	  if (now - last_flow_ms >= FLOW_PERIOD_MS) {
		  int x = 0;
		  int y = 0;
		  uint8_t quality = 0U;

		  last_flow_ms = now;
		  if (pmw_ok) {
			  pollingMotion(&x, &y, &quality, PMW_SCALE_PER_MILLE);
		  }
		  send_optical_flow(&huart2, x, y, quality, distance, distance_valid);
	  }

	  if (now - last_distance_ms >= DISTANCE_PERIOD_MS) {
		  last_distance_ms = now;
		  distance_valid = vl53_ok ? vl53_GetDistance(&distance) : 0U;
		  if (distance_valid != 0U) {
			  send_distance_sensor(&huart2, distance);
		  }
	  }

	  if (now - last_heartbeat_ms >= HEARTBEAT_PERIOD_MS) {
		  last_heartbeat_ms = now;
		  send_heart_beat(&huart2);
	  }

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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00503D58;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 1600-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 10000-1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
