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
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

struct cycleCalculations {
	uint32_t sum;
	uint16_t avg;
	uint16_t min;
	uint16_t max;
	uint16_t numOver750;
	uint32_t sumOver750;
};

struct txHeader_t {
	char preamble[4];
	uint8_t messageType;
	uint16_t bodyLength;
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUF_LEN 995
#define NUM_CIRCUITS 8
#define NUM_SEGMENTS 21 // Must be <= ADC_BUF_LEN
#define HALF_CYCLE_HISTORY_LEN 30
#define TIMER_HZ_PER_120_HZ 2291667
#define TIMER_FREQ 275000000UL
#define TIMER_TICKS_PER_USEC 275
#define L1V_LOWPASS_DELAY_USEC 780
#define GRID_FREQ_MIN_PERIOD TIMER_FREQ / (61 * 2)
#define GRID_FREQ_MAX_PERIOD TIMER_FREQ / (59 * 2)
#define CORRECTION_DIVIDER 10
#define CYCLE_LOOKBACK_LENGTH 120

#define IDLE   0
#define DONE   1
//#define BINARY_TELEMETRY 1
#define TEXT_TELEMETRY 1
#define F_CLK  275000000UL
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;
DMA_HandleTypeDef hdma_adc3;

CRC_HandleTypeDef hcrc;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim23;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
#define ADC_SAMPLE_SIZE 2 // 16 bits = 2 bytes
static __attribute__((section(".adc_dma_buf"))) __attribute__((aligned(0x20))) uint16_t adc_buf1[2][ADC_BUF_LEN * 2];
uint16_t sampleHistory[16][ADC_BUF_LEN * 2];
uint16_t inboundADCBuf = 0;
uint16_t cktComputeBufs[8][ADC_BUF_LEN];
uint32_t segmentSums[NUM_CIRCUITS][NUM_SEGMENTS];
uint16_t segmentCounts[NUM_CIRCUITS][NUM_SEGMENTS];
uint16_t segmentAverages[NUM_CIRCUITS][NUM_SEGMENTS];
uint32_t segmentSumOfAverages[NUM_CIRCUITS][NUM_SEGMENTS];
uint32_t segmentSumOfSO750s[NUM_CIRCUITS][NUM_SEGMENTS];
uint16_t highWaterMark = 0;
uint16_t triggerCount = 0;
uint32_t last_ccr1 = 0;
uint32_t last_ccr2 = 0;
uint16_t ckt1minMin = 65535;
uint16_t ckt1maxMax = 0;
uint16_t ckt2minMin = 65535;
uint16_t ckt2maxMax = 0;
struct txHeader_t txHeader;
struct cycleCalculations cycleCalcsLookback[NUM_CIRCUITS][CYCLE_LOOKBACK_LENGTH];
uint16_t cycleCalcsLookback_tail = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM23_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM5_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */

void doHalfCycleCompute(uint16_t computeADCBuf);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

volatile bool isAdcWork = false;
uint32_t cycleCount = 0;
volatile uint32_t adcCounter = 0;
uint16_t adc3min = 4096;
uint16_t adc3max = 0;
char msg[64];

volatile uint8_t gu8_State = IDLE;
volatile uint8_t gu8_MSG[100] = {'\0'};
volatile uint32_t gu32_T1 = 0;
volatile uint32_t gu32_T2 = 0;
volatile uint32_t gu32_Ticks = 0;
volatile uint16_t gu16_TIM2_OVC = 0;
volatile uint32_t gu32_Freq = 0;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	if (hadc->Instance == ADC1) {
		HAL_GPIO_WritePin(GPIO3_GPIO_Port, GPIO3_Pin, GPIO_PIN_RESET);

		isAdcWork = false;
	} else if (hadc->Instance == ADC2) {
		SCB_InvalidateDCache_by_Addr((uint32_t *)&adc_buf1[0][0], ADC_BUF_LEN * 2);
		adcCounter++;
		HAL_ADC_Start_DMA(&hadc2,(uint32_t *)adc_buf1[0], ADC_BUF_LEN * 2);
	} else if (hadc->Instance == ADC3) {
		//HAL_GPIO_WritePin(GPIO3_GPIO_Port, GPIO3_Pin, GPIO_PIN_RESET);

		isAdcWork = false;
		SCB_InvalidateDCache_by_Addr((uint32_t *)&adc_buf1[0][0], ADC_BUF_LEN * 2);
		adcCounter++;
		//HAL_GPIO_WritePin(PG3_GPIO_Port, PG3_Pin, GPIO_PIN_RESET);

		//for (int i = 0; i < ADC_BUF_LEN; i++) {
		//	sum += adc_buf1[0][i];
		//}
		//avg = sum / ADC_BUF_LEN;

		//HAL_GPIO_WritePin(GPIO3_GPIO_Port, GPIO3_Pin, GPIO_PIN_SET);
		HAL_ADC_Start_DMA(&hadc3, (uint32_t *)adc_buf1[0], ADC_BUF_LEN);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
	if (htim->Instance == TIM3) {
		// Loss of ZC signal
		uint32_t cnt = TIM3->CNT;
		uint32_t sr = TIM3->SR;
		HAL_GPIO_WritePin(GPIO1_GPIO_Port, GPIO1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIO1_GPIO_Port, GPIO1_Pin, GPIO_PIN_RESET);

		//TIM4->ARR = 15964;
		adcCounter++;
	} else if (htim->Instance == TIM4 || htim->Instance == TIM5) {
		if (cycleCount < 2) {
			return;
		}
		HAL_GPIO_WritePin(GPIO3_GPIO_Port, GPIO3_Pin, GPIO_PIN_SET);
		if (inboundADCBuf == 0) {
			inboundADCBuf = 1;
		} else {
			inboundADCBuf = 0;
		}
		SCB_InvalidateDCache_by_Addr((uint32_t *)&adc_buf1[inboundADCBuf], ADC_BUF_LEN * ADC_SAMPLE_SIZE * 2);
		HAL_ADC_Start_DMA(&hadc1,(uint32_t *)adc_buf1[inboundADCBuf], ADC_BUF_LEN * 2);

		uint16_t computeADCBuf;
		if (inboundADCBuf == 0) {
			computeADCBuf = 1;
		} else {
			computeADCBuf = 0;
		}
		if (cycleCount < 3) {
			return;
		}
		doHalfCycleCompute(computeADCBuf);
	} else if (htim->Instance == TIM6) {
	    //sprintf(gu8_MSG, "avg = %u\r\n", ((DMA_Stream_TypeDef *)hadc1.DMA_Handle->Instance)->NDTR);
	    //HAL_UART_Transmit(&huart1, gu8_MSG, strlen(gu8_MSG), 100);

	} else if (htim->Instance == TIM23) {
	    //HAL_UART_Transmit(&huart1, noiseGarbage, 1023, 1000);
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim) {
	uint32_t ccr2 = TIM3->CCR2;
	if (ccr2 == 0) {
		return;
	}
	cycleCount++;
	last_ccr2 = ccr2;
	TIM4->ARR = (ccr2 / 2) - 3100;
	TIM5->ARR = ccr2 - 3100;
	float freq = (275000000. / (ccr2 * 80)) - 0.70;
	//adcCounter++;
	//if (adcCounter > 10) {
	//	HAL_TIM_IC_Stop(&htim3, TIM_CHANNEL_2);
	//	HAL_TIM_IC_Stop(&htim3, TIM_CHANNEL_1);	// Rising edge capture
	//	HAL_TIM_Base_Stop(&htim3);			// Falling edge capture
	//}
	//  htim4.Instance->CR1 &= TIM_CR1_URS | TIM_CR1_OPM;
	//  htim5.Instance->CR1 &= TIM_CR1_URS | TIM_CR1_OPM;

    //sprintf(gu8_MSG, "CCR1 = %.2f\r\n", freq);
    //HAL_UART_Transmit(&huart1, gu8_MSG, strlen(gu8_MSG), 100);

	/*
    if(gu8_State == IDLE)
    {
    	HAL_GPIO_WritePin(GPIO2_GPIO_Port, GPIO2_Pin, GPIO_PIN_SET);
        gu32_T1 = TIM3->CCR2;
        gu16_TIM2_OVC = 0;
        gu8_State = DONE;
    	HAL_GPIO_WritePin(GPIO2_GPIO_Port, GPIO2_Pin, GPIO_PIN_RESET);
    }
    else if(gu8_State == DONE)
    {

        gu32_T2 = TIM3->CCR2;
        gu32_Ticks = (gu32_T2 + (gu16_TIM2_OVC * 65536)) - gu32_T1;
        gu32_Freq = (uint32_t)(F_CLK/gu32_Ticks);
        if(gu32_Freq != 0)
        {
          sprintf(gu8_MSG, "Frequency = %lu Hz\n\r", gu32_Freq / 2);
          HAL_UART_Transmit(&huart1, gu8_MSG, sizeof(gu8_MSG), 100);
        } else {
        	HAL_GPIO_WritePin(GPIO1_GPIO_Port, GPIO1_Pin, GPIO_PIN_RESET);

        }
        gu8_State = IDLE;
    	HAL_GPIO_WritePin(GPIO1_GPIO_Port, GPIO1_Pin, GPIO_PIN_RESET);

    }
    */

	//HAL_GPIO_WritePin(GPIO3_GPIO_Port, GPIO3_Pin, GPIO_PIN_RESET);
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

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_ADC3_Init();
  MX_TIM4_Init();
  MX_TIM23_Init();
  MX_ADC2_Init();
  MX_TIM12_Init();
  MX_TIM5_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(GPIO3_GPIO_Port, GPIO3_Pin, GPIO_PIN_SET);
  for (int i = 0; i < ADC_BUF_LEN; i++) {
	  adc_buf1[0][i] = 0;
	  adc_buf1[1][i] = 0;
	  for (int ix = 0; ix < 8; ix++) {
		  cktComputeBufs[ix][i] = 0;
	  }
  }
  for (int i = 0; i < NUM_CIRCUITS; i++) {
	  for (int ix = 0; ix < CYCLE_LOOKBACK_LENGTH; ix++) {
		  cycleCalcsLookback[i][ix].sum = 0;
		  cycleCalcsLookback[i][ix].avg = 0;
		  cycleCalcsLookback[i][ix].min = 65535;
		  cycleCalcsLookback[i][ix].max = 0;
		  cycleCalcsLookback[i][ix].numOver750 = 0;
		  cycleCalcsLookback[i][ix].sumOver750 = 0;
	  }
  }
  HAL_GPIO_WritePin(GPIO3_GPIO_Port, GPIO3_Pin, GPIO_PIN_RESET);

  sampleHistory[0][0] = 0;
  txHeader.preamble[0] = 0xde;
  txHeader.preamble[1] = 0xad;
  txHeader.preamble[2] = 0xbe;
  txHeader.preamble[3] = 0xef;

  //HAL_GPIO_WritePin(PG3_GPIO_Port, PG3_Pin, GPIO_PIN_SET);
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
  //HAL_GPIO_WritePin(PG3_GPIO_Port, PG3_Pin, GPIO_PIN_RESET);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  TIM2->CCR2 = 1718;		// B
  TIM2->CCR3 = 0;		// G
  TIM2->CCR4 = 0;	// R

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  TIM1->CCR1 = 1718;		// B
  TIM1->CCR4 = 0;		// G
  TIM2->CCR1 = 0;	// R

  HAL_TIM_Base_Start_IT(&htim4);
  htim4.Instance->CR1 &= TIM_CR1_URS | TIM_CR1_OPM;
  HAL_TIM_Base_Start_IT(&htim5);
  htim5.Instance->CR1 &= TIM_CR1_URS | TIM_CR1_OPM;
  HAL_GPIO_WritePin(GPIO1_GPIO_Port, GPIO1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIO1_GPIO_Port, GPIO1_Pin, GPIO_PIN_RESET);
  //HAL_TIM_IC_Start(&htim3, TIM_CHANNEL_1);	// Rising edge capture
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim3);			// Falling edge capture
  HAL_GPIO_WritePin(GPIO1_GPIO_Port, GPIO1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIO1_GPIO_Port, GPIO1_Pin, GPIO_PIN_RESET);
  HAL_TIM_Base_Start_IT(&htim23);
  //HAL_TIM_Base_Start_IT(&htim4);
  //HAL_TIM_Base_Start_IT(&htim5);
  HAL_TIM_OnePulse_Start(&htim4, TIM_CHANNEL_1);
  //htim4.Instance->CR1|=(TIM_CR1_CEN);
  //htim5.Instance->CR1|=(TIM_CR1_CEN);

  htim4.Instance->CR1 |= TIM_CR1_URS | TIM_CR1_OPM;
  htim5.Instance->CR1 |= TIM_CR1_URS | TIM_CR1_OPM;
  htim23.Instance->CR1 |= TIM_CR1_URS | TIM_CR1_OPM;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t dir = 1;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  /*
	  if (dir) {
		  TIM2->CCR3 += 150;
		  TIM1->CCR4 += 150;
		  if (TIM2->CCR3 > 10000) {
			  dir = 0;
		  }
	  } else {
		  TIM2->CCR3 -= 150;
		  TIM1->CCR4 -= 150;
		  if (TIM2->CCR3 < 150) {
			  dir = 1;
		  }
	  }
	  */
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 32;
  RCC_OscInitStruct.PLL.PLLN = 275;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_SPI2;
  PeriphClkInitStruct.PLL2.PLL2M = 4;
  PeriphClkInitStruct.PLL2.PLL2N = 12;
  PeriphClkInitStruct.PLL2.PLL2P = 5;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_ONESHOT;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV8;
  hadc2.Init.Resolution = ADC_RESOLUTION_16B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_ONESHOT;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV256;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.DataAlign = ADC3_DATAALIGN_RIGHT;
  hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.NbrOfConversion = 2;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.SamplingMode = ADC_SAMPLING_MODE_NORMAL;
  hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC3_SAMPLETIME_24CYCLES_5;
  sConfig.SingleDiff = ADC_DIFFERENTIAL_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 2048;
  sConfig.OffsetSign = ADC3_OFFSET_SIGN_NEGATIVE;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_2;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 13750;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 80;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_COMBINED_RESETTRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_TI2FP2;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 15;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 80;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 26102;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim4, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_COMBINED_RESETTRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR2;
  if (HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_RETRIGERRABLE_OPM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 80;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 52247;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim5, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_COMBINED_RESETTRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR2;
  if (HAL_TIM_SlaveConfigSynchro(&htim5, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 0;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 65535;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim12, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */

}

/**
  * @brief TIM23 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM23_Init(void)
{

  /* USER CODE BEGIN TIM23_Init 0 */

  /* USER CODE END TIM23_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM23_Init 1 */

  /* USER CODE END TIM23_Init 1 */
  htim23.Instance = TIM23;
  htim23.Init.Prescaler = 80;
  htim23.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim23.Init.Period = 10000;
  htim23.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim23.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim23) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim23, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim23, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_COMBINED_RESETTRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR2;
  if (HAL_TIM_SlaveConfigSynchro(&htim23, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim23, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM23_Init 2 */

  /* USER CODE END TIM23_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 921600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO1_Pin|GPIO2_Pin|GPIO3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : GPIO1_Pin */
  GPIO_InitStruct.Pin = GPIO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIO1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO2_Pin GPIO3_Pin */
  GPIO_InitStruct.Pin = GPIO2_Pin|GPIO3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void doHalfCycleCompute(uint16_t computeADCBuf) {
	/*
	uint32_t sum;
	uint16_t avg;
	uint16_t min, max;
	uint16_t numOver750;
	uint16_t sumOver750;
	*/
	struct cycleCalculations cycleCalcs[NUM_CIRCUITS];

	for (int i = 0; i < NUM_CIRCUITS; i++) {
		cycleCalcs[i].sum = 0;
		cycleCalcs[i].avg = 0;
		cycleCalcs[i].min = 65535;
		cycleCalcs[i].max = 0;
		cycleCalcs[i].numOver750 = 0;
		cycleCalcs[i].sumOver750 = 0;
	}

	SCB_InvalidateDCache_by_Addr((uint32_t *)&adc_buf1[computeADCBuf], ADC_BUF_LEN * ADC_SAMPLE_SIZE * 2);
	if (computeADCBuf == 1) {
		adcCounter++;
	}
	adcCounter++;
	//HAL_GPIO_WritePin(PG3_GPIO_Port, PG3_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(GPIO1_GPIO_Port, GPIO1_Pin, GPIO_PIN_SET);

	for (int i = 0; i < ADC_BUF_LEN; i++) {
		cktComputeBufs[0][i] = adc_buf1[computeADCBuf][(i * 2)];
		cktComputeBufs[1][i] = adc_buf1[computeADCBuf][(i * 2) + 1];
	}

	HAL_GPIO_WritePin(GPIO1_GPIO_Port, GPIO1_Pin, GPIO_PIN_RESET);

	// Start processing each circuit in sequence
	for (int ckt = 0; ckt <= 1; ckt++) {
		struct cycleCalculations *calcs = &(cycleCalcs[ckt]);
		HAL_GPIO_WritePin(GPIO1_GPIO_Port, GPIO1_Pin, GPIO_PIN_SET);
		for (int i = 0; i < NUM_SEGMENTS; i++) {
			segmentSums[ckt][i] = 0;
			segmentCounts[ckt][i] = 0;
		}

		for (int i = 0; i < ADC_BUF_LEN; i++) {
			//if (cktComputeBufs[ckt][i] == 0) {
			//	HAL_GPIO_WritePin(GPIO2_GPIO_Port, GPIO2_Pin, GPIO_PIN_SET);
			//} else {
			//	HAL_GPIO_WritePin(GPIO2_GPIO_Port, GPIO2_Pin, GPIO_PIN_RESET);
			//}
			calcs->sum += cktComputeBufs[ckt][i];
			if (cktComputeBufs[ckt][i] > calcs->max) {
				calcs->max = cktComputeBufs[ckt][i];
			}
			if (cktComputeBufs[ckt][i] < calcs->min) {
				if (cktComputeBufs[ckt][i] == 0) {
					calcs->min = 123;
				}
				calcs->min = cktComputeBufs[ckt][i];
			}
			uint16_t segmentNumber = i / (ADC_BUF_LEN / NUM_SEGMENTS);
			segmentSums[ckt][segmentNumber] += cktComputeBufs[ckt][i];
			segmentCounts[ckt][segmentNumber]++;
		}
		//HAL_GPIO_WritePin(GPIO2_GPIO_Port, GPIO2_Pin, GPIO_PIN_RESET);
		calcs->avg = calcs->sum / ADC_BUF_LEN;
		if (calcs->avg > highWaterMark) {
			highWaterMark = calcs->avg;
		}
		if (calcs->max > ckt1maxMax) {
			ckt1maxMax = calcs->max;
		}
		if (calcs->min < ckt1minMin) {
			ckt1minMin = calcs->min;
		}

		for (int i = 0; i < ADC_BUF_LEN; i++) {
			if (cktComputeBufs[ckt][i] > calcs->avg + 750) {
				calcs->numOver750++;
				calcs->sumOver750 += cktComputeBufs[ckt][i] - calcs->avg - 750;
				//uint16_t segmentNumber = i / (ADC_BUF_LEN / NUM_SEGMENTS);
				//segmentSumOfSO750s[ckt][segmentNumber] += cktComputeBufs[ckt] - calcs->avg - 750;
			}
		}

		for (int i = 0; i < ADC_BUF_LEN; i++) {
			if (cktComputeBufs[ckt][i] > calcs->avg + 750) {
				uint16_t segmentNumber = i / (ADC_BUF_LEN / NUM_SEGMENTS);
				segmentSumOfSO750s[ckt][segmentNumber] += cktComputeBufs[ckt][i] - calcs->avg - 750;
			}
		}

		if (calcs->sumOver750 > 50000) {  // This is just a place to put a breakpoint for when SO750 > 50000
			int i = 0;
			i++;
		}

		uint16_t foo = sizeof(*calcs);
		memcpy(&cycleCalcsLookback[ckt][cycleCalcsLookback_tail], calcs, sizeof(*calcs));

		for (int i = 0; i < NUM_SEGMENTS; i++) {
			segmentAverages[ckt][i] = segmentSums[ckt][i] / segmentCounts[ckt][i];
			segmentSumOfAverages[ckt][i] += segmentAverages[ckt][i];
		}
		HAL_GPIO_WritePin(GPIO1_GPIO_Port, GPIO1_Pin, GPIO_PIN_RESET);

#ifdef TEXT_TELEMETRY
		if (ckt == 1) {
			sprintf(gu8_MSG, "%lu: avg = %5u, max %5u, no750 %3u, so750 %6u, %5u/%5u, %u\r\n", cycleCount, calcs->avg, calcs->max, calcs->numOver750, calcs->sumOver750, ckt1minMin, ckt1maxMax, triggerCount);
			HAL_UART_Transmit(&huart1, gu8_MSG, strlen(gu8_MSG), 1000);
		}
#endif
	}

#ifdef BINARY_TELEMETRY
	txHeader.messageType = 0x01;
	txHeader.bodyLength = 128;
	HAL_UART_Transmit(&huart1, &txHeader, sizeof(txHeader), 1000);
	HAL_UART_Transmit(&huart1, &cycleCalcs, sizeof(cycleCalcs), 1000);
#endif

	cycleCalcsLookback_tail++;
	if (cycleCalcsLookback_tail >= CYCLE_LOOKBACK_LENGTH) {
		cycleCalcsLookback_tail = 0;
	}

	// Look for trip events
	uint32_t so750Sum = 0;
	uint32_t no750Sum = 0;
	HAL_GPIO_WritePin(GPIO1_GPIO_Port, GPIO1_Pin, GPIO_PIN_SET);
	for (int i = 0; i < CYCLE_LOOKBACK_LENGTH; i++) {
		so750Sum += cycleCalcsLookback[1][i].sumOver750;
		no750Sum += cycleCalcsLookback[1][i].numOver750;
	}
	HAL_GPIO_WritePin(GPIO1_GPIO_Port, GPIO1_Pin, GPIO_PIN_RESET);

	if (so750Sum > 800000) {
		triggerCount++;
		for (int ix = 0; ix < CYCLE_LOOKBACK_LENGTH; ix++) {
			cycleCalcsLookback[1][ix].sum = 0;
			cycleCalcsLookback[1][ix].avg = 0;
			cycleCalcsLookback[1][ix].min = 65535;
			cycleCalcsLookback[1][ix].max = 0;
			cycleCalcsLookback[1][ix].numOver750 = 0;
			cycleCalcsLookback[1][ix].sumOver750 = 0;
		}
	}

	HAL_GPIO_WritePin(GPIO1_GPIO_Port, GPIO1_Pin, GPIO_PIN_SET);
	memcpy(sampleHistory[0][0], adc_buf1[computeADCBuf], sizeof(adc_buf1[computeADCBuf]));
	HAL_GPIO_WritePin(GPIO1_GPIO_Port, GPIO1_Pin, GPIO_PIN_RESET);

	//if (cycleCount % 60 == 0) {
	//	for (int i = 0; i < ADC_BUF_LEN; i++) {
	//	    sprintf(gu8_MSG, "%u\r\n", ckt2ComputeBuf[i]);
	//	    HAL_UART_Transmit(&huart1, gu8_MSG, strlen(gu8_MSG), 1000);
	//	}
	//}
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
