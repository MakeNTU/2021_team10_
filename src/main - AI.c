/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_x-cube-ai.h"
//#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32746g_discovery_ts.h"
#include "stm32746g_discovery_lcd.h"
#ifdef STAGE2
#include <stdbool.h>
#include "img_preprocess.h"
#endif
#ifdef STAGE3
#include "dwt_utility.h"
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define SDRAM_DEVICE_ADDR  ((uint32_t)0xC0000000)
#define LCD_FRAME_BUFFER   SDRAM_DEVICE_ADDR

#ifdef STAGE2
#define PATCH_SIZE 28
#define PEN_POINT_SIZE 7
#define TOUCH_TIMEOUT 700
#define PATCH_SIZE_X 58
#define PATCH_SIZE_Y 32
#endif
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

CRC_HandleTypeDef hcrc;

DMA2D_HandleTypeDef hdma2d;

I2C_HandleTypeDef hi2c3;

LTDC_HandleTypeDef hltdc;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;

SDRAM_HandleTypeDef hsdram1;

/* USER CODE BEGIN PV */
uint8_t RX_BUFFER[1] = { '1' };
#ifdef STAGE2
static uint32_t g_patch8888_5832[PATCH_SIZE_X*PATCH_SIZE_Y];
static uint32_t g_patch8888_2828[28*28];
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA2D_Init(void);
static void MX_FMC_Init(void);
static void MX_I2C3_Init(void);
static void MX_LTDC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
#ifdef STAGE2
    static TS_StateTypeDef ts_state;
    static uint32_t touch_time;
    static bool data_ready = false;
#endif
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

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA2D_Init();
	MX_FMC_Init();
	MX_I2C3_Init();
	MX_LTDC_Init();
	MX_USART1_UART_Init();
	MX_CRC_Init();
	MX_TIM6_Init();
	MX_USART6_UART_Init();
	MX_X_CUBE_AI_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim6);
	/* LCD + TS init*/
	BSP_LCD_Init();
	BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());

	/* Draw "READY" */
	BSP_LCD_LayerDefaultInit(LTDC_ACTIVE_LAYER, LCD_FRAME_BUFFER);
	BSP_LCD_SelectLayer(LTDC_ACTIVE_LAYER);

	BSP_LCD_Clear(LCD_COLOR_BLACK);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	BSP_LCD_SetFont(&Font24);
	BSP_LCD_DisplayStringAt(1, 96, (uint8_t*) "!!WAKE-UP!!", CENTER_MODE);
#ifdef STAGE3
	/* DWT Initialization */
	dwtIpInit();
#endif	//End of #ifdef STAGE3
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
	/* USER CODE END 2 */


	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		if(MX_X_CUBE_AI_Process()==1){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
		}else HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
		/* USER CODE END WHILE */
//		while(MX_X_CUBE_AI_Process()==1){
//			if (RX_BUFFER[0] == '1') {
//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
//			} else if (RX_BUFFER[0] == '0') {
//				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
//			}
//		}
//		MX_X_CUBE_AI_Process();

		/* USER CODE BEGIN 3 */
#ifdef STAGE2
      /* Get status and positions of the touch screen */
	    if( BSP_TS_GetState(&ts_state) == TS_OK )
	    {
	      /* Touch event*/
	      if( ts_state.touchDetected > 0 && data_ready == false )
	      {
	        BSP_LCD_Clear(LCD_COLOR_BLACK);
	        data_ready = true;

	      }

	      /*Draw display*/
	      for(int i=0;i<ts_state.touchDetected;i++)
	      {
	        touch_time = HAL_GetTick();
	        BSP_LCD_FillCircle(ts_state.touchX[i],ts_state.touchY[i],PEN_POINT_SIZE);
	      }

	      /* Draw mini patch */
	      if( data_ready && (HAL_GetTick() - touch_time) > TOUCH_TIMEOUT)
	      {
	        /* retrieve downsampled image 58*32 from display memory (480*272) in SDRAM */
	        int ii = 0;

	        for(uint16_t y=16;y<BSP_LCD_GetYSize();y+=8)
	        {
	          for(uint16_t x=16;x<BSP_LCD_GetXSize();x+=8)
	          {
	            g_patch8888_5832[ii++]= BSP_LCD_ReadPixel(x,y);
	          }
	        }

	        /* Resize from 58*32 to 28*28 */
	        ImageResize((uint8_t*)g_patch8888_5832, 58, 32, 4, 0, 0, 58, 32, (uint8_t*)g_patch8888_2828, 28, 28);

	        BSP_LCD_Clear(LCD_COLOR_BLACK);

	        /* Draw 28*28 */
	        ii=0;
	        for(uint16_t y=0;y<PATCH_SIZE;y+=1)
	        {
	          for(uint16_t x=0;x<PATCH_SIZE;x+=1)
	          {
	            BSP_LCD_DrawPixel(x,y,g_patch8888_2828[ii++]);
	          }
	        }
	        data_ready = false;
	      }
	    }
#endif //End of #ifdef STAGE2
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 10;
	RCC_OscInitStruct.PLL.PLLN = 210;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC
			| RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_USART6 | RCC_PERIPHCLK_I2C3;
	PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
	PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
	PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
	PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV2;
	PeriphClkInitStruct.PLLSAIDivQ = 1;
	PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
	PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
	PeriphClkInitStruct.Usart6ClockSelection = RCC_USART6CLKSOURCE_PCLK2;
	PeriphClkInitStruct.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief CRC Initialization Function
 * @param None
 * @retval None
 */
static void MX_CRC_Init(void) {

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
	if (HAL_CRC_Init(&hcrc) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CRC_Init 2 */

	/* USER CODE END CRC_Init 2 */

}

/**
 * @brief DMA2D Initialization Function
 * @param None
 * @retval None
 */
static void MX_DMA2D_Init(void) {

	/* USER CODE BEGIN DMA2D_Init 0 */

	/* USER CODE END DMA2D_Init 0 */

	/* USER CODE BEGIN DMA2D_Init 1 */

	/* USER CODE END DMA2D_Init 1 */
	hdma2d.Instance = DMA2D;
	hdma2d.Init.Mode = DMA2D_M2M;
	hdma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
	hdma2d.Init.OutputOffset = 0;
	hdma2d.LayerCfg[1].InputOffset = 0;
	hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
	hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
	hdma2d.LayerCfg[1].InputAlpha = 0;
	if (HAL_DMA2D_Init(&hdma2d) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN DMA2D_Init 2 */

	/* USER CODE END DMA2D_Init 2 */

}

/**
 * @brief I2C3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C3_Init(void) {

	/* USER CODE BEGIN I2C3_Init 0 */

	/* USER CODE END I2C3_Init 0 */

	/* USER CODE BEGIN I2C3_Init 1 */

	/* USER CODE END I2C3_Init 1 */
	hi2c3.Instance = I2C3;
	hi2c3.Init.Timing = 0x00A0A3F7;
	hi2c3.Init.OwnAddress1 = 0;
	hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c3.Init.OwnAddress2 = 0;
	hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c3) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}
	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C3_Init 2 */

	/* USER CODE END I2C3_Init 2 */

}

/**
 * @brief LTDC Initialization Function
 * @param None
 * @retval None
 */
static void MX_LTDC_Init(void) {

	/* USER CODE BEGIN LTDC_Init 0 */

	/* USER CODE END LTDC_Init 0 */

	LTDC_LayerCfgTypeDef pLayerCfg = { 0 };
	LTDC_LayerCfgTypeDef pLayerCfg1 = { 0 };

	/* USER CODE BEGIN LTDC_Init 1 */

	/* USER CODE END LTDC_Init 1 */
	hltdc.Instance = LTDC;
	hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
	hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
	hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
	hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
	hltdc.Init.HorizontalSync = 7;
	hltdc.Init.VerticalSync = 3;
	hltdc.Init.AccumulatedHBP = 14;
	hltdc.Init.AccumulatedVBP = 5;
	hltdc.Init.AccumulatedActiveW = 654;
	hltdc.Init.AccumulatedActiveH = 485;
	hltdc.Init.TotalWidth = 660;
	hltdc.Init.TotalHeigh = 487;
	hltdc.Init.Backcolor.Blue = 0;
	hltdc.Init.Backcolor.Green = 0;
	hltdc.Init.Backcolor.Red = 0;
	if (HAL_LTDC_Init(&hltdc) != HAL_OK) {
		Error_Handler();
	}
	pLayerCfg.WindowX0 = 0;
	pLayerCfg.WindowX1 = 0;
	pLayerCfg.WindowY0 = 0;
	pLayerCfg.WindowY1 = 0;
	pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
	pLayerCfg.Alpha = 0;
	pLayerCfg.Alpha0 = 0;
	pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
	pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
	pLayerCfg.FBStartAdress = 0;
	pLayerCfg.ImageWidth = 0;
	pLayerCfg.ImageHeight = 0;
	pLayerCfg.Backcolor.Blue = 0;
	pLayerCfg.Backcolor.Green = 0;
	pLayerCfg.Backcolor.Red = 0;
	if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK) {
		Error_Handler();
	}
	pLayerCfg1.WindowX0 = 0;
	pLayerCfg1.WindowX1 = 0;
	pLayerCfg1.WindowY0 = 0;
	pLayerCfg1.WindowY1 = 0;
	pLayerCfg1.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
	pLayerCfg1.Alpha = 0;
	pLayerCfg1.Alpha0 = 0;
	pLayerCfg1.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
	pLayerCfg1.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
	pLayerCfg1.FBStartAdress = 0;
	pLayerCfg1.ImageWidth = 0;
	pLayerCfg1.ImageHeight = 0;
	pLayerCfg1.Backcolor.Blue = 0;
	pLayerCfg1.Backcolor.Green = 0;
	pLayerCfg1.Backcolor.Red = 0;
	if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg1, 1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN LTDC_Init 2 */

	/* USER CODE END LTDC_Init 2 */

}

/**
 * @brief TIM6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM6_Init(void) {

	/* USER CODE BEGIN TIM6_Init 0 */

	/* USER CODE END TIM6_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM6_Init 1 */

	/* USER CODE END TIM6_Init 1 */
	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 4199;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 9999;
	htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM6_Init 2 */

	/* USER CODE END TIM6_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART6_UART_Init(void) {

	/* USER CODE BEGIN USART6_Init 0 */

	/* USER CODE END USART6_Init 0 */

	/* USER CODE BEGIN USART6_Init 1 */

	/* USER CODE END USART6_Init 1 */
	huart6.Instance = USART6;
	huart6.Init.BaudRate = 9600;
	huart6.Init.WordLength = UART_WORDLENGTH_8B;
	huart6.Init.StopBits = UART_STOPBITS_1;
	huart6.Init.Parity = UART_PARITY_NONE;
	huart6.Init.Mode = UART_MODE_TX_RX;
	huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart6.Init.OverSampling = UART_OVERSAMPLING_16;
	huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart6) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART6_Init 2 */

	/* USER CODE END USART6_Init 2 */

}

/* FMC initialization function */
static void MX_FMC_Init(void) {

	/* USER CODE BEGIN FMC_Init 0 */

	/* USER CODE END FMC_Init 0 */

	FMC_SDRAM_TimingTypeDef SdramTiming = { 0 };

	/* USER CODE BEGIN FMC_Init 1 */

	/* USER CODE END FMC_Init 1 */

	/** Perform the SDRAM1 memory initialization sequence
	 */
	hsdram1.Instance = FMC_SDRAM_DEVICE;
	/* hsdram1.Init */
	hsdram1.Init.SDBank = FMC_SDRAM_BANK1;
	hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
	hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
	hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
	hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
	hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_1;
	hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
	hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_DISABLE;
	hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
	hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
	/* SdramTiming */
	SdramTiming.LoadToActiveDelay = 16;
	SdramTiming.ExitSelfRefreshDelay = 16;
	SdramTiming.SelfRefreshTime = 16;
	SdramTiming.RowCycleDelay = 16;
	SdramTiming.WriteRecoveryTime = 16;
	SdramTiming.RPDelay = 16;
	SdramTiming.RCDDelay = 16;

	if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK) {
		Error_Handler();
	}

	/* USER CODE BEGIN FMC_Init 2 */

	/* USER CODE END FMC_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOJ_CLK_ENABLE();
	__HAL_RCC_GPIOI_CLK_ENABLE();
	__HAL_RCC_GPIOK_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin,
			GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOI,
	ARDUINO_D7_Pin | ARDUINO_D8_Pin | GPIO_PIN_1 | LCD_DISP_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_Port, LCD_BL_CTRL_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(DCMI_PWR_EN_GPIO_Port, DCMI_PWR_EN_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOG, ARDUINO_D4_Pin | ARDUINO_D2_Pin | EXT_RST_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin : OTG_HS_OverCurrent_Pin */
	GPIO_InitStruct.Pin = OTG_HS_OverCurrent_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(OTG_HS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : QSPI_D2_Pin */
	GPIO_InitStruct.Pin = QSPI_D2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
	HAL_GPIO_Init(QSPI_D2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : RMII_TXD1_Pin RMII_TXD0_Pin RMII_TX_EN_Pin */
	GPIO_InitStruct.Pin = RMII_TXD1_Pin | RMII_TXD0_Pin | RMII_TX_EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/*Configure GPIO pins : ARDUINO_SCL_D15_Pin ARDUINO_SDA_D14_Pin */
	GPIO_InitStruct.Pin = ARDUINO_SCL_D15_Pin | ARDUINO_SDA_D14_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : ULPI_D7_Pin ULPI_D6_Pin ULPI_D5_Pin ULPI_D3_Pin
	 ULPI_D2_Pin ULPI_D1_Pin ULPI_D4_Pin */
	GPIO_InitStruct.Pin = ULPI_D7_Pin | ULPI_D6_Pin | ULPI_D5_Pin | ULPI_D3_Pin
			| ULPI_D2_Pin | ULPI_D1_Pin | ULPI_D4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : ARDUINO_PWM_D3_Pin */
	GPIO_InitStruct.Pin = ARDUINO_PWM_D3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
	HAL_GPIO_Init(ARDUINO_PWM_D3_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : SPDIF_RX0_Pin */
	GPIO_InitStruct.Pin = SPDIF_RX0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF8_SPDIFRX;
	HAL_GPIO_Init(SPDIF_RX0_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : SDMMC_CK_Pin SDMMC_D3_Pin SDMMC_D2_Pin PC9
	 PC8 */
	GPIO_InitStruct.Pin = SDMMC_CK_Pin | SDMMC_D3_Pin | SDMMC_D2_Pin
			| GPIO_PIN_9 | GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : ARDUINO_PWM_D9_Pin */
	GPIO_InitStruct.Pin = ARDUINO_PWM_D9_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
	HAL_GPIO_Init(ARDUINO_PWM_D9_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : DCMI_D6_Pin DCMI_D7_Pin */
	GPIO_InitStruct.Pin = DCMI_D6_Pin | DCMI_D7_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pin : QSPI_NCS_Pin */
	GPIO_InitStruct.Pin = QSPI_NCS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
	HAL_GPIO_Init(QSPI_NCS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : OTG_FS_VBUS_Pin */
	GPIO_InitStruct.Pin = OTG_FS_VBUS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(OTG_FS_VBUS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : Audio_INT_Pin */
	GPIO_InitStruct.Pin = Audio_INT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(Audio_INT_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : OTG_FS_P_Pin OTG_FS_N_Pin OTG_FS_ID_Pin */
	GPIO_InitStruct.Pin = OTG_FS_P_Pin | OTG_FS_N_Pin | OTG_FS_ID_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : SAI2_MCLKA_Pin SAI2_SCKA_Pin SAI2_FSA_Pin SAI2_SDA_Pin */
	GPIO_InitStruct.Pin = SAI2_MCLKA_Pin | SAI2_SCKA_Pin | SAI2_FSA_Pin
			| SAI2_SDA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF10_SAI2;
	HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

	/*Configure GPIO pin : SAI2_SDB_Pin */
	GPIO_InitStruct.Pin = SAI2_SDB_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF10_SAI2;
	HAL_GPIO_Init(SAI2_SDB_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
	GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : DCMI_D5_Pin */
	GPIO_InitStruct.Pin = DCMI_D5_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
	HAL_GPIO_Init(DCMI_D5_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : ARDUINO_D7_Pin ARDUINO_D8_Pin PI1 LCD_DISP_Pin */
	GPIO_InitStruct.Pin = ARDUINO_D7_Pin | ARDUINO_D8_Pin | GPIO_PIN_1
			| LCD_DISP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

	/*Configure GPIO pin : uSD_Detect_Pin */
	GPIO_InitStruct.Pin = uSD_Detect_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(uSD_Detect_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LCD_BL_CTRL_Pin */
	GPIO_InitStruct.Pin = LCD_BL_CTRL_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LCD_BL_CTRL_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : DCMI_VSYNC_Pin */
	GPIO_InitStruct.Pin = DCMI_VSYNC_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
	HAL_GPIO_Init(DCMI_VSYNC_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
	GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : SDMMC_D0_Pin */
	GPIO_InitStruct.Pin = SDMMC_D0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
	HAL_GPIO_Init(SDMMC_D0_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : TP3_Pin NC2_Pin */
	GPIO_InitStruct.Pin = TP3_Pin | NC2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

	/*Configure GPIO pin : DCMI_PWR_EN_Pin */
	GPIO_InitStruct.Pin = DCMI_PWR_EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(DCMI_PWR_EN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : DCMI_D4_Pin DCMI_D3_Pin DCMI_D0_Pin DCMI_D2_Pin
	 DCMI_D1_Pin */
	GPIO_InitStruct.Pin = DCMI_D4_Pin | DCMI_D3_Pin | DCMI_D0_Pin | DCMI_D2_Pin
			| DCMI_D1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
	HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

	/*Configure GPIO pin : ARDUINO_PWM_CS_D5_Pin */
	GPIO_InitStruct.Pin = ARDUINO_PWM_CS_D5_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
	HAL_GPIO_Init(ARDUINO_PWM_CS_D5_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PA8 */
	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : LCD_INT_Pin */
	GPIO_InitStruct.Pin = LCD_INT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(LCD_INT_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : ULPI_NXT_Pin */
	GPIO_InitStruct.Pin = ULPI_NXT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
	HAL_GPIO_Init(ULPI_NXT_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : ARDUINO_D4_Pin ARDUINO_D2_Pin EXT_RST_Pin */
	GPIO_InitStruct.Pin = ARDUINO_D4_Pin | ARDUINO_D2_Pin | EXT_RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/*Configure GPIO pins : ARDUINO_A4_Pin ARDUINO_A5_Pin ARDUINO_A1_Pin ARDUINO_A2_Pin
	 ARDUINO_A3_Pin */
	GPIO_InitStruct.Pin = ARDUINO_A4_Pin | ARDUINO_A5_Pin | ARDUINO_A1_Pin
			| ARDUINO_A2_Pin | ARDUINO_A3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	/*Configure GPIO pins : ULPI_STP_Pin ULPI_DIR_Pin */
	GPIO_InitStruct.Pin = ULPI_STP_Pin | ULPI_DIR_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
	GPIO_InitStruct.Pin = RMII_MDC_Pin | RMII_RXD0_Pin | RMII_RXD1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PB2 */
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : QSPI_D1_Pin QSPI_D3_Pin QSPI_D0_Pin */
	GPIO_InitStruct.Pin = QSPI_D1_Pin | QSPI_D3_Pin | QSPI_D0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pin : RMII_RXER_Pin */
	GPIO_InitStruct.Pin = RMII_RXER_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(RMII_RXER_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
	GPIO_InitStruct.Pin = RMII_REF_CLK_Pin | RMII_MDIO_Pin | RMII_CRS_DV_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : ARDUINO_A0_Pin */
	GPIO_InitStruct.Pin = ARDUINO_A0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(ARDUINO_A0_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : DCMI_HSYNC_Pin PA6 */
	GPIO_InitStruct.Pin = DCMI_HSYNC_Pin | GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : ULPI_CLK_Pin ULPI_D0_Pin */
	GPIO_InitStruct.Pin = ULPI_CLK_Pin | ULPI_D0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : ARDUINO_PWM_D6_Pin */
	GPIO_InitStruct.Pin = ARDUINO_PWM_D6_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF9_TIM12;
	HAL_GPIO_Init(ARDUINO_PWM_D6_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : ARDUINO_MISO_D12_Pin ARDUINO_MOSI_PWM_D11_Pin */
	GPIO_InitStruct.Pin = ARDUINO_MISO_D12_Pin | ARDUINO_MOSI_PWM_D11_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
