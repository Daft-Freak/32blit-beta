/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fatfs.h"
#include "hrtim.h"
#include "i2c.h"
#include "jpeg.h"
#include "quadspi.h"
#include "rng.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"

#include "gpio.hpp"
#include "display.hpp"
#include "sound.hpp"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "32blit.h"
#include "32blit.hpp"
#include "graphics/color.hpp"
#include "CDCResetHandler.h"
#include "CDCInfoHandler.h"
#include "CDCCommandStream.h"
#include "USBManager.h"

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
extern USBManager g_usbManager;
extern CDCCommandStream g_commandStream;
CDCResetHandler g_resetHandler;
CDCInfoHandler g_infoHandler;
bool is_beta_unit = false;


uint8_t charge_led_r = 0;
uint8_t charge_led_g = 0;
uint8_t charge_led_b = 0;

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
  gpio::init();
  sound::init();

  is_beta_unit = HAL_GPIO_ReadPin(VERSION_GPIO_Port, VERSION_Pin);

  //MX_GPIO_Init();

  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  //MX_DAC1_Init();
  MX_HRTIM_Init();
  MX_I2C4_Init();
#if (INITIALISE_QSPI==1)
  MX_QUADSPI_Init();
#endif
  MX_ADC1_Init();
  MX_ADC3_Init();
  //MX_USB_OTG_HS_USB_Init();
  MX_SPI1_Init();
  MX_SPI4_Init();
  //MX_TIM6_Init();
  MX_TIM15_Init();
  //MX_TIM16_Init();
  MX_FATFS_Init();
  MX_RNG_Init();
  MX_USB_DEVICE_Init();
  MX_JPEG_Init();
  /* USER CODE BEGIN 2 */

  //NVIC_SetPriority(SysTick_IRQn, 0x0);

#if (INITIALISE_QSPI==1)
  qspi_init();
#endif

  blit_init();

  // add CDC handler to reset device on receiving "_RST" and "SWIT"
	g_commandStream.AddCommandHandler(CDCCommandHandler::CDCFourCCMake<'_', 'R', 'S', 'T'>::value, &g_resetHandler);
	g_commandStream.AddCommandHandler(CDCCommandHandler::CDCFourCCMake<'S', 'W', 'I', 'T'>::value, &g_resetHandler);

  // add CDC handler to log info device on receiving "INFO"
	g_commandStream.AddCommandHandler(CDCCommandHandler::CDCFourCCMake<'I', 'N', 'F', 'O'>::value, &g_infoHandler);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t t_start = blit::now();

//    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
//    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
//    uint32_t uTicksPerUs = SystemCoreClock / 1000000;
//    uint32_t uTimeStart = DWT->CYCCNT;

    blit_tick();
    //HAL_Delay(1000);

    // USB
    g_usbManager.Update();

    if(USB_GetMode(USB_OTG_HS))
      MX_USB_HOST_Process();

    // handle CDC input
    g_commandStream.Stream();

    uint32_t t_elapsed = blit::now() - t_start;
//    uint32_t uTimeElapsed = (DWT->CYCCNT - uTimeStart)/uTicksPerUs;
//    printf("%lu, %lu\n\r", t_elapsed, uTimeElapsed);
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_4) {}

  LL_PWR_ConfigSupply(LL_PWR_LDO_SUPPLY);
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE0);
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1) {}

  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1) {}

  LL_RCC_HSI_SetCalibTrimming(32);
  LL_RCC_HSI_SetDivider(LL_RCC_HSI_DIV1);
  LL_RCC_HSI48_Enable();

  /* Wait till HSI48 is ready */
  while(LL_RCC_HSI48_IsReady() != 1) {}

  LL_RCC_PLL_SetSource(LL_RCC_PLLSOURCE_HSE);
  LL_RCC_PLL1P_Enable();
  LL_RCC_PLL1Q_Enable();
  LL_RCC_PLL1R_Enable();
  LL_RCC_PLL1_SetVCOInputRange(LL_RCC_PLLINPUTRANGE_8_16);
  LL_RCC_PLL1_SetVCOOutputRange(LL_RCC_PLLVCORANGE_WIDE);
  LL_RCC_PLL1_SetM(1);
  LL_RCC_PLL1_SetN(80);
  LL_RCC_PLL1_SetP(2);
  LL_RCC_PLL1_SetQ(2);
  LL_RCC_PLL1_SetR(2);
  LL_RCC_PLL1_Enable();

  /* Wait till PLL is ready */
  while(LL_RCC_PLL1_IsReady() != 1) {}

  LL_RCC_PLL2P_Enable();
  LL_RCC_PLL2_SetVCOInputRange(LL_RCC_PLLINPUTRANGE_8_16);
  LL_RCC_PLL2_SetVCOOutputRange(LL_RCC_PLLVCORANGE_MEDIUM);
  LL_RCC_PLL2_SetM(1);
  LL_RCC_PLL2_SetN(12);
  LL_RCC_PLL2_SetP(1);
  LL_RCC_PLL2_SetQ(2);
  LL_RCC_PLL2_SetR(2);
  LL_RCC_PLL2_SetFRACN(4096);
  LL_RCC_PLL2FRACN_Enable();
  LL_RCC_PLL2_Enable();

  /* Wait till PLL is ready */
  while(LL_RCC_PLL2_IsReady() != 1) {}

  LL_RCC_PLL3R_Enable();
  LL_RCC_PLL3_SetVCOInputRange(LL_RCC_PLLINPUTRANGE_2_4);
  LL_RCC_PLL3_SetVCOOutputRange(LL_RCC_PLLVCORANGE_WIDE);
  LL_RCC_PLL3_SetM(6);
  LL_RCC_PLL3_SetN(129);
  LL_RCC_PLL3_SetP(2);
  LL_RCC_PLL3_SetQ(2);
  LL_RCC_PLL3_SetR(53);
  LL_RCC_PLL3_Enable();

  /* Wait till PLL is ready */
  while(LL_RCC_PLL3_IsReady() != 1) {}

  /* Intermediate AHB prescaler 2 when target frequency clock is higher than 80 MHz */
  LL_RCC_SetAHBPrescaler(LL_RCC_AHB_DIV_2);

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL1);
  LL_RCC_SetSysPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAHBPrescaler(LL_RCC_AHB_DIV_2);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
  LL_RCC_SetAPB3Prescaler(LL_RCC_APB3_DIV_2);
  LL_RCC_SetAPB4Prescaler(LL_RCC_APB4_DIV_2);
  LL_SetSystemCoreClock(480000000);

  /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_CRS_SetSyncDivider(LL_CRS_SYNC_DIV_1);
  LL_CRS_SetSyncPolarity(LL_CRS_SYNC_POLARITY_RISING);
  LL_CRS_SetSyncSignalSource(LL_CRS_SYNC_SOURCE_USB);
  LL_CRS_SetReloadCounter(__LL_CRS_CALC_CALCULATE_RELOADVALUE(48000000, 1000));
  LL_CRS_SetFreqErrorLimit(34);
  LL_CRS_SetHSI48SmoothTrimming(32);

  LL_RCC_SetQSPIClockSource(LL_RCC_QSPI_CLKSOURCE_HCLK);
  LL_RCC_SetRNGClockSource(LL_RCC_RNG_CLKSOURCE_HSI48);
  LL_RCC_SetI2CClockSource(LL_RCC_I2C4_CLKSOURCE_PCLK4);
  LL_RCC_SetSPIClockSource(LL_RCC_SPI123_CLKSOURCE_PLL1Q);
  LL_RCC_SetSPIClockSource(LL_RCC_SPI45_CLKSOURCE_HSI);
  LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSOURCE_PLL2P);
  LL_RCC_SetUSBClockSource(LL_RCC_USB_CLKSOURCE_HSI48);
  LL_RCC_SetHRTIMClockSource(LL_RCC_HRTIM_CLKSOURCE_CPU);

  __HAL_RCC_DMA2D_CLK_ENABLE();

  /** Enable USB Voltage detector
  */
  HAL_PWREx_EnableUSBVoltageDetector();
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
