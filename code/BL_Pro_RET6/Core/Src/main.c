/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : PWM 裸测 —— 仅 TIM1 + TIM4，屏蔽所有其他功能
  ******************************************************************************
  * Copyright (c) 2026 STMicroelectronics. All rights reserved.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sys.h"   /* 保留：INT.c 中有 extern AS5047P_Handle encoder_right */
#include <math.h>
#include "stm32g4xx_hal.h"
#include "AS5047P_RW.h"
#include "sensor.h"
#include "BLDCMotor.h"
#include "app_foc.h"
#include "app_foc_bus.h"
#include "app_foc_debug.h"
#include "app_attitude.h"
#include "usb_debug.h"
#include "icm42688p.h"
#include "debug_link.h"

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
static int16_t Main_ScaleToI16Clamped(float value, float scale)
{
  float scaled;

  scaled = value * scale;
  if (scaled > 32767.0f) {
    scaled = 32767.0f;
  } else if (scaled < -32768.0f) {
    scaled = -32768.0f;
  }
  return (int16_t)scaled;
}

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

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_SPI3_Init();
  MX_USB_Device_Init();
  MX_TIM8_Init();
  MX_SPI1_Init();
  MX_TIM5_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(2000);

  App_Attitude_Init();

  DebugLink_Init();
  if (!App_FOCStack_Init()) {
      USB_Debug_Printf("FOC stack init failed, keep bus telemetry only\r\n");
      (void)App_FOC_BusTelemetryInit();
  } else {
      if (!App_StartupCalibrate()) {
          USB_Debug_Printf("FOC startup calibrate failed, keep stack init only\r\n");
      }
  }
  App_FOCControlIT_Enable();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    App_Attitude_Loop();
    App_FOC_BusTelemetryService();

    /* 更新 DebugLink 状态快照 */
    {
        DebugLink_StatusSnapshot_t st;
        App_AttitudeTelemetry_t att_telem;
        App_FOCTelemetry_t foc_telem;
        App_Attitude_GetTelemetry(&att_telem);
        App_FOC_GetTelemetry(&foc_telem);
        st.tick_ms                  = HAL_GetTick();
        st.pitch_target_deg_x100    = (int16_t)(att_telem.pitch_target_rad * 5729.578f);
        st.speed_p_term_deg_x100    = (int16_t)(att_telem.speed_p_term_rad * 5729.578f);
        st.speed_i_term_deg_x100    = (int16_t)(att_telem.speed_i_term_rad * 5729.578f);
        st.pitch_meas_deg_x100      = (int16_t)(att_telem.pitch_meas_rad * 5729.578f);
        st.pitch_rate_dps_x100      = (int16_t)(att_telem.pitch_rate_meas_radps * 5729.578f);
        st.speed_target_radps_x100  = Main_ScaleToI16Clamped(att_telem.speed_target_radps, 100.0f);
        st.speed_meas_radps_x100    = Main_ScaleToI16Clamped(att_telem.speed_meas_radps, 100.0f);
        st.attitude_p_term_ma     = (int16_t)(att_telem.attitude_p_term_a * 1000.0f);
        st.attitude_d_term_ma     = (int16_t)(att_telem.attitude_d_term_a * 1000.0f);
        st.iq_cmd_ma                = (int16_t)(att_telem.iq_cmd_a * 1000.0f);
        st.iq_cmd_clamped_ma        = (int16_t)(att_telem.iq_cmd_clamped_a * 1000.0f);
        st.speed_output_limit_deg_x100 = (int16_t)(att_telem.speed_output_limit_rad * 5729.578f);
        st.attitude_output_limit_ma = (int16_t)(att_telem.attitude_output_limit_a * 1000.0f);
        st.iq_l_x1000               = (int16_t)(foc_telem.filtered_iq_left_a * 1000.0f);
        st.iq_r_x1000               = (int16_t)(foc_telem.filtered_iq_right_a * 1000.0f);
        st.uq_l_mv                  = (int16_t)(foc_telem.uq_left_v * 1000.0f);
        st.uq_r_mv                  = (int16_t)(foc_telem.uq_right_v * 1000.0f);
        st.bus_mv                   = (uint16_t)(foc_telem.bus_voltage_v * 1000.0f);
        st.fault_flags              = foc_telem.status_flags;
        st.speed_raw_radps_x100     = Main_ScaleToI16Clamped(att_telem.speed_raw_radps, 100.0f);
        DebugLink_UpdateStatusSnapshot(&st);
    }

    DebugLink_Process();

    //DebuginWhile();
    //Process_USB_Command();
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
