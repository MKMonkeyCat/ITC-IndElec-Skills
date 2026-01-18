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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
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
uint8_t  mode, old_mode;
uint16_t led_r, led_b, led_g;
uint8_t  node_mode;
uint8_t  software_pwm_count, software_pwm_th;

__IO uint16_t time_count1, time_count2;
__IO uint8_t  display_count = 0;
uint8_t       display_index;

// The ADXL335 output is ratiometric, therefore, the output
// sensitivity (or scale factor) varies proportionally to the
// supply voltage. At VS = 3.6 V, the output sensitivity is typically 360 mV/g. At VS = 2 V, the output sensitivity is
// typically 195 mV/g.
// use 3.3v 1g ~= 0.33v
const float ONE_G_SENSOR_VALUE = 0.33f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint16_t calcLedFromAngle(int angle);
void     baseMode5(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t adc[5];
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
  MX_ADC_Init();
  MX_TIM6_Init();
  MX_TIM22_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_PWM_Start(&htim22, TIM_CHANNEL_1);

  HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc, (uint32_t *)adc, 5);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    mode = (GPIOB->IDR >> 2) & 0x7;
    // mode = 5;
    if (mode != old_mode) {
      old_mode    = mode;
      time_count1 = time_count2 = node_mode = led_b = led_r = led_g = 0;

      TIM21->CCR1 = 0;  // stop buzzer

      switch (mode) {
        case 1: {
          led_b = 1 << 5;
          led_r = 1 << (5 - 1);
        } break;
        case 2: {
          GPIOC->ODR = 1;
        } break;
      }
    }

    switch (mode) {
      case 1: {
        if (!time_count1) {
          time_count1 = 1000 * 10;  // 1s

          led_r <<= 1;
          if (led_r >> 12) led_r = 1;
          else if (led_r >> 11) led_b <<= 1;
          if (led_b >> 12) led_b = 1;
        }
      } break;

      case 2: {
        if (!time_count1) {
          software_pwm_count = (software_pwm_count + 1) % 10;
          time_count1        = 1;  // 0.1ms
        }

        if (!time_count2) {
          if (node_mode % 2) {
            software_pwm_th++;
            if (software_pwm_th > 10) node_mode++;
          } else {
            software_pwm_th--;
            if (software_pwm_th == 0xff) {
              node_mode++;
              software_pwm_th = 0;
            }
          }
          node_mode %= 5;
          time_count2 = 100 * 10;  // 100ms
        }

        uint8_t status1 = node_mode > 2;  // 0, 1
        uint8_t status2 = software_pwm_count < software_pwm_th;
        HAL_GPIO_WritePin(LR_COM_GPIO_Port, LR_COM_Pin, !status1 && status2);
        HAL_GPIO_WritePin(LB_COM_GPIO_Port, LB_COM_Pin, status1 && status2);
        // led_r = software_pwm_count > software_pwm_th ? 0xfff : 0;
      } break;

      case 3: {
        float voltage_x = adc[0] * 3.3f / 4096;

        float x_g = (voltage_x - 1.65f) / ONE_G_SENSOR_VALUE;
        led_r = led_b = led_g = 0;
        if (x_g < -0.7) led_r = 1 << 2;       // LR3
        else if (x_g < -0.3) led_b = 1 << 2;  // LB3
        else if (x_g < 0.3) led_g = 1 << 0;   // LG1
        else if (x_g < 0.7) led_b = 1 << 8;   // LB9
        else led_r = 1 << 8;                  // LR9
      } break;

      case 4: {
        float voltage_x_int = adc[3] * 3.3f / 4096;

        if (voltage_x_int < 0.8) led_r = 1 << 8;
        else if (voltage_x_int < 1.2) led_b = 1 << 8;
        else if (voltage_x_int < 2.0) led_g = 1;
        else if (voltage_x_int < 2.4) led_b = 1 << 2;
        else led_r = 1 << 2;
      } break;

      case 5:
      case 6: {
        baseMode5();

        if (mode == 5) break;

        uint16_t mask = 1 << node_mode;

        if (!time_count2) {
          TIM22->CCR1 = 0;

          if (led_r & mask || led_b & mask) {  // 判斷重疊
            time_count2 = 200 * 10;            // 200ms
            TIM22->CCR1 = 127;
          }
        }

        led_r |= mask;
        led_b |= mask;

        if (!time_count1) {
          time_count1 = 200 * 10;  // 200ms

          node_mode = (node_mode + 1) % 12;  // 0-11 [1~12 LEDs]
        }
      } break;

      case 7: {
        float voltage_x = adc[0] * 3.3f / 4096;
        float voltage_y = adc[1] * 3.3f / 4096;

        float x_g  = (voltage_x - 1.65f) / ONE_G_SENSOR_VALUE;
        float y_g  = (voltage_y - 1.65f) / ONE_G_SENSOR_VALUE;
        float xy_g = sqrtf(x_g * x_g + y_g * y_g);

        led_r = led_b = led_g = 0;

        if (xy_g < 0.3) {
          led_r = 0xfff;  // all on
          led_b = (1 << 2) | (1 << 5) | (1 << 8) | (1 << 11);
          led_g = 1;
        } else if (xy_g < 0.7) {  // off all led (do nothing)
        } else {
          float    angle = atan2f(y_g, x_g) / 3.1415926f * 180.0f;
          uint16_t led   = calcLedFromAngle(angle);

          led_g = 1;
          led_r = led_b = led;
        }
      } break;
    }

    if (!display_count && mode != 2) {
      HAL_GPIO_WritePin(LR_COM_GPIO_Port, LR_COM_Pin, display_index);   // LR
      HAL_GPIO_WritePin(LB_COM_GPIO_Port, LB_COM_Pin, !display_index);  // LB
      GPIOC->ODR    = (~(display_index ? led_r : led_b) & 0xfff) << 1 | !led_g;
      display_index = !display_index;
      display_count = 10 * 10;
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
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV          = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
uint16_t calcLedFromAngle(int angle) {
  if (angle < -15) angle += 360 - 15;
  if (angle < -15)
    ;
  else if (angle < 15) return 1 << 2;
  else if (angle < 45) return 1 << 1;
  else if (angle < 75) return 1 << 0;
  else if (angle < 105) return 1 << 11;
  else if (angle < 135) return 1 << 10;
  else if (angle < 165) return 1 << 9;
  else if (angle < 195) return 1 << 8;
  else if (angle < 225) return 1 << 7;
  else if (angle < 255) return 1 << 6;
  else if (angle < 285) return 1 << 5;
  else if (angle < 315) return 1 << 4;
  else if (angle < 345) return 1 << 3;

  return 0;
}

void baseMode5(void) {
  float voltage_x = adc[0] * 3.3f / 4096;
  float voltage_y = adc[1] * 3.3f / 4096;

  float x_g  = (voltage_x - 1.65f) / ONE_G_SENSOR_VALUE;
  float y_g  = (voltage_y - 1.65f) / ONE_G_SENSOR_VALUE;
  float xy_g = sqrtf(x_g * x_g + y_g * y_g);

  led_r = led_b = led_g = 0;

  if (xy_g < 0.3) led_g = 1;
  else {
    float    angle = atan2f(y_g, x_g) / 3.1415926f * 180.0f;
    uint16_t led   = calcLedFromAngle(angle);

    if (xy_g < 0.7) led_b = led;
    else led_r = led;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  // every 0.1ms
  if (htim->Instance == TIM6) {
    if (time_count1 > 0) time_count1--;
    if (time_count2 > 0) time_count2--;
    if (display_count > 0) display_count--;
  }
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
