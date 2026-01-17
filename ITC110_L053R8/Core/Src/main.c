/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
enum Statue {
  IDLE,              // 閒置
  SELECTED,          // 已選擇難度
  WAITING,           // 等待任一 摩斯訊號輸入
  READING,           // 讀取中 (摩斯訊號輸入中)
  WAITING_FORINPUT,  // 等待 3T('_' 字元分割) 判斷期間內有無新輸入
  DONE,              // 完成輸入 buffer 已滿或碰到 '='
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define T_MS 100
#define MAX_SYMBOLS 10

const uint16_t DS_PINS[6]   = {DS1_Pin, DS2_Pin, DS3_Pin, DS4_Pin, DS5_Pin, DS6_Pin};
const uint16_t DATA_PINS[8] = {
    DATA_A_Pin, DATA_B_Pin, DATA_C_Pin, DATA_D_Pin, DATA_E_Pin, DATA_F_Pin, DATA_G_Pin, DATA_DP_Pin};
const uint16_t SW_PINS[5] = {SW1_Pin, SW2_Pin, SW3_Pin, SW4_Pin, SW5_Pin};

uint8_t SEG_TABLE[] = {
    0x3F,  // 0
    0x06,  // 1
    0x5B,  // 2
    0x4F,  // 3
    0x66,  // 4
    0x6D,  // 5
    0x7D,  // 6
    0x07,  // 7
    0x7F,  // 8
    0x6F,  // 9
    0x00,  // blank
    0x40,  // -
};

uint8_t DECODE_TABLE[][2] = {
    {0b100000, 0},  // 0
    {0b110000, 1},  // 1
    {0b111000, 2},  // 2
    {0b111100, 3},  // 3
    {0b111110, 4},  // 4
    {0b111111, 5},  // 5
    {0b101111, 6},  // 6
    {0b100111, 7},  // 7
    {0b100011, 8},  // 8
    {0b100001, 9},  // 9
    {0b1100, '+'},  // +
    {0b1110, '-'},  // -
    {0b1011, '*'},  // *
    {0b1001, '/'},  // /
    {0b1111, '='},  // =
};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
enum Statue current_state = IDLE;

uint8_t ds_index = 0, level = 0;

uint8_t symbol_index         = 0;
uint8_t symbol_buffer        = 0;
uint8_t symbols[MAX_SYMBOLS] = {0};

uint8_t display_buffer[6] = {10, 10, 10, 10, 10, 10};  // all blank

uint8_t  sw_count      = 0;
uint8_t  sw_buff[5]    = {0};
uint16_t last_sw_state = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void    resetState(void);
void    resetDisplay(void);
int     calcData(uint8_t *data);
void    morseScan(void);
uint8_t morseDecode(uint8_t symbol_buffer);
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
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim6);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    for (int i = 0; i < 4; i++) {
      if (sw_buff[i] && (level != i + 1 || current_state != SELECTED)) {
        resetState();
        level = i + 1;
        resetDisplay();
        display_buffer[5] = sw_buff[i] ? i + 1 : 10;
        current_state     = SELECTED;
        break;
      }
    }

    if ((current_state == SELECTED || current_state == IDLE) && sw_buff[4]) {
      resetState();
      resetDisplay();
      current_state     = WAITING;
      display_buffer[5] = 10;
    }

    if (current_state == WAITING || current_state == READING || current_state == WAITING_FORINPUT) {
      morseScan();
    } else if (current_state == DONE) {
      int value = calcData(symbols);

      uint8_t is_negative = value < 0;
      if (is_negative) value *= -1;
      for (int i = 0; i < 6; i++) {
        display_buffer[i] = value % 10;
        value /= 10;
        if (value == 0) {
          if (is_negative && i < 5) {
            display_buffer[i + 1] = 11;  // '-'
          }
          break;
        }
      }

      current_state = IDLE;
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
  RCC_OscInitTypeDef       RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef       RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit     = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void segScan(void) {
  for (int i = 0; i < 6; i++) {
    HAL_GPIO_WritePin(DS1_GPIO_Port, DS_PINS[i], i != ds_index);
  }

  uint8_t seg_data = SEG_TABLE[display_buffer[ds_index]];
  for (int i = 0; i < 8; i++) {
    HAL_GPIO_WritePin(DATA_A_GPIO_Port, DATA_PINS[i], !((seg_data >> i) & 1));
  }

  ds_index = (ds_index + 1) % 6;
}

void resetState(void) {
  level         = 0;
  current_state = IDLE;
  symbol_index  = 0;
  symbol_buffer = 0;
  for (int i = 0; i < MAX_SYMBOLS; i++) {
    symbols[i] = 0;
  }
}

void resetDisplay(void) {
  for (int i = 0; i < 6; i++) {
    display_buffer[i] = 10;  // blank
  }
}

int getNumber(uint8_t *data, int *index) {
  int value = 0;
  while (*index < MAX_SYMBOLS && data[*index] != 0x0f) {
    uint8_t decoded = morseDecode(data[*index]);
    if (decoded <= 9) {
      value = value * 10 + decoded;
      (*index)++;
    } else {
      break;
    }
  }
  return value;
}

int getNextTerm(uint8_t *data, int *index) {
  int value = getNumber(data, index);

  // 處理乘除
  while (*index < MAX_SYMBOLS && data[*index] != 0x0f) {
    uint8_t op = morseDecode(data[*index]);
    if (op == '*' || op == '/') {
      (*index)++;  // 跳過運算子

      // 獲取下一個數字
      int nextVal = getNumber(data, index);
      if (op == '*') value *= nextVal;
      else if (op == '/') {
        if (nextVal == 0) return 0;  // 應處理除以零錯誤
        value /= nextVal;
      }
    } else {
      break;  // 遇到加減，回給上一層處理
    }
  }
  return value;
}

int calcData(uint8_t *data) {
  int index       = 0;
  int totalResult = getNextTerm(data, &index);

  while (index < MAX_SYMBOLS && data[index] != 0x0f) {
    uint8_t op = morseDecode(data[index]);
    index++;

    int nextTerm = getNextTerm(data, &index);

    if (op == '+') totalResult += nextTerm;
    else if (op == '-') totalResult -= nextTerm;
  }

  return totalResult;
}

uint8_t morseDecode(uint8_t symbol_buffer) {
  for (int i = 0; i < 15; i++) {
    if (DECODE_TABLE[i][0] == symbol_buffer) {
      return DECODE_TABLE[i][1];
    }
  }

  return (uint8_t)-1;
}

void morseScan(void) {
  static uint32_t pulse_start_time = 0, pulse_end_time = 0;
  static uint32_t last_fall_time    = 0;
  static uint8_t  last_signal_state = 0;

  uint8_t  single = HAL_GPIO_ReadPin(MORSE_SIGNAL_GPIO_Port, MORSE_SIGNAL_Pin);
  uint32_t now    = HAL_GetTick();

  if (last_signal_state == single) {
    if (last_fall_time == 0) return;

    uint32_t elapsed = now - last_fall_time;
    // 1/400Hz -> 2.27ms use 4ms
    if (current_state == READING && elapsed > 4) {  // 脈波訊號消失，結束讀取
      current_state  = WAITING_FORINPUT;
      pulse_end_time = now;

      uint32_t pulse_duration = pulse_end_time - pulse_start_time;
      if (pulse_duration >= 2.5 * T_MS) {  // 3T
        // 保證第一位為 1，避免前置零被忽略
        if (symbol_buffer == 0) symbol_buffer |= 1;
        symbol_buffer = (symbol_buffer << 1) | 0;  // dash
      } else if (pulse_duration >= 0.5 * T_MS) {
        // 保證第一位為 1，避免前置零被忽略
        if (symbol_buffer == 0) symbol_buffer |= 1;
        symbol_buffer = (symbol_buffer << 1) | 1;  // dot
      } else {
        current_state = WAITING;
      }
    } else if (current_state == WAITING_FORINPUT && elapsed >= 2.5 * T_MS) {  // 超過 3T 無訊號輸入，視為字元結束
      if (symbol_index < MAX_SYMBOLS) {
        symbols[symbol_index++] = symbol_buffer;  // 儲存字元
      } else {
        current_state = DONE;
      }

      // '=' 結束符號 0b1_111
      if (symbol_buffer == 0x0f || current_state == DONE) {
        current_state = DONE;
      } else {
        current_state = WAITING;
      }

      symbol_buffer = 0;
    }

    return;
  }
  last_signal_state = single;

  if (single) {
    if (current_state == WAITING || current_state == WAITING_FORINPUT) {
      current_state    = READING;
      pulse_start_time = now;
    }
  } else {
    last_fall_time = now;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  // every 1ms
  if (htim->Instance == TIM6) {
    segScan();

    if (++sw_count >= 5) {
      uint16_t idr = SW1_GPIO_Port->IDR & (SW1_Pin | SW2_Pin | SW3_Pin | SW4_Pin | SW5_Pin);
      if (idr != last_sw_state) {
        for (int i = 0; i < 5; i++) {
          if ((idr ^ last_sw_state) & SW_PINS[i]) {
            sw_buff[i] = (idr & SW_PINS[i]) == 0;
          }
        }
      }
      last_sw_state = idr;
      sw_count      = 0;
    }
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
