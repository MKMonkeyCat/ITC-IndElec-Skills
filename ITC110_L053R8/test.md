T = 0.1s
●(短音) 為 1 個 T 時間長度中，皆為固定頻率(440Hz)之脈波訊號
━(長音) 為 3 個 T 時間長度中，皆為固定頻率(440Hz)之脈波訊號

一個編碼後的訊號，如果是由多個●或━組成，每個●或━中間會有 1 個 T 時間長度中，皆為 Low(無聲)之訊號

_(字元間之分隔) 為字元跟字元間，當傳送 2 個以上的字元，需
要該分隔訊號，會有 3 個 T 時間長度中，皆為 Low(無聲)之訊號

若傳送一個 "ET" -> ●_━_
> ● _ ━ _
> T 3T 3T 3T
> 脈波 持續LOW 脈波 持續LOW



uint8_t  symbol_index  = 0;
uint8_t  symbol_buffer = 0;
uint8_t  symbols[6]    = {0};
uint32_t low_count     = 0;
uint32_t high_count    = 0;


void morseScan(void) {
  if (current_state != WAITING && current_state != READING) {
    return;
  }

  uint8_t single = HAL_GPIO_ReadPin(MORSE_SIGNAL_GPIO_Port, MORSE_SIGNAL_Pin);
  if (single) {
    high_count++;
    low_count = 0;
  } else {
    if (high_count > 0) {
      low_count++;
    }

    if (low_count >= T_MS * 0.8 && current_state == WAITING && high_count > 0) {
      high_count    = 0;
      current_state = READING;
      if (high_count >= 3 * T_MS * 0.8) {  // 長音 (3T)
        symbol_buffer |= 1;
        symbol_buffer = (symbol_buffer << 1) | 1;
      } else if (high_count >= T_MS * 0.8) {  // 短音 (1T)
        symbol_buffer |= 1;
        symbol_buffer <<= 1;
      }
    }

    if (current_state == READING && low_count >= 3 * T_MS * 0.8) {  // 字元分割 (3T)
      current_state = WAITING;

      symbols[symbol_index++] = symbol_buffer;
      if (symbol_buffer == 0x08) {  // '=' 結束
        symbol_index  = 0;
        current_state = IDLE;
      }

      symbol_buffer = low_count = 0;
    }
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  // every 1ms
  if (htim->Instance == TIM6) {
    morseScan();
  }
}
