#include "supvc.h"
#include "i2c.h"
#include <math.h>

TIM_HandleTypeDef htim2;

#define SUPVC_HCSR04_CHANNEL_COUNT      2U
#define SUPVC_US016_CHANNEL_COUNT       3U
#define SUPVC_KS103_CHANNEL_INDEX       5U

typedef enum {
  SUPVC_KS103_PHASE_IDLE = 0,
  SUPVC_KS103_PHASE_WAIT_TX,
  SUPVC_KS103_PHASE_WAIT_CONVERSION,
  SUPVC_KS103_PHASE_WAIT_RX
} SUPVC_KS103_Phase_t;

static volatile uint32_t supvc_echo_start[SUPVC_CHANNEL_COUNT] = {0};
static volatile uint32_t supvc_echo_width_us[SUPVC_CHANNEL_COUNT] = {0};
static volatile uint8_t supvc_echo_capture_rising[SUPVC_CHANNEL_COUNT] = {1U, 1U, 0U, 0U, 0U, 0U};
static volatile float supvc_distance_cm[SUPVC_CHANNEL_COUNT] = {
  -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f
};
static volatile uint8_t supvc_distance_valid[SUPVC_CHANNEL_COUNT] = {0, 0, 0, 0, 0, 0};
static volatile uint8_t supvc_timeout_ticks[SUPVC_CHANNEL_COUNT] = {0, 0, 0, 0, 0, 0};
static volatile uint8_t supvc_filter_initialized[SUPVC_CHANNEL_COUNT] = {0, 0, 0, 0, 0, 0};

/* 中值滤波缓冲区：每个通道保存最近5个测量值 */
#define SUPVC_MEDIAN_WINDOW_SIZE  5U
static float supvc_median_buffer[SUPVC_CHANNEL_COUNT][SUPVC_MEDIAN_WINDOW_SIZE] = {0};
static uint8_t supvc_median_index[SUPVC_CHANNEL_COUNT] = {0};
static uint8_t supvc_median_count[SUPVC_CHANNEL_COUNT] = {0};

static volatile uint8_t supvc_ks103_busy = 0U;
static volatile uint8_t supvc_ks103_data_ready = 0U;
static volatile uint16_t supvc_ks103_raw_cm = 0U;
static volatile uint8_t supvc_ks103_timeout_ticks = 0U;
static volatile uint8_t supvc_ks103_conversion_ticks = 0U;
static volatile uint8_t supvc_ks103_direct_read_mode = 0U;
static volatile uint8_t supvc_ks103_timeout_recover = 0U;
static volatile uint8_t supvc_ks103_addr_index = 0U;
static volatile uint8_t supvc_ks103_addr7_manual = 0U;
static volatile uint8_t supvc_ks103_addr7_value = SUPVC_KS103_I2C_ADDRESS_7BIT;
static volatile uint8_t supvc_ks103_alt_trigger_mode = 0U;
static volatile uint8_t supvc_ks103_comm_fail_count = 0U;
static volatile SUPVC_KS103_Phase_t supvc_ks103_phase = SUPVC_KS103_PHASE_IDLE;
static uint8_t supvc_ks103_rx_buf[2] = {0U, 0U};
static uint8_t supvc_ks103_tx_cmd_cm = 0x51U;

static const uint16_t trig_pin[SUPVC_HCSR04_CHANNEL_COUNT] = {SUPVC_TRIG1_PIN, SUPVC_TRIG2_PIN};
static GPIO_TypeDef* const trig_port[SUPVC_HCSR04_CHANNEL_COUNT] = {
  SUPVC_TRIG1_GPIO_PORT,
  SUPVC_TRIG2_GPIO_PORT
};
static const uint32_t echo_channel[SUPVC_HCSR04_CHANNEL_COUNT] = {TIM_CHANNEL_1, TIM_CHANNEL_4};

static const uint16_t us016_range_pin[SUPVC_US016_CHANNEL_COUNT] = {
  SUPVC_US016_CH3_RANGE_PIN,
  SUPVC_US016_CH4_RANGE_PIN,
  SUPVC_US016_CH5_RANGE_PIN
};
static GPIO_TypeDef* const us016_range_port[SUPVC_US016_CHANNEL_COUNT] = {
  SUPVC_US016_CH3_RANGE_GPIO_PORT,
  SUPVC_US016_CH4_RANGE_GPIO_PORT,
  SUPVC_US016_CH5_RANGE_GPIO_PORT
};
static const uint8_t us016_adc_channel[SUPVC_US016_CHANNEL_COUNT] = {
  SUPVC_US016_CH3_ADC_CHANNEL,
  SUPVC_US016_CH4_ADC_CHANNEL,
  SUPVC_US016_CH5_ADC_CHANNEL
};
static const uint8_t us016_channel_index[SUPVC_US016_CHANNEL_COUNT] = {2U, 3U, 4U};
static const GPIO_PinState us016_range_level[SUPVC_US016_CHANNEL_COUNT] = {
  GPIO_PIN_SET,   /* D3: 3m 量程 */
  GPIO_PIN_RESET, /* D4: 近距 1m 量程 */
  GPIO_PIN_RESET  /* D5: 近距 1m 量程 */
};
static const float us016_range_cm_cfg[SUPVC_US016_CHANNEL_COUNT] = {300.0f, 100.0f, 100.0f};
static const float us016_min_cm_cfg[SUPVC_US016_CHANNEL_COUNT] = {2.0f, 0.8f, 0.8f};
static const uint8_t us016_adc_samples[SUPVC_US016_CHANNEL_COUNT] = {8U, 16U, 16U};
static const float us016_alpha_cfg[SUPVC_US016_CHANNEL_COUNT] = {0.18f, 0.25f, 0.25f};
static const float us016_jump_limit_cfg[SUPVC_US016_CHANNEL_COUNT] = {35.0f, 8.0f, 8.0f};
static uint8_t supvc_us016_invalid_ticks[SUPVC_US016_CHANNEL_COUNT] = {0U, 0U, 0U};

#define SUPVC_TRIGGER_US                15U
#define SUPVC_STARTUP_DELAY_TICKS       30U
#define SUPVC_HCSR04_GAP_TICKS          6U
#define SUPVC_TIMEOUT_MAX_TICKS         20U
#define SUPVC_US016_ADC_MAX             4095.0f
#define SUPVC_US016_ADC_FAULT_MIN       0U
#define SUPVC_US016_INVALID_HOLD_TICKS  10U
#define SUPVC_HCSR04_ALPHA              0.30f
#define SUPVC_KS103_ALPHA               0.22f
#define SUPVC_HCSR04_JUMP_LIMIT_CM      45.0f
#define SUPVC_KS103_JUMP_LIMIT_CM       45.0f
#define SUPVC_KS103_MIN_CM              2.0f
#define SUPVC_KS103_MAX_CM              500.0f
#define SUPVC_KS103_MAX_RAW_MM          5000U
#define SUPVC_KS103_MM_TO_CM            0.1f
#define SUPVC_KS103_TIMEOUT_MAX_TICKS   30U
#define SUPVC_KS103_CONVERSION_TICKS    7U
#define SUPVC_KS103_RECOVER_DIRECT_THR  3U
#define SUPVC_KS103_PROFILE_SWITCH_FAILS 10U
#define SUPVC_KS103_ADDR_CANDIDATE_COUNT 6U
#define SUPVC_KS103_ADDR_8BIT           ((uint16_t)(SUPVC_KS103_I2C_ADDRESS_7BIT << 1U))
#define SUPVC_KS103_TRIGGER_REG         0x00U
#define SUPVC_KS103_TRIGGER_CMD_CM      0x51U
#define SUPVC_KS103_ALT_TRIGGER_REG     0x02U
#define SUPVC_KS103_ALT_TRIGGER_CMD_CM  0xB4U
#define SUPVC_KS103_DISTANCE_REG        0x02U

/* 5点中值滤波：对脉冲干扰有很好的抑制作用 */
static float SUPVC_Median5(float buf[SUPVC_MEDIAN_WINDOW_SIZE])
{
  float sorted[SUPVC_MEDIAN_WINDOW_SIZE];
  uint8_t i, j;
  float temp;

  /* 复制到临时数组 */
  for (i = 0U; i < SUPVC_MEDIAN_WINDOW_SIZE; i++) {
    sorted[i] = buf[i];
  }

  /* 冒泡排序 */
  for (i = 0U; i < SUPVC_MEDIAN_WINDOW_SIZE - 1U; i++) {
    for (j = 0U; j < SUPVC_MEDIAN_WINDOW_SIZE - 1U - i; j++) {
      if (sorted[j] > sorted[j + 1U]) {
        temp = sorted[j];
        sorted[j] = sorted[j + 1U];
        sorted[j + 1U] = temp;
      }
    }
  }

  return sorted[2U];  /* 返回中值 */
}

static const uint8_t supvc_ks103_addr7_candidates[SUPVC_KS103_ADDR_CANDIDATE_COUNT] = {
  SUPVC_KS103_I2C_ADDRESS_7BIT,
  (uint8_t)(SUPVC_KS103_I2C_ADDRESS_7BIT >> 1U),
  0x70U,
  0x71U,
  0x72U,
  0x73U
};

static float SUPVC_ApplySmoothFilter(uint8_t ch, float measurement, float alpha, float jump_limit_cm)
{
  float median_value;
  float current;
  float limited;
  uint8_t i;
  float sorted[SUPVC_MEDIAN_WINDOW_SIZE];
  float temp;

  /* 中值滤波步骤：将测量值加入缓冲区 */
  supvc_median_buffer[ch][supvc_median_index[ch]] = measurement;
  supvc_median_index[ch] = (uint8_t)((supvc_median_index[ch] + 1U) % SUPVC_MEDIAN_WINDOW_SIZE);
  if (supvc_median_count[ch] < SUPVC_MEDIAN_WINDOW_SIZE) {
    supvc_median_count[ch]++;
  }

  /* 如果缓冲区未满，直接返回测量值 */
  if (supvc_median_count[ch] < SUPVC_MEDIAN_WINDOW_SIZE) {
    supvc_filter_initialized[ch] = 1U;
    return measurement;
  }

  /* 5点中值滤波：复制并排序 */
  for (i = 0U; i < SUPVC_MEDIAN_WINDOW_SIZE; i++) {
    sorted[i] = supvc_median_buffer[ch][i];
  }
  /* 冒泡排序找中值 */
  for (i = 0U; i < SUPVC_MEDIAN_WINDOW_SIZE - 1U; i++) {
    uint8_t j;
    for (j = 0U; j < SUPVC_MEDIAN_WINDOW_SIZE - 1U - i; j++) {
      if (sorted[j] > sorted[j + 1U]) {
        temp = sorted[j];
        sorted[j] = sorted[j + 1U];
        sorted[j + 1U] = temp;
      }
    }
  }
  median_value = sorted[2U];  /* 取中值 */

  /* IIR 滤波平滑 */
  if (!supvc_filter_initialized[ch] || !supvc_distance_valid[ch] || (supvc_distance_cm[ch] < 0.0f)) {
    supvc_filter_initialized[ch] = 1U;
    return median_value;
  }

  current = supvc_distance_cm[ch];
  limited = median_value;

  /* 跳变限制（防止异常值） */
  if (fabsf(median_value - current) > jump_limit_cm) {
    if (median_value > current) {
      limited = current + jump_limit_cm;
    } else {
      limited = current - jump_limit_cm;
    }
  }

  return current + alpha * (limited - current);
}

static void SUPVC_ResetChannelState(uint8_t ch)
{
  supvc_echo_width_us[ch] = 0U;
  supvc_distance_cm[ch] = -1.0f;
  supvc_distance_valid[ch] = 0U;
  supvc_filter_initialized[ch] = 0U;
  supvc_median_count[ch] = 0U;
  supvc_median_index[ch] = 0U;
}

static uint8_t SUPVC_InISRContext(void)
{
  return (__get_IPSR() != 0U) ? 1U : 0U;
}

static uint16_t SUPVC_KS103_CurrentAddr8(void)
{
  uint8_t addr7;

  if (supvc_ks103_addr7_manual) {
    addr7 = supvc_ks103_addr7_value;
  } else {
    addr7 = supvc_ks103_addr7_candidates[supvc_ks103_addr_index];
  }

  return (uint16_t)((uint16_t)addr7 << 1U);
}

static uint8_t SUPVC_KS103_CurrentTriggerReg(void)
{
  return supvc_ks103_alt_trigger_mode ? SUPVC_KS103_ALT_TRIGGER_REG : SUPVC_KS103_TRIGGER_REG;
}

static uint8_t SUPVC_KS103_CurrentTriggerCmd(void)
{
  return supvc_ks103_alt_trigger_mode ? SUPVC_KS103_ALT_TRIGGER_CMD_CM : SUPVC_KS103_TRIGGER_CMD_CM;
}

static void SUPVC_KS103_RegisterSuccess(void)
{
  supvc_ks103_timeout_recover = 0U;
  supvc_ks103_comm_fail_count = 0U;
}

static void SUPVC_KS103_RegisterFailure(void)
{
  if (supvc_ks103_addr7_manual) {
    if (supvc_ks103_timeout_recover < 0xFFU) {
      supvc_ks103_timeout_recover++;
    }
    if (supvc_ks103_timeout_recover >= SUPVC_KS103_RECOVER_DIRECT_THR) {
      supvc_ks103_direct_read_mode = 1U;
    }
    return;
  }

  if (supvc_ks103_timeout_recover < 0xFFU) {
    supvc_ks103_timeout_recover++;
  }
  if (supvc_ks103_timeout_recover >= SUPVC_KS103_RECOVER_DIRECT_THR) {
    supvc_ks103_direct_read_mode = 1U;
  }

  if (supvc_ks103_comm_fail_count < 0xFFU) {
    supvc_ks103_comm_fail_count++;
  }

  if (supvc_ks103_comm_fail_count >= SUPVC_KS103_PROFILE_SWITCH_FAILS) {
    supvc_ks103_comm_fail_count = 0U;
    supvc_ks103_timeout_recover = 0U;
    supvc_ks103_direct_read_mode = 0U;

    if (supvc_ks103_alt_trigger_mode == 0U) {
      supvc_ks103_alt_trigger_mode = 1U;
    } else {
      supvc_ks103_alt_trigger_mode = 0U;
      supvc_ks103_addr_index = (uint8_t)((supvc_ks103_addr_index + 1U) % SUPVC_KS103_ADDR_CANDIDATE_COUNT);
    }
  }
}

static void SUPVC_SetCaptureEdge(uint8_t ch, uint8_t rising)
{
  __HAL_TIM_SET_CAPTUREPOLARITY(&htim2,
                                echo_channel[ch],
                                rising ? TIM_INPUTCHANNELPOLARITY_RISING : TIM_INPUTCHANNELPOLARITY_FALLING);
  supvc_echo_capture_rising[ch] = rising;
}

static void SUPVC_DelayUs(uint32_t us)
{
  uint32_t start_ticks;
  uint32_t wait_ticks;

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  start_ticks = DWT->CYCCNT;
  wait_ticks = us * (SystemCoreClock / 1000000U);
  while ((DWT->CYCCNT - start_ticks) < wait_ticks) {
  }
}

static void SUPVC_ADC_Init(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

  GPIOA->MODER |= (3UL << (4U * 2U)) | (3UL << (6U * 2U));
  GPIOA->PUPDR &= ~((3UL << (4U * 2U)) | (3UL << (6U * 2U)));
  GPIOC->MODER |= (3UL << (0U * 2U));
  GPIOC->PUPDR &= ~(3UL << (0U * 2U));

  ADC->CCR &= ~ADC_CCR_ADCPRE;
  ADC->CCR |= ADC_CCR_ADCPRE_0;

  ADC1->CR1 = 0U;
  ADC1->CR2 = ADC_CR2_ADON;

  ADC1->SMPR2 &= ~((7UL << (4U * 3U)) | (7UL << (6U * 3U)));
  ADC1->SMPR2 |= (7UL << (4U * 3U)) | (7UL << (6U * 3U));
  ADC1->SMPR1 &= ~(7UL << ((10U - 10U) * 3U));
  ADC1->SMPR1 |= (7UL << ((10U - 10U) * 3U));

  ADC1->SQR1 = 0U;
  ADC1->SQR2 = 0U;
  ADC1->SQR3 = SUPVC_US016_CH3_ADC_CHANNEL;
}

static uint16_t SUPVC_ADC_ReadUS016Raw(uint8_t adc_channel)
{
  ADC1->SQR3 = (uint32_t)adc_channel;
  ADC1->CR2 &= ~ADC_CR2_CONT;
  ADC1->CR2 |= ADC_CR2_SWSTART;
  while ((ADC1->SR & ADC_SR_EOC) == 0U) {
  }
  return (uint16_t)(ADC1->DR & 0xFFFFU);
}

static uint16_t SUPVC_ADC_ReadUS016Average(uint8_t adc_channel, uint8_t sample_count)
{
  uint32_t sum = 0U;
  uint8_t i;

  if (sample_count == 0U) {
    sample_count = 1U;
  }

  for (i = 0U; i < sample_count; i++) {
    sum += SUPVC_ADC_ReadUS016Raw(adc_channel);
  }

  return (uint16_t)(sum / sample_count);
}

static void SUPVC_UpdateUS016Channel(uint8_t us016_slot, uint8_t channel_index, uint16_t adc_raw)
{
  float analog_distance_cm;
  float range_cm = us016_range_cm_cfg[us016_slot];
  float min_cm = us016_min_cm_cfg[us016_slot];

  supvc_echo_width_us[channel_index] = adc_raw;
  analog_distance_cm = ((float)adc_raw / SUPVC_US016_ADC_MAX) * range_cm;

  if (adc_raw > SUPVC_US016_ADC_FAULT_MIN) {
    if (analog_distance_cm < min_cm) {
      analog_distance_cm = min_cm;
    } else if (analog_distance_cm > range_cm) {
      analog_distance_cm = range_cm;
    }

    supvc_us016_invalid_ticks[us016_slot] = 0U;
    supvc_distance_cm[channel_index] = SUPVC_ApplySmoothFilter(channel_index,
                                                               analog_distance_cm,
                                                               us016_alpha_cfg[us016_slot],
                                                               us016_jump_limit_cfg[us016_slot]);
    supvc_distance_valid[channel_index] = 1U;
  } else {
    if ((us016_slot == 1U) || (us016_slot == 2U)) {
      /* D4/D5 在贴近墙面时可能瞬时掉到 0，近距模式下将其视为最小可测距离而不是直接无效。 */
      supvc_us016_invalid_ticks[us016_slot] = 0U;
      supvc_distance_cm[channel_index] = SUPVC_ApplySmoothFilter(channel_index,
                                                                 min_cm,
                                                                 us016_alpha_cfg[us016_slot],
                                                                 us016_jump_limit_cfg[us016_slot]);
      supvc_distance_valid[channel_index] = 1U;
      return;
    }

    if (supvc_us016_invalid_ticks[us016_slot] < 0xFFU) {
      supvc_us016_invalid_ticks[us016_slot]++;
    }
    if (supvc_us016_invalid_ticks[us016_slot] > SUPVC_US016_INVALID_HOLD_TICKS) {
      SUPVC_ResetChannelState(channel_index);
    }
  }
}

static void SUPVC_KS103_StartTriggerIT(void)
{
  if (supvc_ks103_busy) {
    return;
  }

  if (SUPVC_InISRContext()) {
    return;
  }

  if (hi2c1.Instance != I2C1) {
    return;
  }

  if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
    return;
  }

  supvc_ks103_tx_cmd_cm = SUPVC_KS103_CurrentTriggerCmd();
  if (HAL_I2C_Mem_Write_IT(&hi2c1,
                           SUPVC_KS103_CurrentAddr8(),
                           SUPVC_KS103_CurrentTriggerReg(),
                           I2C_MEMADD_SIZE_8BIT,
                           &supvc_ks103_tx_cmd_cm,
                           1U) == HAL_OK) {
    supvc_ks103_busy = 1U;
    supvc_ks103_phase = SUPVC_KS103_PHASE_WAIT_TX;
  } else {
    SUPVC_KS103_RegisterFailure();
  }
}

static void SUPVC_KS103_StartReadIT(void)
{
  if (supvc_ks103_busy) {
    return;
  }

  if (SUPVC_InISRContext()) {
    return;
  }

  if (hi2c1.Instance != I2C1) {
    return;
  }

  if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
    return;
  }

  if (HAL_I2C_Mem_Read_IT(&hi2c1,
                          SUPVC_KS103_CurrentAddr8(),
                          SUPVC_KS103_DISTANCE_REG,
                          I2C_MEMADD_SIZE_8BIT,
                          supvc_ks103_rx_buf,
                          2U) == HAL_OK) {
    supvc_ks103_busy = 1U;
    supvc_ks103_phase = SUPVC_KS103_PHASE_WAIT_RX;
  } else {
    SUPVC_KS103_RegisterFailure();
  }
}

static void SUPVC_ProcessKS103Channel(void)
{
  float distance_cm;
  uint16_t raw;

  if (supvc_ks103_timeout_ticks < 0xFFU) {
    supvc_ks103_timeout_ticks++;
  }

  if (supvc_ks103_data_ready) {
    supvc_ks103_data_ready = 0U;
    supvc_ks103_timeout_ticks = 0U;

    raw = supvc_ks103_raw_cm;
    supvc_echo_width_us[SUPVC_KS103_CHANNEL_INDEX] = (uint32_t)raw;

    /* 兼容 KS103 不同固件单位：若回传值明显大于 500，按 mm 转 cm。 */
    if ((raw > (uint16_t)SUPVC_KS103_MAX_CM) && (raw <= SUPVC_KS103_MAX_RAW_MM)) {
      distance_cm = (float)raw * SUPVC_KS103_MM_TO_CM;
    } else {
      distance_cm = (float)raw;
    }

    if ((distance_cm >= SUPVC_KS103_MIN_CM) && (distance_cm <= SUPVC_KS103_MAX_CM)) {
      supvc_distance_cm[SUPVC_KS103_CHANNEL_INDEX] = SUPVC_ApplySmoothFilter(SUPVC_KS103_CHANNEL_INDEX,
                                                                              distance_cm,
                                                                              SUPVC_KS103_ALPHA,
                                                                              SUPVC_KS103_JUMP_LIMIT_CM);
      supvc_distance_valid[SUPVC_KS103_CHANNEL_INDEX] = 1U;
      SUPVC_KS103_RegisterSuccess();
    } else {
      SUPVC_ResetChannelState(SUPVC_KS103_CHANNEL_INDEX);
    }
    supvc_ks103_phase = SUPVC_KS103_PHASE_IDLE;
  }

  if (supvc_ks103_timeout_ticks > SUPVC_KS103_TIMEOUT_MAX_TICKS) {
    SUPVC_ResetChannelState(SUPVC_KS103_CHANNEL_INDEX);
    supvc_ks103_busy = 0U;
    supvc_ks103_phase = SUPVC_KS103_PHASE_IDLE;
    supvc_ks103_conversion_ticks = 0U;
    SUPVC_KS103_RegisterFailure();
  }

  switch (supvc_ks103_phase) {
    case SUPVC_KS103_PHASE_IDLE:
      if (supvc_ks103_direct_read_mode) {
        SUPVC_KS103_StartReadIT();
      } else {
        SUPVC_KS103_StartTriggerIT();
      }
      break;

    case SUPVC_KS103_PHASE_WAIT_CONVERSION:
      if (supvc_ks103_conversion_ticks < 0xFFU) {
        supvc_ks103_conversion_ticks++;
      }
      if (supvc_ks103_conversion_ticks >= SUPVC_KS103_CONVERSION_TICKS) {
        SUPVC_KS103_StartReadIT();
      }
      break;

    case SUPVC_KS103_PHASE_WAIT_TX:
    case SUPVC_KS103_PHASE_WAIT_RX:
    default:
      break;
  }
}

void MX_TIM2_Init(void)
{
  TIM_IC_InitTypeDef sConfigIC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 8;

  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
}

void SUPVC_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  uint8_t i;

  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  for (i = 0U; i < SUPVC_CHANNEL_COUNT; i++) {
    supvc_echo_start[i] = 0U;
    supvc_echo_width_us[i] = 0U;
    supvc_echo_capture_rising[i] = (i < SUPVC_HCSR04_CHANNEL_COUNT) ? 1U : 0U;
    supvc_distance_cm[i] = -1.0f;
    supvc_distance_valid[i] = 0U;
    supvc_timeout_ticks[i] = 0U;
    supvc_filter_initialized[i] = 0U;
  }
  supvc_ks103_busy = 0U;
  supvc_ks103_data_ready = 0U;
  supvc_ks103_raw_cm = 0U;
  supvc_ks103_timeout_ticks = 0U;
  supvc_ks103_conversion_ticks = 0U;
  supvc_ks103_direct_read_mode = 0U;
  supvc_ks103_timeout_recover = 0U;
  supvc_ks103_addr_index = 0U;
  supvc_ks103_addr7_manual = 0U;
  supvc_ks103_addr7_value = SUPVC_KS103_I2C_ADDRESS_7BIT;
  supvc_ks103_alt_trigger_mode = 0U;
  supvc_ks103_comm_fail_count = 0U;
  supvc_ks103_phase = SUPVC_KS103_PHASE_IDLE;
  for (i = 0U; i < SUPVC_US016_CHANNEL_COUNT; i++) {
    supvc_us016_invalid_ticks[i] = 0U;
  }

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

  for (i = 0U; i < SUPVC_HCSR04_CHANNEL_COUNT; i++) {
    GPIO_InitStruct.Pin = trig_pin[i];
    HAL_GPIO_Init(trig_port[i], &GPIO_InitStruct);
    HAL_GPIO_WritePin(trig_port[i], trig_pin[i], GPIO_PIN_RESET);
  }

  for (i = 0U; i < SUPVC_US016_CHANNEL_COUNT; i++) {
    GPIO_InitStruct.Pin = us016_range_pin[i];
    HAL_GPIO_Init(us016_range_port[i], &GPIO_InitStruct);
    HAL_GPIO_WritePin(us016_range_port[i], us016_range_pin[i], us016_range_level[i]);
  }

  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(TIM2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);

  SUPVC_ADC_Init();
  MX_TIM2_Init();
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);

  SUPVC_SetCaptureEdge(0U, 1U);
  SUPVC_SetCaptureEdge(1U, 1U);
}

void SUPVC_Trigger(uint8_t ch)
{
  if ((ch < 1U) || (ch > SUPVC_HCSR04_CHANNEL_COUNT)) return;
  ch -= 1U;

  HAL_GPIO_WritePin(trig_port[ch], trig_pin[ch], GPIO_PIN_RESET);
  SUPVC_DelayUs(2U);
  HAL_GPIO_WritePin(trig_port[ch], trig_pin[ch], GPIO_PIN_SET);
  SUPVC_DelayUs(SUPVC_TRIGGER_US);
  HAL_GPIO_WritePin(trig_port[ch], trig_pin[ch], GPIO_PIN_RESET);
}

void SUPVC_Service_10ms(void)
{
  static uint8_t next_channel = 1U;
  static uint8_t startup_delay = SUPVC_STARTUP_DELAY_TICKS;
  static uint8_t cooldown_ticks = 0U;
  uint8_t i;
  uint16_t adc_raw;

  for (i = 0U; i < SUPVC_HCSR04_CHANNEL_COUNT; i++) {
    if (supvc_timeout_ticks[i] < 0xFFU) {
      supvc_timeout_ticks[i]++;
    }
    if (supvc_timeout_ticks[i] > SUPVC_TIMEOUT_MAX_TICKS) {
      SUPVC_ResetChannelState(i);
      SUPVC_SetCaptureEdge(i, 1U);
    }
  }

  if (startup_delay > 0U) {
    startup_delay--;
    return;
  }

  for (i = 0U; i < SUPVC_US016_CHANNEL_COUNT; i++) {
    adc_raw = SUPVC_ADC_ReadUS016Average(us016_adc_channel[i], us016_adc_samples[i]);
    SUPVC_UpdateUS016Channel(i, us016_channel_index[i], adc_raw);
  }

  if (cooldown_ticks > 0U) {
    cooldown_ticks--;
    return;
  }

  SUPVC_Trigger(next_channel);
  next_channel++;
  if (next_channel > SUPVC_HCSR04_CHANNEL_COUNT) {
    next_channel = 1U;
  }

  cooldown_ticks = SUPVC_HCSR04_GAP_TICKS;
}

void SUPVC_Service_MainLoop(void)
{
  static uint32_t ks103_last_ms = 0U;
  uint32_t now_ms = HAL_GetTick();

  if ((now_ms - ks103_last_ms) < 10U) {
    return;
  }

  ks103_last_ms = now_ms;
  SUPVC_ProcessKS103Channel();
}

void SUPVC_SetKS103Address7bit(uint8_t addr7)
{
  if ((addr7 < 0x08U) || (addr7 > 0x77U)) {
    return;
  }

  __disable_irq();
  supvc_ks103_addr7_manual = 1U;
  supvc_ks103_addr7_value = addr7;
  supvc_ks103_busy = 0U;
  supvc_ks103_data_ready = 0U;
  supvc_ks103_phase = SUPVC_KS103_PHASE_IDLE;
  supvc_ks103_timeout_ticks = 0U;
  supvc_ks103_conversion_ticks = 0U;
  supvc_ks103_timeout_recover = 0U;
  supvc_ks103_comm_fail_count = 0U;
  supvc_ks103_direct_read_mode = 0U;
  __enable_irq();
}

void SUPVC_SetKS103Mode(uint8_t direct_read, uint8_t alt_trigger)
{
  __disable_irq();
  supvc_ks103_direct_read_mode = direct_read ? 1U : 0U;
  supvc_ks103_alt_trigger_mode = alt_trigger ? 1U : 0U;
  supvc_ks103_busy = 0U;
  supvc_ks103_phase = SUPVC_KS103_PHASE_IDLE;
  supvc_ks103_conversion_ticks = 0U;
  supvc_ks103_timeout_ticks = 0U;
  __enable_irq();
}

void SUPVC_SetKS103AutoDetect(void)
{
  __disable_irq();
  supvc_ks103_addr7_manual = 0U;
  supvc_ks103_addr7_value = SUPVC_KS103_I2C_ADDRESS_7BIT;
  supvc_ks103_addr_index = 0U;
  supvc_ks103_alt_trigger_mode = 0U;
  supvc_ks103_direct_read_mode = 0U;
  supvc_ks103_timeout_recover = 0U;
  supvc_ks103_comm_fail_count = 0U;
  supvc_ks103_busy = 0U;
  supvc_ks103_phase = SUPVC_KS103_PHASE_IDLE;
  supvc_ks103_timeout_ticks = 0U;
  supvc_ks103_conversion_ticks = 0U;
  __enable_irq();
}

uint8_t SUPVC_GetKS103Address7bit(void)
{
  if (supvc_ks103_addr7_manual) {
    return supvc_ks103_addr7_value;
  }
  return supvc_ks103_addr7_candidates[supvc_ks103_addr_index];
}

void SUPVC_GetKS103Flags(uint8_t *manual_addr, uint8_t *direct_read, uint8_t *alt_trigger)
{
  if (manual_addr != NULL) {
    *manual_addr = supvc_ks103_addr7_manual;
  }
  if (direct_read != NULL) {
    *direct_read = supvc_ks103_direct_read_mode;
  }
  if (alt_trigger != NULL) {
    *alt_trigger = supvc_ks103_alt_trigger_mode;
  }
}

float SUPVC_GetDistanceCm(uint8_t ch)
{
  if ((ch < 1U) || (ch > SUPVC_CHANNEL_COUNT)) return -1.0f;
  ch -= 1U;
  if (!supvc_distance_valid[ch]) return -1.0f;
  return supvc_distance_cm[ch];
}

uint32_t SUPVC_GetEchoWidthUs(uint8_t ch)
{
  if ((ch < 1U) || (ch > SUPVC_CHANNEL_COUNT)) return 0U;
  return supvc_echo_width_us[ch - 1U];
}

uint8_t SUPVC_IsValid(uint8_t ch)
{
  if ((ch < 1U) || (ch > SUPVC_CHANNEL_COUNT)) return 0U;
  return supvc_distance_valid[ch - 1U];
}

void HAL_TIM_IC_MspInit(TIM_HandleTypeDef* tim_icHandle)
{
  (void)tim_icHandle;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  uint8_t ch = 0xFFU;
  uint32_t capture;
  uint32_t start;
  uint32_t width;
  float distance_cm;

  if (htim->Instance != TIM2) return;

  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) ch = 0U;
  else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) ch = 1U;
  else return;

  capture = HAL_TIM_ReadCapturedValue(htim, echo_channel[ch]);

  if (supvc_echo_capture_rising[ch]) {
    supvc_echo_start[ch] = capture;
    SUPVC_SetCaptureEdge(ch, 0U);
    return;
  }

  start = supvc_echo_start[ch];
  if (capture >= start) {
    width = capture - start;
  } else {
    width = (0xFFFFFFFFUL - start) + capture + 1UL;
  }

  SUPVC_SetCaptureEdge(ch, 1U);
  supvc_timeout_ticks[ch] = 0U;

  if ((width < 100U) || (width > 25000U)) {
    return;
  }

  supvc_echo_width_us[ch] = width;
  distance_cm = (width * 0.0343f) / 2.0f;

  supvc_distance_cm[ch] = SUPVC_ApplySmoothFilter(ch,
                                                  distance_cm,
                                                  SUPVC_HCSR04_ALPHA,
                                                  SUPVC_HCSR04_JUMP_LIMIT_CM);
  supvc_distance_valid[ch] = 1U;
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance != I2C1) {
    return;
  }

  supvc_ks103_busy = 0U;
  if (supvc_ks103_phase == SUPVC_KS103_PHASE_WAIT_TX) {
    supvc_ks103_phase = SUPVC_KS103_PHASE_WAIT_CONVERSION;
    supvc_ks103_conversion_ticks = 0U;
  }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance != I2C1) {
    return;
  }

  supvc_ks103_raw_cm = (uint16_t)((((uint16_t)supvc_ks103_rx_buf[0]) << 8U) | supvc_ks103_rx_buf[1]);
  supvc_ks103_data_ready = 1U;
  supvc_ks103_busy = 0U;
  supvc_ks103_phase = SUPVC_KS103_PHASE_IDLE;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance != I2C1) {
    return;
  }

  supvc_ks103_busy = 0U;
  supvc_ks103_phase = SUPVC_KS103_PHASE_IDLE;
  supvc_ks103_conversion_ticks = 0U;
  SUPVC_KS103_RegisterFailure();
}
