#include "supvc.h"
#include <math.h>

TIM_HandleTypeDef htim2;

static volatile uint32_t supvc_echo_start[SUPVC_CHANNEL_COUNT] = {0};
static volatile uint32_t supvc_echo_width_us[SUPVC_CHANNEL_COUNT] = {0};
static volatile uint8_t supvc_echo_capture_rising[SUPVC_CHANNEL_COUNT] = {1, 1, 1};
static volatile float supvc_distance_cm[SUPVC_CHANNEL_COUNT] = {-1.0f, -1.0f, -1.0f};
static volatile uint8_t supvc_distance_valid[SUPVC_CHANNEL_COUNT] = {0, 0, 0};
static volatile uint8_t supvc_timeout_ticks[SUPVC_CHANNEL_COUNT] = {0, 0, 0};
static volatile uint8_t supvc_filter_initialized[SUPVC_CHANNEL_COUNT] = {0, 0, 0};

static const uint16_t trig_pin[2] = {SUPVC_TRIG1_PIN, SUPVC_TRIG2_PIN};
static GPIO_TypeDef* const trig_port[2] = {SUPVC_TRIG1_GPIO_PORT, SUPVC_TRIG2_GPIO_PORT};
static const uint32_t echo_channel[2] = {TIM_CHANNEL_1, TIM_CHANNEL_4};

#define SUPVC_TRIGGER_US            15U
#define SUPVC_STARTUP_DELAY_TICKS   30U
#define SUPVC_HCSR04_GAP_TICKS      6U
#define SUPVC_TIMEOUT_MAX_TICKS     20U
#define SUPVC_US016_RANGE_3M        1U
#define SUPVC_US016_ADC_MAX         4095.0f
#define SUPVC_US016_RANGE_CM        300.0f
#define SUPVC_US016_ADC_SAMPLES     8U
#define SUPVC_HCSR04_ALPHA          0.30f
#define SUPVC_US016_ALPHA           0.18f
#define SUPVC_HCSR04_JUMP_LIMIT_CM  45.0f
#define SUPVC_US016_JUMP_LIMIT_CM   35.0f

static float SUPVC_ApplySmoothFilter(uint8_t ch, float measurement, float alpha, float jump_limit_cm)
{
  float current;
  float limited = measurement;

  if (!supvc_filter_initialized[ch] || !supvc_distance_valid[ch] || (supvc_distance_cm[ch] < 0.0f)) {
    supvc_filter_initialized[ch] = 1U;
    return measurement;
  }

  current = supvc_distance_cm[ch];
  if (fabsf(measurement - current) > jump_limit_cm) {
    if (measurement > current) {
      limited = current + jump_limit_cm;
    } else {
      limited = current - jump_limit_cm;
    }
  }

  return current + alpha * (limited - current);
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
  GPIOA->MODER |= (3UL << (6U * 2U));
  GPIOA->PUPDR &= ~(3UL << (6U * 2U));

  ADC->CCR &= ~ADC_CCR_ADCPRE;
  ADC->CCR |= ADC_CCR_ADCPRE_0;

  ADC1->CR1 = 0U;
  ADC1->CR2 = ADC_CR2_ADON;
  ADC1->SMPR2 &= ~(7UL << 18U);
  ADC1->SMPR2 |= (7UL << 18U);
  ADC1->SQR1 = 0U;
  ADC1->SQR2 = 0U;
  ADC1->SQR3 = 6U;
}

static uint16_t SUPVC_ADC_ReadUS016Raw(void)
{
  ADC1->CR2 &= ~ADC_CR2_CONT;
  ADC1->CR2 |= ADC_CR2_SWSTART;
  while ((ADC1->SR & ADC_SR_EOC) == 0U) {
  }
  return (uint16_t)(ADC1->DR & 0xFFFFU);
}

static uint16_t SUPVC_ADC_ReadUS016Average(void)
{
  uint32_t sum = 0U;
  uint8_t i;

  for (i = 0U; i < SUPVC_US016_ADC_SAMPLES; i++) {
    sum += SUPVC_ADC_ReadUS016Raw();
  }

  return (uint16_t)(sum / SUPVC_US016_ADC_SAMPLES);
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

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

  for (i = 0; i < 2U; i++) {
    GPIO_InitStruct.Pin = trig_pin[i];
    HAL_GPIO_Init(trig_port[i], &GPIO_InitStruct);
    HAL_GPIO_WritePin(trig_port[i], trig_pin[i], GPIO_PIN_RESET);
  }

  GPIO_InitStruct.Pin = SUPVC_TRIG3_PIN;
  HAL_GPIO_Init(SUPVC_TRIG3_GPIO_PORT, &GPIO_InitStruct);
#if SUPVC_US016_RANGE_3M
  HAL_GPIO_WritePin(SUPVC_TRIG3_GPIO_PORT, SUPVC_TRIG3_PIN, GPIO_PIN_SET);
#else
  HAL_GPIO_WritePin(SUPVC_TRIG3_GPIO_PORT, SUPVC_TRIG3_PIN, GPIO_PIN_RESET);
#endif

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
  if ((ch < 1U) || (ch > 2U)) return;
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
  float analog_distance_cm;

  for (i = 0; i < 2U; i++) {
    if (supvc_timeout_ticks[i] < 0xFFU) {
      supvc_timeout_ticks[i]++;
    }
    if (supvc_timeout_ticks[i] > SUPVC_TIMEOUT_MAX_TICKS) {
      supvc_distance_valid[i] = 0U;
      supvc_distance_cm[i] = -1.0f;
      supvc_echo_width_us[i] = 0U;
      supvc_filter_initialized[i] = 0U;
      SUPVC_SetCaptureEdge(i, 1U);
    }
  }

  if (startup_delay > 0U) {
    startup_delay--;
    return;
  }

  adc_raw = SUPVC_ADC_ReadUS016Average();
  supvc_echo_width_us[2] = adc_raw;
  analog_distance_cm = ((float)adc_raw / SUPVC_US016_ADC_MAX) * SUPVC_US016_RANGE_CM;
  if ((analog_distance_cm >= 2.0f) && (analog_distance_cm <= SUPVC_US016_RANGE_CM)) {
    supvc_distance_cm[2] = SUPVC_ApplySmoothFilter(2U,
                                                   analog_distance_cm,
                                                   SUPVC_US016_ALPHA,
                                                   SUPVC_US016_JUMP_LIMIT_CM);
    supvc_distance_valid[2] = 1U;
  } else {
    supvc_distance_valid[2] = 0U;
    supvc_distance_cm[2] = -1.0f;
    supvc_filter_initialized[2] = 0U;
  }

  if (cooldown_ticks > 0U) {
    cooldown_ticks--;
    return;
  }

  SUPVC_Trigger(next_channel);
  next_channel++;
  if (next_channel > 2U) {
    next_channel = 1U;
  }

  cooldown_ticks = SUPVC_HCSR04_GAP_TICKS;
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
