#include "stm32f4xx_hal.h"
#include <stdio.h>

#define POT_PORT GPIOA
#define POT_PIN GPIO_PIN_0 /* ADC1_0 */

#define LED_PORT GPIOA
#define LED_PIN GPIO_PIN_1 /* TIM2_CH2, AF01 */

#define UART_TX_PORT GPIOA
#define UART_TX_PIN GPIO_PIN_2 /* USART2_TX, AF07 */

#define MOTOR_PORT GPIOA
#define MOTOR_PIN GPIO_PIN_3 /* TIM5_CH4, AF02 */

#define TIM_TICK_HZ 1000000U /* 16 MHz / PSC 16 */

/* TIM2: LED PWM */
#define LED_PWM_FREQ_HZ 5000U
#define LED_PWM_PSC 16U
#define LED_PWM_ARR (TIM_TICK_HZ / LED_PWM_FREQ_HZ - 1U)
#define LED_PWM_STEPS (LED_PWM_ARR + 1U)

/* TIM5: MOTOR PWM */
#define MOTOR_PWM_FREQ_HZ 500U
#define MOTOR_PWM_PSC 16U
#define MOTOR_PWM_ARR (TIM_TICK_HZ / MOTOR_PWM_FREQ_HZ - 1U)
#define MOTOR_PWM_STEPS (MOTOR_PWM_ARR + 1U)

#define PRINT_EVERY 500U /* ADC 5 kHz -> UART 10 Hz */

/* ---------- TIM4: trigger for the injected group (VREFINT) ---------- */
#define VREF_PERIOD_MS 100U /* 10 Hz */

#define VREFINT_CAL_ADDR ((const uint16_t *)0x1FFF7A2AU)
#define VDDA_NOMINAL_MV 3300U
#define VREFINT_AVG_N 4U /* average of the last N VREFINT samples, 1 = no averaging */

#define ADC_MAX 4095U

/* ---------- HAL handles ---------- */
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim2; // LED PWM, 5 kHz
TIM_HandleTypeDef htim4; // VREFINT, 10 Hz
TIM_HandleTypeDef htim5; // Motor PWM, 500 Hz
UART_HandleTypeDef huart2;

/* ---------- Shared with interrupts ---------- */
volatile uint16_t g_raw = 0;
volatile uint32_t g_led_duty = 0;
volatile uint32_t g_motor_duty = 0;
volatile bool g_new = false;
volatile uint32_t g_vdda_mv = VDDA_NOMINAL_MV;

static uint32_t duty_from_raw(uint16_t raw, uint32_t steps)
{
    return (uint32_t)raw * steps / ADC_MAX;
}

static void GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};

    gpio.Pin = POT_PIN;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(POT_PORT, &gpio);

    gpio.Pin = LED_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(LED_PORT, &gpio);

    gpio.Pin = MOTOR_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(MOTOR_PORT, &gpio);

    gpio.Pin = UART_TX_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(UART_TX_PORT, &gpio);
}

void Error_Handler(void);

static void UART2_Init(void)
{
    __HAL_RCC_USART2_CLK_ENABLE();

    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
}

static void ADC_Init(void)
{
    __HAL_RCC_ADC1_CLK_ENABLE();

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    ADC_ChannelConfTypeDef adcChannel = {0};

    adcChannel.Channel = ADC_CHANNEL_0;
    adcChannel.Rank = 1;
    adcChannel.SamplingTime = ADC_SAMPLETIME_84CYCLES;

    if (HAL_ADC_ConfigChannel(&hadc1, &adcChannel) != HAL_OK)
    {
        Error_Handler();
    }

    ADC_InjectionConfTypeDef adcInjected = {0};

    adcInjected.InjectedChannel = ADC_CHANNEL_VREFINT;
    adcInjected.InjectedRank = ADC_INJECTED_RANK_1;
    adcInjected.InjectedNbrOfConversion = 1;
    adcInjected.InjectedSamplingTime = ADC_SAMPLETIME_480CYCLES;
    adcInjected.InjectedOffset = 0;
    adcInjected.InjectedDiscontinuousConvMode = DISABLE;
    adcInjected.AutoInjectedConv = DISABLE;
    adcInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
    adcInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T4_TRGO;

    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &adcInjected) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_NVIC_SetPriority(ADC_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);
}

static void TIM2_Init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = LED_PWM_PSC - 1U;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = LED_PWM_ARR;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }

    TIM_OC_InitTypeDef pwmChannel = {0};

    pwmChannel.OCMode = TIM_OCMODE_PWM1;
    pwmChannel.Pulse = 0;
    pwmChannel.OCPolarity = TIM_OCPOLARITY_HIGH;
    pwmChannel.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&htim2, &pwmChannel, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }

    TIM_MasterConfigTypeDef master = {0};

    master.MasterOutputTrigger = TIM_TRGO_UPDATE;
    master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &master) != HAL_OK)
    {
        Error_Handler();
    }

    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
}

static void TIM4_Init(void)
{
    __HAL_RCC_TIM4_CLK_ENABLE();

    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 16000U - 1U;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = VREF_PERIOD_MS - 1U;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
    {
        Error_Handler();
    }

    TIM_MasterConfigTypeDef master = {0};
    master.MasterOutputTrigger = TIM_TRGO_UPDATE;
    master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &master) != HAL_OK)
    {
        Error_Handler();
    }
}

static void TIM5_Init(void)
{
    __HAL_RCC_TIM5_CLK_ENABLE();

    htim5.Instance = TIM5;
    htim5.Init.Prescaler = MOTOR_PWM_PSC - 1U;
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = MOTOR_PWM_ARR;
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
    {
        Error_Handler();
    }

    TIM_OC_InitTypeDef pwmChannel = {0};

    pwmChannel.OCMode = TIM_OCMODE_PWM1;
    pwmChannel.Pulse = 0;
    pwmChannel.OCPolarity = TIM_OCPOLARITY_HIGH;
    pwmChannel.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&htim5, &pwmChannel, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
    }
}

int main(void)
{
    HAL_Init();

    GPIO_Init();
    UART2_Init();
    ADC_Init();

    TIM2_Init();
    TIM4_Init();
    TIM5_Init();

    setvbuf(stdout, NULL, _IONBF, 0);

    if (HAL_ADC_Start_IT(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_TIM_Base_Start(&htim4) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
    }

    while (1)
    {
        uint16_t raw = 0;
        uint32_t ledDuty = 0;
        uint32_t motorDuty = 0;
        uint32_t vdda = 0;
        bool ready = false;

        uint32_t primask = __get_PRIMASK();
        __disable_irq();

        if (g_new)
        {
            raw = g_raw;
            ledDuty = g_led_duty;
            motorDuty = g_motor_duty;
            vdda = g_vdda_mv;

            g_new = false;
            ready = true;
        }

        __set_PRIMASK(primask);

        if (ready)
        {
            uint32_t before = (uint32_t)raw * VDDA_NOMINAL_MV / ADC_MAX;
            uint32_t after = (uint32_t)raw * vdda / ADC_MAX;

            printf("raw=%u  vdda=%lu mV  before=%lu mV  "
                   "after=%lu mV  LED=%lu%%  Motor=%lu%%\r\n",
                   (unsigned)raw,
                   (unsigned long)vdda,
                   (unsigned long)before,
                   (unsigned long)after,
                   (unsigned long)(ledDuty * 100U / LED_PWM_STEPS),
                   (unsigned long)(motorDuty * 100U / MOTOR_PWM_STEPS));
        }
    }
}

#ifdef __cplusplus
extern "C"
{
#endif
    /* ---------------- Interrupt handlers ---------------- */

    void SysTick_Handler(void)
    {
        HAL_IncTick();
    }

    void ADC_IRQHandler(void)
    {
        HAL_ADC_IRQHandler(&hadc1);
    }

    int _write(int fd, char *ptr, int len)
    {
        (void)fd;

        if (len <= 0)
        {
            return 0;
        }

        if (len > 65535)
        {
            return -1;
        }

        if (HAL_UART_Transmit(&huart2,
                              (uint8_t *)ptr,
                              (uint16_t)len,
                              100U) != HAL_OK)
        {
            return -1;
        }

        return len;
    }

    /* ---------------- Callbacks ---------------- */

    void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
    {
        static uint16_t printCounter = 0;

        if (hadc->Instance != ADC1)
        {
            return;
        }

        uint16_t raw = (uint16_t)HAL_ADC_GetValue(hadc);

        uint32_t ledDuty = duty_from_raw(raw, LED_PWM_STEPS);
        uint32_t motorDuty = duty_from_raw(raw, MOTOR_PWM_STEPS);

        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, ledDuty);
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, motorDuty);

        g_raw = raw;
        g_led_duty = ledDuty;
        g_motor_duty = motorDuty;

        if (++printCounter >= PRINT_EVERY)
        {
            printCounter = 0;
            g_new = true;
        }
    }

    void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
    {
        if (hadc->Instance != ADC1)
        {
            return;
        }

        static uint16_t buffer[VREFINT_AVG_N] = {0};
        static uint32_t sum = 0;
        static uint8_t index = 0;
        static uint8_t count = 0;

        uint16_t raw = (uint16_t)HAL_ADCEx_InjectedGetValue(
            hadc, ADC_INJECTED_RANK_1);

        sum -= buffer[index];
        buffer[index] = raw;
        sum += raw;

        index++;
        if (index >= VREFINT_AVG_N)
        {
            index = 0;
        }

        if (count < VREFINT_AVG_N)
        {
            count++;
        }

        uint32_t average = sum / count;

        if (average != 0U)
        {
            g_vdda_mv = (uint32_t)VDDA_NOMINAL_MV * (*VREFINT_CAL_ADDR) / average;
        }
    }

#ifdef __cplusplus
}
#endif

void Error_Handler(void)
{
    __disable_irq();

    __HAL_RCC_GPIOA_CLK_ENABLE();

    HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_PIN, GPIO_PIN_RESET);

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = MOTOR_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(MOTOR_PORT, &gpio);

    while (1)
    {
        __NOP();
    }
}
