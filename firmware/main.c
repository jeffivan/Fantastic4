/**
 * @file    main.c
 * @brief   Physiological state estimator - orchestration and telemetry.
 *          STM32F401CCU6 "Black Pill", 84 MHz, HAL + CMSIS-DSP.
 *
 * =====================================================================
 * ASSUMPTIONS (stated explicitly, as required)
 * =====================================================================
 *  A1. The board is a WeAct Black Pill v3.x with a 25 MHz HSE crystal. For an
 *      8 MHz crystal set CFG_PLL_M to 8; to run from the internal 16 MHz HSI
 *      set CFG_PLL_M to 16 and switch the oscillator in SystemClock_Config().
 *  A2. VDD = VREF+ = 3.3 V. ADC readings are raw 12-bit counts throughout; the
 *      pipeline never needs volts, and the high-pass removes the bias anyway.
 *  A3. The AD8232 is wired in its single-lead heart-rate configuration
 *      (~0.5-40 Hz analog band-pass, output biased to VCC/2 so the ECG rests
 *      near ADC code 2048). This is what justifies the 250 Hz sample rate and
 *      the choice of streaming decimation.
 *  A4. AD8232 LO+ / LO- are push-pull outputs that go HIGH when an electrode
 *      loses contact, so PB0/PB1 are configured with no pull resistors.
 *  A5. Exactly ONE DS18B20 is on the 1-Wire bus, externally powered (not
 *      parasite powered), at its factory-default 12-bit resolution. The driver
 *      therefore uses SKIP ROM and never runs a ROM search.
 *  A6. The 1-Wire cable is short (< ~20 cm). The STM32's internal pull-up is
 *      ~40 kOhm, far weaker than the usual 4k7, so the bus rise time is the
 *      binding timing constraint (see ds18b20.c).
 *  A7. The DS18B20 measures skin/axillary temperature, which reads 1-2 C below
 *      core. FZ_TEMP_OFFSET_C in config.h converts it; it ships at 0.0.
 *  A8. The host keeps reading the serial port. If it stalls, output lines are
 *      DROPPED and counted - the firmware never blocks on the UART.
 *  A9. Beat timestamps use the ADC sample-counter timebase (ecg_now_ms()), not
 *      SysTick. Scheduling uses HAL_GetTick(). Both derive from the same PLL,
 *      so they do not drift apart; they simply have different origins.
 * A10. The telemetry spec only defines "---" for RR and TEMP. For consistency
 *      HR and HRV also degrade to "---" when unavailable, since graceful
 *      degradation is mandatory and emitting HR:0 would be a fabricated value.
 * A11. Single-threaded cooperative scheduling. Every *_process/*_task function
 *      runs in main context, so no module needs locking against another; only
 *      the DMA/UART ISRs touch shared state, and those use flags and a
 *      single-producer/single-consumer ring.
 * A12. CMSIS-DSP is linked as the hard-float Cortex-M4 variant
 *      (libarm_cortexM4lf_math.a) or compiled from source with __FPU_PRESENT.
 * A13. Uptime beyond ~49.7 days is out of scope: the millisecond counters are
 *      uint32 and all comparisons are written as unsigned differences, which
 *      wrap correctly, but the beat timestamps would not.
 * =====================================================================
 */

#include "stm32f4xx_hal.h"
#include "arm_math.h"

#include "config.h"
#include "ecg.h"
#include "respiration.h"
#include "ds18b20.h"
#include "fuzzy.h"

/* ===================================================================== */
/* Peripheral handles                                                    */
/* ===================================================================== */

ADC_HandleTypeDef  hadc1;
TIM_HandleTypeDef  htim2;
UART_HandleTypeDef huart1;
DMA_HandleTypeDef  hdma_adc1;
DMA_HandleTypeDef  hdma_usart1_tx;

static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
void Error_Handler(void);   /* not static: CubeMX-generated files call it */

/* ===================================================================== */
/* Non-blocking UART transmit ring                                       */
/* ===================================================================== */

/* Single producer (main context) writes head; single consumer (the UART TX
 * complete ISR) advances tail. The only shared mutable state is the busy flag
 * and the pending count, both touched inside a very short PRIMASK guard. A
 * full ring drops the message and increments a counter - it never blocks, and
 * it never busy-waits, because doing either would stall the ECG pipeline. */

#define TX_MASK (UART_TX_RING_SIZE - 1U)
#if (UART_TX_RING_SIZE & TX_MASK) != 0U
#error "UART_TX_RING_SIZE must be a power of two"
#endif

static uint8_t           s_tx_ring[UART_TX_RING_SIZE];
static volatile uint16_t s_tx_head;
static volatile uint16_t s_tx_tail;
static volatile uint16_t s_tx_pending;
static volatile uint8_t  s_tx_busy;
static volatile uint32_t s_tx_dropped;

static uint16_t uart_free(void)
{
    const uint16_t used = (uint16_t)((s_tx_head - s_tx_tail) & TX_MASK);
    return (uint16_t)(UART_TX_RING_SIZE - 1U - used);
}

static void uart_kick(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (!s_tx_busy && (s_tx_head != s_tx_tail)) {
        /* DMA needs a contiguous block, so a wrapped ring is sent in two goes. */
        const uint16_t chunk = (s_tx_head > s_tx_tail)
                             ? (uint16_t)(s_tx_head - s_tx_tail)
                             : (uint16_t)(UART_TX_RING_SIZE - s_tx_tail);
        s_tx_busy    = 1U;
        s_tx_pending = chunk;
        if (HAL_UART_Transmit_DMA(&huart1, &s_tx_ring[s_tx_tail], chunk) != HAL_OK) {
            s_tx_busy    = 0U;
            s_tx_pending = 0U;
        }
    }

    __set_PRIMASK(primask);
}

static int uart_write(const uint8_t *data, uint16_t len)
{
    uint16_t i;

    if (len > uart_free()) { s_tx_dropped++; return 0; }

    for (i = 0U; i < len; i++) {
        s_tx_ring[s_tx_head] = data[i];
        s_tx_head = (uint16_t)((s_tx_head + 1U) & TX_MASK);
    }
    uart_kick();
    return 1;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart1) { return; }
    s_tx_tail    = (uint16_t)((s_tx_tail + s_tx_pending) & TX_MASK);
    s_tx_pending = 0U;
    s_tx_busy    = 0U;
    uart_kick();                      /* chain straight into the next chunk   */
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart1) { return; }
    /* Abort and re-arm rather than leaving the ring wedged forever. */
    s_tx_busy    = 0U;
    s_tx_pending = 0U;
    uart_kick();
}

/* ===================================================================== */
/* Minimal ASCII formatting                                              */
/* ===================================================================== */

/* printf() with %f drags in a large, slow, non-reentrant formatter. These
 * helpers cover everything the protocol needs in a few dozen instructions. */

static uint16_t put_str(uint8_t *b, uint16_t i, const char *s)
{
    while (*s) { b[i++] = (uint8_t)*s++; }
    return i;
}

static uint16_t put_u32(uint8_t *b, uint16_t i, uint32_t v)
{
    char     tmp[10];
    uint8_t  n = 0U;

    do { tmp[n++] = (char)('0' + (v % 10U)); v /= 10U; } while (v != 0U);
    while (n--) { b[i++] = (uint8_t)tmp[n]; }
    return i;
}

/** Fixed-point with one decimal place, e.g. 36.75 -> "36.8". */
static uint16_t put_fix1(uint8_t *b, uint16_t i, float32_t v)
{
    int32_t scaled;

    if (v < 0.0f) { b[i++] = (uint8_t)'-'; v = -v; }
    scaled = (int32_t)(v * 10.0f + 0.5f);
    i = put_u32(b, i, (uint32_t)(scaled / 10));
    b[i++] = (uint8_t)'.';
    b[i++] = (uint8_t)('0' + (scaled % 10));
    return i;
}

static const char *quality_name(ecg_quality_t q)
{
    switch (q) {
    case ECG_QUALITY_GOOD:      return "GOOD";
    case ECG_QUALITY_NOISY:     return "NOISY";
    default:                    return "LEADS_OFF";
    }
}

/* ===================================================================== */
/* Telemetry                                                             */
/* ===================================================================== */

static uint8_t s_line[128];

static void send_summary(void)
{
    const respiration_result_t *resp = respiration_get();
    fuzzy_inputs_t  fin;
    fuzzy_result_t  fout;
    ecg_quality_t   quality = ecg_get_quality();
    float32_t       hr      = ecg_get_heart_rate();
    float32_t       rmssd   = ecg_get_rmssd();
    float32_t       temp_c  = 0.0f;
    uint8_t         temp_ok = ds18b20_get_temperature(HAL_GetTick(), &temp_c);
    uint16_t        i       = 0U;

    /* Availability rules. Leads off means the ECG-derived trio is not just
     * noisy, it is absent - respiration included, since it is computed FROM
     * the R-R series. Passing them through would be fabricating inputs. */
    const uint8_t leads_ok = (quality != ECG_QUALITY_LEADS_OFF);

    fin.hr_bpm     = hr;
    fin.hr_valid   = (leads_ok && hr > 0.0f) ? 1U : 0U;
    fin.rmssd_ms   = rmssd;
    fin.rmssd_valid= (leads_ok && rmssd >= 0.0f) ? 1U : 0U;
    fin.resp_brpm  = resp->rate_brpm;
    fin.resp_valid = (leads_ok && resp->valid &&
                      resp->confidence >= RESP_CONF_ACCEPT) ? 1U : 0U;
    fin.temp_c     = temp_c + FZ_TEMP_OFFSET_C;
    fin.temp_valid = temp_ok;

    fuzzy_evaluate(&fin, &fout);

    i = put_str(s_line, i, "HR:");
    if (fin.hr_valid) { i = put_u32(s_line, i, (uint32_t)(hr + 0.5f)); }
    else              { i = put_str(s_line, i, "---"); }

    i = put_str(s_line, i, ",HRV:");
    if (fin.rmssd_valid) { i = put_u32(s_line, i, (uint32_t)(rmssd + 0.5f)); }
    else                 { i = put_str(s_line, i, "---"); }

    i = put_str(s_line, i, ",RR:");
    if (fin.resp_valid) { i = put_fix1(s_line, i, resp->rate_brpm); }
    else                { i = put_str(s_line, i, "---"); }

    i = put_str(s_line, i, ",TEMP:");
    if (temp_ok) { i = put_fix1(s_line, i, fin.temp_c); }
    else         { i = put_str(s_line, i, "---"); }

    i = put_str(s_line, i, ",STATE:");
    i = put_str(s_line, i, fuzzy_state_name(fout.state));

    i = put_str(s_line, i, ",RISK:");
    i = put_u32(s_line, i, (uint32_t)(fout.risk + 0.5f));

    i = put_str(s_line, i, ",QUALITY:");
    i = put_str(s_line, i, quality_name(quality));

    /* Not in the required field list, but free and the first thing anyone will
     * ask when a state looks wrong: how much of it rested on real inputs. */
    i = put_str(s_line, i, ",CONF:");
    i = put_u32(s_line, i, (uint32_t)(fout.confidence * 100.0f + 0.5f));

    i = put_str(s_line, i, "\r\n");

    (void)uart_write(s_line, i);
}

static void stream_ecg(void)
{
    uint32_t n;
    uint16_t sample;

    /* Bounded per pass so a backlog can never monopolise a loop iteration, and
     * gated on the reserve so the 1 Hz summary always has room. */
    for (n = 0U; n < ECG_STREAM_MAX_PER_LOOP; n++) {
        uint8_t  buf[12];
        uint16_t i = 0U;

        if (uart_free() < UART_TX_RESERVE_BYTES) { break; }
        if (!ecg_stream_pop(&sample))            { break; }

        buf[i++] = (uint8_t)'E';
        buf[i++] = (uint8_t)':';
        i = put_u32(buf, i, sample);
        buf[i++] = (uint8_t)'\r';
        buf[i++] = (uint8_t)'\n';
        (void)uart_write(buf, i);
    }
}

/* ===================================================================== */
/* main                                                                  */
/* ===================================================================== */

int main(void)
{
    uint32_t t_resp, t_tel;

    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_TIM2_Init();
    MX_USART1_UART_Init();

    fuzzy_init();
    respiration_init();
    ds18b20_init(CFG_OW_PORT, CFG_OW_PIN);
    ecg_init(&hadc1, &htim2);           /* starts TIM2 -> ADC -> DMA          */

    (void)uart_write((const uint8_t *)
        "# physiological state estimator ready\r\n", 39U);

    t_resp = HAL_GetTick();
    t_tel  = HAL_GetTick();

    /* Cooperative scheduler. Nothing here blocks: every task either does a
     * bounded amount of work or returns immediately. Ordered by deadline -
     * the ECG blocks have the tightest one (128 ms), so they go first. */
    for (;;) {
        const uint32_t now = HAL_GetTick();

        ecg_process();                  /* filter + detect any ready DMA block */
        stream_ecg();                   /* drain decimated samples to the UART */
        ds18b20_task(now);              /* exactly one 1-Wire step, <= ~1 ms    */

        if ((uint32_t)(now - t_resp) >= RESP_UPDATE_PERIOD_MS) {
            t_resp += RESP_UPDATE_PERIOD_MS;
            if ((int32_t)(now - t_resp) > (int32_t)RESP_UPDATE_PERIOD_MS) {
                t_resp = now;           /* we fell far behind: resynchronise   */
            }
            /* ~60 us of FFT every 5 s. Uses the ADC timebase because that is
             * what the beat timestamps are expressed in. */
            (void)respiration_update(ecg_now_ms());
        }

        if ((uint32_t)(now - t_tel) >= TELEMETRY_PERIOD_MS) {
            t_tel += TELEMETRY_PERIOD_MS;
            if ((int32_t)(now - t_tel) > (int32_t)TELEMETRY_PERIOD_MS) {
                t_tel = now;
            }
            send_summary();
        }
    }
}

/* ===================================================================== */
/* Clock: 25 MHz HSE -> PLL -> 84 MHz SYSCLK                             */
/* ===================================================================== */

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    /* Scale 1 is required above 60 MHz on the STM32F401. */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    osc.OscillatorType       = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState             = RCC_HSE_ON;
    osc.PLL.PLLState         = RCC_PLL_ON;
    osc.PLL.PLLSource        = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM             = CFG_PLL_M;      /* 25 MHz / 25 = 1 MHz          */
    osc.PLL.PLLN             = CFG_PLL_N;      /* 1 MHz * 336 = 336 MHz VCO    */
    osc.PLL.PLLP             = RCC_PLLP_DIV4;  /* 336 / 4 = 84 MHz SYSCLK      */
    osc.PLL.PLLQ             = CFG_PLL_Q;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) { Error_Handler(); }

    clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                       | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;      /* HCLK  = 84 MHz               */
    clk.APB1CLKDivider = RCC_HCLK_DIV2;        /* PCLK1 = 42 MHz, TIM2 = 84 MHz*/
    clk.APB2CLKDivider = RCC_HCLK_DIV1;        /* PCLK2 = 84 MHz               */
    /* 2 wait states: 64 < HCLK <= 84 MHz at 2.7-3.6 V. */
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }
}

/* ===================================================================== */
/* GPIO                                                                  */
/* ===================================================================== */

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* PC13 status LED, active low - park it off. */
    HAL_GPIO_WritePin(CFG_LED_PORT, CFG_LED_PIN, GPIO_PIN_SET);
    g.Pin   = CFG_LED_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(CFG_LED_PORT, &g);

    /* PB0 / PB1 lead-off detect. The AD8232 drives these push-pull, so no
     * pull resistor is wanted; adding one would fight the driver. */
    g.Pin  = CFG_LEADOFF_P_PIN | CFG_LEADOFF_N_PIN;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(CFG_LEADOFF_P_PORT, &g);

    /* PA1 1-Wire. Open drain with the INTERNAL pull-up: writing 1 releases the
     * line, writing 0 drives it low, and IDR still reads the real pin level.
     * Start released so the bus idles high. */
    HAL_GPIO_WritePin(CFG_OW_PORT, CFG_OW_PIN, GPIO_PIN_SET);
    g.Pin   = CFG_OW_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;     /* fastest achievable falling edge    */
    HAL_GPIO_Init(CFG_OW_PORT, &g);

    /* PA4 is left alone here - HAL_ADC_MspInit configures it as analog.
     * PA13/PA14 (SWD) are never touched. */
}

/* ===================================================================== */
/* DMA                                                                   */
/* ===================================================================== */

static void MX_DMA_Init(void)
{
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* ADC first: a late half-transfer flag costs beat-detection latency,
     * while a late UART flag costs nothing but throughput. */
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

    HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
}

/* ===================================================================== */
/* ADC1 - PA4 / IN4, triggered by TIM2 TRGO                              */
/* ===================================================================== */

static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef ch = {0};

    hadc1.Instance                   = ADC1;
    /* PCLK2 84 MHz / 4 = 21 MHz, comfortably under the 36 MHz maximum. */
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode          = DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;   /* the timer sets the rate  */
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc1.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T2_TRGO;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 1;
    hadc1.Init.DMAContinuousRequests = ENABLE;    /* required for circular DMA */
    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) { Error_Handler(); }

    ch.Channel = CFG_ECG_ADC_CHANNEL;
    ch.Rank    = 1;
    /* 84 cycles + 12 for the SAR = 4.6 us per conversion. The AD8232 output is
     * op-amp buffered so a long sample window is not strictly needed, but at
     * 250 Hz (4 ms budget) it is free insurance against source impedance. */
    ch.SamplingTime = ADC_SAMPLETIME_84CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &ch) != HAL_OK) { Error_Handler(); }
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
    GPIO_InitTypeDef g = {0};

    if (hadc->Instance != ADC1) { return; }

    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    g.Pin  = GPIO_PIN_4;                /* PA4 = ADC1_IN4 */
    g.Mode = GPIO_MODE_ANALOG;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &g);

    hdma_adc1.Instance                 = DMA2_Stream0;
    hdma_adc1.Init.Channel             = DMA_CHANNEL_0;
    hdma_adc1.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode                = DMA_CIRCULAR;   /* ping-pong          */
    hdma_adc1.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_adc1.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) { Error_Handler(); }

    __HAL_LINKDMA(hadc, DMA_Handle, hdma_adc1);
}

/* ===================================================================== */
/* TIM2 - exact 250 Hz TRGO                                              */
/* ===================================================================== */

static void MX_TIM2_Init(void)
{
    TIM_ClockConfigTypeDef  src = {0};
    TIM_MasterConfigTypeDef mst = {0};

    /* APB1 is 42 MHz but its timer clock doubles to 84 MHz because the APB1
     * prescaler is not 1. 84 MHz / 84 / 4000 = 250.000 Hz exactly - no
     * rounding, which is what lets sample index convert to milliseconds by a
     * shift-free integer multiply. */
    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 83;          /* -> 1 MHz tick              */
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 3999;        /* -> 250 Hz update           */
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) { Error_Handler(); }

    src.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &src) != HAL_OK) { Error_Handler(); }

    mst.MasterOutputTrigger = TIM_TRGO_UPDATE;  /* drives ADC1 EXTSEL         */
    mst.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &mst) != HAL_OK) {
        Error_Handler();
    }
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) { __HAL_RCC_TIM2_CLK_ENABLE(); }
}

/* ===================================================================== */
/* USART1 - PA9 / PA10, 115200 8N1, DMA transmit                         */
/* ===================================================================== */

static void MX_USART1_UART_Init(void)
{
    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = UART_BAUDRATE;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) { Error_Handler(); }
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef g = {0};

    if (huart->Instance != USART1) { return; }

    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    g.Pin       = GPIO_PIN_9 | GPIO_PIN_10;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &g);

    hdma_usart1_tx.Instance                 = DMA2_Stream7;
    hdma_usart1_tx.Init.Channel             = DMA_CHANNEL_4;
    hdma_usart1_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode                = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority            = DMA_PRIORITY_MEDIUM;
    hdma_usart1_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK) { Error_Handler(); }

    __HAL_LINKDMA(huart, hdmatx, hdma_usart1_tx);

    /* HAL raises TxCpltCallback from the USART transfer-complete interrupt,
     * not from the DMA one, so the USART IRQ must be enabled or the ring will
     * stall after the very first chunk. */
    HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/* =====================================================================
 * Interrupt handlers
 *
 * If you regenerate this project with STM32CubeMX, it will also emit these
 * three handlers in Core/Src/stm32f4xx_it.c and the link will fail on
 * duplicate symbols. Pick one home for them: either delete this block and let
 * stm32f4xx_it.c own them (it already calls HAL_DMA_IRQHandler /
 * HAL_UART_IRQHandler on the same handles), or do not let CubeMX generate
 * stm32f4xx_it.c. Nothing else in this file collides.
 * ===================================================================== */

void DMA2_Stream0_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_adc1); }
void DMA2_Stream7_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_usart1_tx); }
void USART1_IRQHandler(void)       { HAL_UART_IRQHandler(&huart1); }

/* ===================================================================== */

void Error_Handler(void)
{
    __disable_irq();
    /* Nothing useful can run, so hold the LED on and wait for the watchdog or
     * a human. This is the only place in the firmware that spins forever. */
    HAL_GPIO_WritePin(CFG_LED_PORT, CFG_LED_PIN, CFG_LED_ON_STATE);
    for (;;) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file; (void)line;
    Error_Handler();
}
#endif
