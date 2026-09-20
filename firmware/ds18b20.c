/**
 * @file    ds18b20.c
 * @brief   Non-blocking DS18B20 1-Wire driver.
 *
 * TIMING STRATEGY
 *   1-Wire is a bit-banged protocol with microsecond-scale slots, and a
 *   DS18B20 conversion takes 750 ms. Both of those are hostile to a firmware
 *   whose headline requirement is "the ECG pipeline must never stall", so the
 *   driver is split at two levels:
 *
 *     - The 750 ms conversion is never waited on. The state machine records a
 *       HAL_GetTick() deadline and returns; ds18b20_task() simply does nothing
 *       until the deadline passes.
 *     - Every remaining operation is ATOMIC BUT SHORT. One call to
 *       ds18b20_task() performs exactly one reset pulse, or one byte write, or
 *       one byte read - never a whole transaction. Worst case is the reset
 *       pulse at ~960 us; a byte is ~560 us. A full temperature read is
 *       12 such steps spread over 12 main-loop passes.
 *
 *   Interrupts are masked only inside the individually timed portions of a bit
 *   slot (tens of microseconds), never across a whole byte. The ADC keeps
 *   converting regardless - DMA is hardware - so even the worst-case mask
 *   costs nothing but a little latency on the half-transfer flag, which has a
 *   128 ms budget.
 *
 * WHY THE INTERNAL PULL-UP NEEDS CARE
 *   The STM32's internal pull-up is ~40 kOhm, roughly 8x weaker than the 4k7
 *   this bus normally uses. With ~30 pF of bus capacitance the line needs
 *   ~3.6 us to rise, which eats into the 15 us window in which the master must
 *   sample a read slot. The read-slot constants in config.h are therefore set
 *   with a short master-low (3 us) and a late sample point (13 us into the
 *   slot). Keep the DQ wire short; check it on a scope before blaming the
 *   sensor.
 */

#include "ds18b20.h"
#include "config.h"
#include <string.h>

/* ===================================================================== */
/* DWT microsecond delay                                                 */
/* ===================================================================== */

static uint32_t s_cycles_per_us = 84U;
static uint8_t  s_dwt_ok;

static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    s_cycles_per_us = SystemCoreClock / 1000000U;
    if (s_cycles_per_us == 0U) { s_cycles_per_us = 84U; }

    /* If the counter does not run (some debug configurations), every bit slot
     * would collapse to zero length and the bus would be silently broken.
     * Detect it once and report it rather than producing garbage readings. */
    {
        const uint32_t t0 = DWT->CYCCNT;
        volatile uint32_t spin = 100U;
        while (spin--) { __NOP(); }
        s_dwt_ok = (DWT->CYCCNT != t0) ? 1U : 0U;
    }
}

static inline void delay_us(uint32_t us)
{
    const uint32_t start  = DWT->CYCCNT;
    const uint32_t target = us * s_cycles_per_us;
    while ((DWT->CYCCNT - start) < target) { /* spin */ }
}

/* ===================================================================== */
/* Bus primitives                                                        */
/* ===================================================================== */

static GPIO_TypeDef *s_port;
static uint16_t      s_pin;

/* Open-drain output: writing 1 releases the line to the internal pull-up,
 * writing 0 drives it hard low. The input data register still reflects the
 * real pin level while the port is configured as open-drain output, so no
 * mode switching is needed to read. */
static inline void ow_low(void)     { s_port->BSRR = (uint32_t)s_pin << 16U; }
static inline void ow_release(void) { s_port->BSRR = (uint32_t)s_pin; }
static inline uint32_t ow_read(void){ return (s_port->IDR & s_pin) ? 1U : 0U; }

/** @return 1 if at least one device answered with a presence pulse. */
static uint8_t ow_reset(void)
{
    uint32_t primask;
    uint32_t present;

    ow_low();
    delay_us(OW_RESET_LOW_US);          /* not microsecond-critical: any ISR
                                         * that lengthens it stays well inside
                                         * the 960 us the DS18B20 tolerates  */

    /* The presence pulse appears 15-60 us after release and lasts 60-240 us.
     * Only this short window needs protection from interrupts. */
    primask = __get_PRIMASK();
    __disable_irq();
    ow_release();
    delay_us(OW_RESET_PRESENCE_WAIT_US);
    present = (ow_read() == 0U) ? 1U : 0U;
    __set_PRIMASK(primask);

    delay_us(OW_RESET_RECOVERY_US);
    return (uint8_t)present;
}

static void ow_write_bit(uint32_t bit)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (bit) {
        ow_low();
        delay_us(OW_WRITE1_LOW_US);     /* must stay under 15 us              */
        ow_release();
        __set_PRIMASK(primask);         /* the rest of the slot is just idle  */
        delay_us(OW_WRITE1_HIGH_US);
    } else {
        ow_low();
        delay_us(OW_WRITE0_LOW_US);
        ow_release();
        __set_PRIMASK(primask);
        delay_us(OW_WRITE0_HIGH_US);
    }
}

static uint32_t ow_read_bit(void)
{
    uint32_t primask = __get_PRIMASK();
    uint32_t bit;

    __disable_irq();
    ow_low();
    delay_us(OW_READ_LOW_US);
    ow_release();
    delay_us(OW_READ_SAMPLE_US);        /* slave holds low for a 0; a 1 is the
                                         * weak pull-up winning the race      */
    bit = ow_read();
    __set_PRIMASK(primask);

    delay_us(OW_READ_RECOVERY_US);
    return bit;
}

static void ow_write_byte(uint8_t v)
{
    uint32_t i;
    for (i = 0U; i < 8U; i++) {         /* 1-Wire is LSB first */
        ow_write_bit((v >> i) & 1U);
    }
}

static uint8_t ow_read_byte(void)
{
    uint32_t i;
    uint8_t  v = 0U;
    for (i = 0U; i < 8U; i++) {
        if (ow_read_bit()) { v |= (uint8_t)(1U << i); }
    }
    return v;
}

/* Maxim/Dallas CRC-8, polynomial x^8 + x^5 + x^4 + 1 (0x8C reflected).
 * Bitwise rather than table-driven: 72 iterations every 2 seconds is free,
 * and it saves 256 bytes of flash. */
static uint8_t ow_crc8(const uint8_t *data, uint32_t len)
{
    uint8_t crc = 0U;
    uint32_t i, b;

    for (i = 0U; i < len; i++) {
        uint8_t byte = data[i];
        for (b = 0U; b < 8U; b++) {
            const uint8_t mix = (uint8_t)((crc ^ byte) & 0x01U);
            crc >>= 1;
            if (mix) { crc ^= 0x8CU; }
            byte >>= 1;
        }
    }
    return crc;
}

/* ===================================================================== */
/* State machine                                                         */
/* ===================================================================== */

typedef enum {
    DS_IDLE = 0,
    DS_CONV_RESET,
    DS_CONV_SKIP,
    DS_CONV_CMD,
    DS_CONV_WAIT,
    DS_READ_RESET,
    DS_READ_SKIP,
    DS_READ_CMD,
    DS_READ_BYTES,
    DS_BACKOFF
} ds_state_t;

static ds_state_t s_state;
static uint8_t    s_scratch[DS18B20_SCRATCHPAD_LEN];
static uint32_t   s_byte_idx;
static uint32_t   s_t_cycle_start;
static uint32_t   s_t_conv_start;
static uint32_t   s_t_backoff;
static uint32_t   s_t_last_good;
static uint8_t    s_have_reading;
static float32_t  s_temp_c;
static uint32_t   s_crc_errors;
static uint32_t   s_bus_errors;

static void enter_backoff(uint32_t now_ms)
{
    s_bus_errors++;
    s_t_backoff = now_ms;
    s_state     = DS_BACKOFF;
}

static void finish_read(uint32_t now_ms)
{
    int16_t   raw;
    float32_t t;

    /* The scratchpad CRC is the only thing standing between a glitched bit
     * slot and a fabricated body temperature. Never skip it. */
    if (ow_crc8(s_scratch, DS18B20_SCRATCHPAD_LEN - 1U)
            != s_scratch[DS18B20_SCRATCHPAD_LEN - 1U]) {
        s_crc_errors++;
        return;
    }

    raw = (int16_t)(((uint16_t)s_scratch[1] << 8) | (uint16_t)s_scratch[0]);

    /* 0x0550 == exactly 85.00 C is the DS18B20's power-on scratchpad value.
     * Seeing it before any successful conversion means the part reset, not
     * that the subject is on fire. */
    if (raw == 0x0550 && !s_have_reading) { return; }

    t = (float32_t)raw * 0.0625f;       /* 12-bit resolution: 1/16 C per LSB  */

    if (t < DS18B20_MIN_C || t > DS18B20_MAX_C) { return; }

    s_temp_c       = t;
    s_have_reading = 1U;
    s_t_last_good  = now_ms;
}

void ds18b20_init(GPIO_TypeDef *port, uint16_t pin)
{
    s_port = port;
    s_pin  = pin;

    dwt_init();
    ow_release();

    memset(s_scratch, 0, sizeof(s_scratch));
    s_state         = DS_IDLE;
    s_byte_idx      = 0U;
    s_t_cycle_start = 0U;
    s_t_conv_start  = 0U;
    s_t_backoff     = 0U;
    s_t_last_good   = 0U;
    s_have_reading  = 0U;
    s_temp_c        = 0.0f;
    s_crc_errors    = 0U;
    s_bus_errors    = 0U;
}

void ds18b20_task(uint32_t now_ms)
{
    if (!s_dwt_ok) { return; }          /* no usable microsecond clock        */

    switch (s_state) {

    case DS_IDLE:
        if ((now_ms - s_t_cycle_start) >= DS18B20_SAMPLE_PERIOD_MS) {
            s_t_cycle_start = now_ms;
            s_state = DS_CONV_RESET;
        }
        break;

    /* ---- kick off a conversion ---- */
    case DS_CONV_RESET:
        if (ow_reset()) { s_state = DS_CONV_SKIP; }
        else            { enter_backoff(now_ms);  }
        break;

    case DS_CONV_SKIP:                  /* one device on the bus -> SKIP ROM  */
        ow_write_byte(DS18B20_CMD_SKIP_ROM);
        s_state = DS_CONV_CMD;
        break;

    case DS_CONV_CMD:
        ow_write_byte(DS18B20_CMD_CONVERT_T);
        s_t_conv_start = now_ms;
        s_state = DS_CONV_WAIT;
        break;

    case DS_CONV_WAIT:                  /* 750 ms, spent doing anything else  */
        if ((now_ms - s_t_conv_start) >= DS18B20_CONV_TIME_MS) {
            s_state = DS_READ_RESET;
        }
        break;

    /* ---- read the scratchpad back ---- */
    case DS_READ_RESET:
        if (ow_reset()) { s_state = DS_READ_SKIP; }
        else            { enter_backoff(now_ms);  }
        break;

    case DS_READ_SKIP:
        ow_write_byte(DS18B20_CMD_SKIP_ROM);
        s_state = DS_READ_CMD;
        break;

    case DS_READ_CMD:
        ow_write_byte(DS18B20_CMD_READ_SCRATCH);
        s_byte_idx = 0U;
        s_state = DS_READ_BYTES;
        break;

    case DS_READ_BYTES:                 /* one byte per call, ~560 us each    */
        s_scratch[s_byte_idx++] = ow_read_byte();
        if (s_byte_idx >= DS18B20_SCRATCHPAD_LEN) {
            finish_read(now_ms);
            s_state = DS_IDLE;
        }
        break;

    case DS_BACKOFF:
        if ((now_ms - s_t_backoff) >= DS18B20_RETRY_MS) { s_state = DS_IDLE; }
        break;

    default:
        s_state = DS_IDLE;
        break;
    }
}

uint8_t ds18b20_get_temperature(uint32_t now_ms, float32_t *temp_c)
{
    /* A single dropped transaction should not blank the display; a sensor that
     * has been unplugged for ten seconds should. */
    if (!s_have_reading) { return 0U; }
    if ((now_ms - s_t_last_good) > DS18B20_STALE_MS) { return 0U; }

    *temp_c = s_temp_c;
    return 1U;
}

uint32_t ds18b20_get_crc_errors(void) { return s_crc_errors; }
uint32_t ds18b20_get_bus_errors(void) { return s_bus_errors; }
uint8_t  ds18b20_timing_ok(void)      { return s_dwt_ok;     }
