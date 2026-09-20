/**
 * @file    ds18b20.h
 * @brief   Non-blocking DS18B20 driver on a single open-drain GPIO.
 */

#ifndef DS18B20_H
#define DS18B20_H

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "arm_math.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @param port,pin  the 1-Wire DQ pin. Must already be configured as
 *                  GPIO_MODE_OUTPUT_OD with GPIO_PULLUP.
 * Also enables the DWT cycle counter used for microsecond timing.
 */
void ds18b20_init(GPIO_TypeDef *port, uint16_t pin);

/**
 * Advance the state machine by at most ONE bus operation.
 * Worst-case time inside this call is one reset pulse (~1 ms); worst-case
 * interrupts-disabled time is one timed bit slot (~70 us). The 750 ms
 * conversion is waited out across calls using @p now_ms, never inside one.
 */
void ds18b20_task(uint32_t now_ms);

/**
 * @param temp_c  written only when the return value is 1.
 * @return 1 if a CRC-valid, in-range reading is available and not stale.
 */
uint8_t ds18b20_get_temperature(uint32_t now_ms, float32_t *temp_c);

/* Diagnostics */
uint32_t ds18b20_get_crc_errors(void);
uint32_t ds18b20_get_bus_errors(void);
uint8_t  ds18b20_timing_ok(void);

#ifdef __cplusplus
}
#endif

#endif /* DS18B20_H */
