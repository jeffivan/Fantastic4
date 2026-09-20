# Physiological State Estimator — STM32F401CCU6

Infers heart rate, HRV, **respiration rate (from the ECG alone, via respiratory
sinus arrhythmia)** and a fused health state from two sensors: an AD8232
single-lead ECG front end and a DS18B20 temperature probe.

## Files

| File | Role |
|---|---|
| `config.h` | Every tunable constant, with units. The offline tuning script overwrites this file and nothing else. |
| `ecg.c/.h` | TIM2→ADC1→DMA acquisition at 250 Hz, Pan-Tompkins QRS detection, HR / RMSSD / quality. |
| `respiration.c/.h` | RSA extraction: resample → detrend → Hann → `arm_rfft_fast_f32` → band-limited peak → parabolic refinement. |
| `ds18b20.c/.h` | Non-blocking 1-Wire state machine, DWT µs timing, scratchpad CRC8. |
| `fuzzy.c/.h` | Mamdani engine, 14 rules, centroid defuzzification, graceful degradation. |
| `main.c` | Clock/peripheral init, cooperative scheduler, non-blocking UART ring, telemetry. |

## Output protocol (115200 8N1)

Two line types, both CRLF-terminated:

```
HR:72,HRV:45,RR:14.2,TEMP:36.8,STATE:NORMAL,RISK:22,QUALITY:GOOD,CONF:87
E:2051
```

* `RR` — breaths/min to 1 decimal; `---` when confidence is below
  `RESP_CONF_ACCEPT` or fewer than 20 beats contributed.
* `TEMP` — °C to 1 decimal; `---` when the reading is stale, out of range or
  fails CRC.
* `HR` / `HRV` — also `---` when unavailable. Not in the original field spec,
  but emitting `HR:0` would be a fabricated value and the brief forbids that.
* `STATE` — `NORMAL` / `STRESSED` / `FEVERISH` / `CONCERNING` / `UNKNOWN`.
* `QUALITY` — `GOOD` / `NOISY` / `LEADS_OFF`.
* `CONF` — 0–100, how much of the state rests on real inputs. Extra field;
  drop the two lines in `send_summary()` if your parser is strict.
* `E:<0..4095>` — raw ADC counts for the laptop plot.

### Decimation factor: why 2

| | bytes/s | % of link |
|---|---|---|
| Link capacity, 115200 8N1 (10 bits/byte) | 11520 | 100 % |
| Summary line, ~80 B at 1 Hz | 80 | 0.7 % |
| `E:` at 250 Hz (N=1), 8 B/line | 2000 | 17.4 % |
| **`E:` at 125 Hz (N=2), 8 B/line** | **1000** | **8.7 %** |
| `E:` at 62.5 Hz (N=4) | 500 | 4.3 % |

N=1 *fits*, so the choice is not forced by bandwidth. N=2 is chosen because:

1. **No aliasing.** The AD8232 in its heart-rate configuration band-limits the
   analog signal to ~40 Hz. 125 Hz is >3× that, so nothing folds back. N=4
   (62.5 Hz) would put the Nyquist edge at 31 Hz — inside the signal band.
2. **The decimation is filtered, not naive.** `ECG_STREAM_AVERAGE` averages the
   2 samples before dropping one: a 2-tap boxcar with a null at exactly 125 Hz.
3. **Headroom is the point.** 9.4 % total utilisation means a burst of dropped
   host reads, an added debug line, or a USB-serial adapter with a lazy driver
   cannot back-pressure the firmware into dropping telemetry.
4. **The QRS survives it.** At 125 Hz the ~30 ms R upstroke still gets 4 samples
   — enough for a plot that a judge can recognise as an ECG.

Change `ECG_STREAM_DECIMATION` in `config.h` if your host is slower.

---

## STM32CubeMX settings

Device: **STM32F401CCU6**, package UFQFPN48.

### System Core → RCC
* High Speed Clock (HSE): **Crystal/Ceramic Resonator**
* Low Speed Clock (LSE): Disable

### Clock Configuration tab
| Field | Value |
|---|---|
| Input frequency | 25 MHz |
| PLL Source Mux | HSE |
| PLLM | **/25** (→ 1 MHz VCO input) |
| PLLN | **×336** (→ 336 MHz VCO) |
| PLLP | **/4** |
| System Clock Mux | PLLCLK |
| AHB Prescaler | /1 → **HCLK 84 MHz** |
| APB1 Prescaler | **/2** → PCLK1 42 MHz, **APB1 timer clock 84 MHz** |
| APB2 Prescaler | /1 → PCLK2 84 MHz |
| PLLQ | /7 (unused; keeps CubeMX quiet about USB) |
| Flash latency | 2 WS (CubeMX sets this) |
| Voltage scaling | **Scale 1** — required above 60 MHz on the F401 |

*8 MHz-crystal board:* set PLLM to /8. *HSI only:* select HSI as the PLL
source, PLLM /16, PLLN ×336, PLLP /4, and change `CFG_PLL_M` in `config.h`.

### System Core → SYS
* Debug: **Serial Wire** (this is what keeps PA13/PA14 out of the GPIO pool)
* Timebase Source: SysTick

### Analog → ADC1
* **IN4** checked (PA4)
* Clock Prescaler: **PCLK2 divided by 4** (→ 21 MHz, max is 36)
* Resolution: **12 bits**
* Scan Conversion Mode: Disabled
* Continuous Conversion Mode: **Disabled** (the timer sets the rate)
* Discontinuous Conversion Mode: Disabled
* DMA Continuous Requests: **Enabled** ← required for circular DMA
* End Of Conversion Selection: EOC flag at the end of single conversion
* External Trigger Conversion Source: **Timer 2 Trigger Out event**
* External Trigger Conversion Edge: **Rising edge**
* Rank 1 → Channel 4, Sampling Time: **84 Cycles**

### Analog → ADC1 → DMA Settings
* Add → **ADC1**
* Stream: **DMA2 Stream 0**, Channel 0 (CubeMX fills this in)
* Direction: Peripheral To Memory
* Priority: **High**
* Mode: **Circular**
* Increment Address: Memory ✔, Peripheral ✘
* Data Width: **Half Word** / **Half Word**
* FIFO: Disabled (direct mode)

### Analog → ADC1 → NVIC Settings
* DMA2 stream0 global interrupt: **enabled**, preemption priority **1**

### Timers → TIM2
* Clock Source: **Internal Clock**
* Prescaler (PSC − 1): **83** → 1 MHz counter
* Counter Mode: Up
* Counter Period (ARR − 1): **3999** → 84 MHz / 84 / 4000 = **250.000 Hz exactly**
* Internal Clock Division: No Division
* auto-reload preload: Disable
* **Parameter Settings → Trigger Output (TRGO) Parameters:**
  * Master/Slave Mode: Disable
  * **Trigger Event Selection: Update Event**
* NVIC: TIM2 global interrupt **not** needed — TRGO is a hardware path.

### Connectivity → USART1
* Mode: **Asynchronous**
* Baud Rate: **115200**, Word Length 8 bits (no parity), Parity None, Stop Bits 1
* Data Direction: Receive and Transmit
* Over Sampling: 16 samples
* Pins: **PA9 = USART1_TX**, **PA10 = USART1_RX**

### Connectivity → USART1 → DMA Settings
* Add → **USART1_TX**
* Stream: **DMA2 Stream 7**, Channel 4
* Direction: Memory To Peripheral
* Priority: Medium
* Mode: **Normal** (not circular — each chunk is a discrete transfer)
* Increment Address: Memory ✔, Peripheral ✘
* Data Width: **Byte** / **Byte**
* FIFO: Disabled

### Connectivity → USART1 → NVIC Settings
* USART1 global interrupt: **enabled**, preemption priority **2**
  ⚠️ This one is easy to miss. HAL raises `HAL_UART_TxCpltCallback` from the
  USART transfer-complete interrupt, not the DMA one. Without it the ring
  transmits exactly one chunk and then stops forever.
* DMA2 stream7 global interrupt: **enabled**, preemption priority **2**

### GPIO (System Core → GPIO)
| Pin | Mode | Pull | Speed | Label |
|---|---|---|---|---|
| **PA4** | Analog mode | No pull-up/down | — | `ECG_IN` (ADC1_IN4) |
| **PA1** | **Output Open Drain** | **Pull-up** | **High** | `OW_DQ` — internal ~40 kΩ, no external resistor |
| **PB0** | Input mode | No pull-up/down | — | `LO_PLUS` (AD8232 drives it push-pull) |
| **PB1** | Input mode | No pull-up/down | — | `LO_MINUS` |
| **PC13** | Output Push Pull | No pull-up/down | Low | `LED` — active LOW, default output level **High** |
| PA9 / PA10 | Alternate Function Push Pull (AF7) | Pull-up | Very High | USART1 |
| PA13 / PA14 | (reserved by SYS = Serial Wire) | — | — | **do not touch** |

### NVIC priority summary
Priority grouping 4 (all bits preemption; `HAL_Init()` sets this).

| IRQ | Preempt | Why |
|---|---|---|
| SysTick | 15 | Only drives `HAL_GetTick()` |
| DMA2_Stream0 (ADC) | 1 | Tightest deadline; sets a flag and returns |
| DMA2_Stream7 (UART TX) | 2 | Throughput only |
| USART1 | 2 | Raises the TX-complete callback |

---

## Project settings (STM32CubeIDE)

1. **Add CMSIS-DSP.** Either:
   * link the prebuilt `libarm_cortexM4lf_math.a` from
     `Drivers/CMSIS/Lib/GCC/`, add `Drivers/CMSIS/DSP/Include` to the include
     paths, and define `ARM_MATH_CM4`; **or**
   * add `Drivers/CMSIS/DSP/Source` to the build (simplest is to add
     `arm_biquad_cascade_df1_f32.c`, `arm_rfft_fast_f32.c`,
     `arm_rfft_fast_init_f32.c`, `arm_cfft_f32.c`, `arm_cmplx_mag_f32.c`,
     `arm_mean_f32.c`, `arm_offset_f32.c`, `arm_mult_f32.c` plus the
     `arm_common_tables.c` / `arm_const_structs.c` they pull in).
2. **Symbols:** `ARM_MATH_CM4`, `__FPU_PRESENT=1U`, `ARM_MATH_LOOPUNROLL`
   (optional, faster vector loops).
3. **Floating point:** MCU Settings → FPU **FPv4-SP-D16**, Floating-point ABI
   **Hardware**. Without this the whole `float32_t` pipeline runs in software
   and the 128-point FFT goes from ~60 µs to ~1 ms.
4. **Optimisation:** `-O2`. `-Og` also works and is easier to debug.
5. If CubeMX generates `stm32f4xx_it.c`, delete the three IRQ handlers at the
   bottom of `main.c` (or the generated ones) — see the comment there.

---

## Timing budget

| Activity | Period | Cost | Notes |
|---|---|---|---|
| ADC conversion | 4 ms | 0 CPU | hardware, DMA |
| DMA half-block processing | 128 ms | ~120 µs | 32 samples × (biquad + derivative + square + MWI + peak test) |
| Respiration FFT | 5 s | ~60 µs | 128-pt real FFT + resampling on the FPU |
| Fuzzy inference | 1 s | ~25 µs | 14 rules × 51 output samples |
| 1-Wire step | main loop | ≤ 960 µs | interrupts masked ≤ 70 µs of that |
| Telemetry formatting | 1 s | ~10 µs | no `printf` |

Worst case the main loop is busy for about a millisecond (a 1-Wire reset) out
of a 128 ms ECG deadline. `ecg_get_block_overruns()` counts any time that
assumption is violated; it should stay at 0.

## Tuning-script contract

The Python tuner only ever rewrites `config.h`. Constants most worth sweeping,
roughly in order of impact:

* `ECG_THR_SIGNAL_FRACTION`, `ECG_THR_ABS_FLOOR` — detector sensitivity.
* `ECG_BP_HIGHPASS_HZ`, `ECG_BP_LOWPASS_HZ` — per-subject QRS band.
* `RESP_CONF_RATIO_MIN`, `RESP_CONF_RATIO_FULL` — how readily the respiration
  estimate is trusted; drives the `RR:---` rate.
* `FZ_TEMP_OFFSET_C` — skin-to-core calibration. Set this before touching any
  temperature membership function.
* All `FZ_*_A/B/C` breakpoints and `FZ_W_*` rule weights.
* `FZ_STATE_*_MIN` — where the state labels sit on the risk axis.

## Bring-up order

1. Flash, open a terminal at 115200. You should see the banner and `E:` lines
   at ~125 Hz sitting near 2048 with the electrodes off.
2. Touch the electrodes. Within ~2 s (the learning phase) PC13 should start
   flashing once per beat and `HR:` should populate.
3. Pull one electrode: `QUALITY:LEADS_OFF`, and HR/HRV/RR all go to `---`.
4. `TEMP:` appears within ~3 s. If it stays `---`, scope PA1: a rise time over
   ~5 µs means the internal pull-up is losing to cable capacitance — shorten
   the wire or fit an external 4k7.
5. `RR:` needs a full 32 s window plus 20 beats, so give it ~40 s of clean
   signal. Breathe deliberately at ~12 breaths/min and watch it converge.
