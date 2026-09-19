# Inferring the Unmeasured: An AI Physiological State Estimator

**graVITas '26 Hackathon — VIT | IEEE Signal Processing Society**

An STM32-based system that infers **hidden physiological states** — states that are never
directly sensed — from only two cheap, easily-obtained signals: a single-lead ECG and a
skin temperature reading.

> **Problem Statement 4 — AI-Based Physiological State Estimator Using Synthetic Patient Data**
> *"One of the major directions in modern digital-health research is attempting to infer
> physiological variables that are difficult or expensive to measure continuously, in which
> case we make use of the signals that are easier to obtain. Develop a physiological patient
> model by using the ECG and temperature sensor measurements to estimate hidden physiological
> states."*

---

## The Core Idea

We never attach a respiration sensor. We never attach a stress monitor. Yet the system
reports both.

The insight is **respiratory sinus arrhythmia**: breathing physically modulates the timing
between heartbeats. Inhale, and the heart speeds up fractionally; exhale, and it slows. That
modulation is buried in the beat-to-beat intervals of an ordinary ECG — a signal we already
have. Pull it out in the frequency domain and you recover the subject's respiration rate
without ever measuring their breath.

That inferred respiration rate is then fused with heart rate, heart rate variability, and
body temperature by an on-device fuzzy logic classifier into a categorical physiological
state.

**Everything runs on the microcontroller.** The laptop plots results and, beforehand, tunes
the algorithm — but no inference is offloaded to it.

---

## System Architecture

```
   ┌──────────────┐
   │  Electrodes  │
   └──────┬───────┘
          │ analog
   ┌──────▼───────┐        ┌──────────────┐
   │   AD8232     │        │   DS18B20    │
   │ ECG frontend │        │ temperature  │
   └──────┬───────┘        └──────┬───────┘
          │ PA4 (ADC1_IN4)        │ PA1 (1-Wire)
   ┌──────▼───────────────────────▼──────────────────────────┐
   │              STM32F401CCU6  @ 84 MHz, FPU               │
   │                                                         │
   │  ADC + DMA ──► R-peak detection ──► R-R interval series │
   │   250 Hz        (Pan-Tompkins)            │             │
   │                                           ▼             │
   │                            4 Hz resample → Hann → FFT   │
   │                              (CMSIS-DSP, 128-pt)        │
   │                                           │             │
   │                                           ▼             │
   │                              RESPIRATION RATE  ◄── hidden state
   │                                           │             │
   │   HR ──┬── HRV (RMSSD) ──┬────────────────┤             │
   │        │                 │                │             │
   │        ▼                 ▼                ▼      TEMP ──┤
   │     ┌──────────────────────────────────────────────┐    │
   │     │   FUZZY INFERENCE  (Mamdani, on-device)      │    │
   │     │   NORMAL / STRESSED / FEVERISH / CONCERNING  │    │
   │     └──────────────────────────────────────────────┘    │
   └────────────────────────┬────────────────────────────────┘
                            │ USART1 @ 115200
                            ▼
                    ┌───────────────┐
                    │ Laptop        │
                    │ live dashboard│
                    └───────────────┘
```

---

## Hardware

| Component | Role |
|---|---|
| STM32F401CCU6 (Black Pill) | Cortex-M4 @ 84 MHz with FPU — all acquisition, DSP and inference |
| AD8232 + electrodes | Single-lead ECG analog front end |
| DS18B20 | Body temperature (1-Wire) |
| ST-Link V2 | Flashing, debugging, and 3.3 V supply |
| USB-to-TTL | Telemetry link to the dashboard |

### Wiring

| From | Pin | To | Notes |
|---|---|---|---|
| AD8232 | OUTPUT | PA4 | ADC1_IN4 |
| AD8232 | LO+ / LO− | PB0 / PB1 | lead-off detection |
| AD8232 | 3.3V / GND | 3.3 V / GND rail | |
| DS18B20 | DQ | PA1 | internal pull-up — no external resistor |
| DS18B20 | VDD / GND | 3.3 V / GND rail | |
| USB-TTL | TX / RX | PA10 / PA9 | crossed |
| ST-Link | SWDIO / SWCLK | PA13 / PA14 | |
| ST-Link | 3.3V / GND | 3.3 V / GND rail | sole power source |
| — | — | PC13 | onboard LED, pulses on each detected beat |

---

## Signal Processing Pipeline

**1 — Acquisition.** TIM2 triggers ADC1 at a hardware-exact 250 Hz. DMA fills a circular
buffer in ping-pong fashion, so the CPU processes one half while the other fills. No sample
is ever dropped to CPU load.

**2 — Beat detection.** A Pan-Tompkins-style chain — bandpass (5–15 Hz) → derivative →
square → moving-window integration → adaptive threshold — locates R-peaks. A 200 ms
refractory window suppresses double-detection. Each detected beat pulses the onboard LED,
giving an immediate visual confirmation that detection is live and correct.

**3 — Respiration extraction (the hidden state).** R-R intervals arrive irregularly, once
per beat, so they are linearly interpolated onto a uniform 4 Hz grid. A 128-sample window
(32 s) is detrended, Hann-windowed, and transformed with `arm_rfft_fast_f32`. The magnitude
spectrum is searched **only** within 0.15–0.40 Hz — the physiological respiratory band,
equivalent to 9–24 breaths/min — and the peak is refined by parabolic interpolation across
its neighbouring bins for sub-bin resolution. Confidence is the ratio of peak magnitude to
in-band mean magnitude.

This runs once every 5 seconds rather than continuously: the output changes slowly, and
recomputing it per-sample would waste the power budget of a wearable for no accuracy gain.

**4 — Fuzzy inference.** A hand-written Mamdani engine takes heart rate, RMSSD, respiration
rate and temperature. Each input carries three triangular membership functions
(LOW / NORMAL / HIGH); roughly a dozen rules are combined with min-AND and max-aggregation,
then defuzzified by centroid into a 0–100 risk score that maps to the reported state.

---

## Synthetic Patient Data

We cannot tune a respiration estimator on real subjects, because we have no instrument that
tells us a real person's true respiration rate to the precision required for ground truth.
So we generate patients whose answers we know by construction.

Two generators are used:

- **Analytic R-R synthesis** — the interval series is built directly as
  `RR(t) = RR_mean + A_rsa·sin(2πf_resp·t) + noise`, giving mathematically exact ground
  truth for tuning the frequency analysis.
- **Full waveform synthesis** — realistic ECG waveforms with embedded respiratory
  modulation, passed through the complete chain including R-peak detection, to validate
  end-to-end rather than just the FFT stage.

Both sweep respiration rate (9–24 brpm), heart rate (50–120 BPM), RSA modulation depth,
additive noise, and randomly dropped beats (0 / 5 / 15 %) simulating electrode lead-off and
motion artifact.

The Python extractor is a **step-for-step reimplementation of the firmware's algorithm** —
same resample rate, same window length, same band, same interpolation. If the two diverged,
the tuned constants would not transfer. The tuning run emits `tuned_config.h`, which is
pasted directly into the firmware.

---

## Results

<!-- FILL THESE IN AFTER YOUR TUNING RUN — DO NOT SUBMIT WITH PLACEHOLDERS -->

| Metric | Result |
|---|---|
| Respiration MAE (clean signal) | `__ breaths/min` |
| Respiration MAE (15 % beat loss) | `__ breaths/min` |
| Fuzzy state classification accuracy | `__ %` |
| Synthetic patients evaluated | `__` |

**Live validation.** Beyond synthetic metrics, respiration output was checked against a
human subject breathing at a deliberately counted rate:

| Counted rate (brpm) | System estimate (brpm) |
|---|---|
| `__` | `__` |
| `__` | `__` |

Plots: `results/true_vs_estimated.png`, `results/error_vs_snr.png`,
`results/error_vs_beatloss.png`, `results/confusion_matrix.png`

---

## Repository Structure

```
├── firmware/              STM32 project (CubeIDE)
│   ├── Core/Src/
│   │   ├── main.c              cooperative scheduler, telemetry output
│   │   ├── ecg.c               ADC/DMA acquisition + R-peak detection
│   │   ├── respiration.c       CMSIS-DSP respiration extraction
│   │   ├── ds18b20.c           non-blocking 1-Wire driver
│   │   └── fuzzy.c             Mamdani inference engine
│   └── Core/Inc/
│       └── config.h            all tunable constants (from tuning run)
│
├── synthetic/             offline data generation + tuning
│   ├── synth.py                synthetic patient generators
│   ├── extractor.py            firmware-parity respiration algorithm
│   ├── fuzzy.py                Python mirror of the classifier
│   ├── tune.py                 grid search, validation, reporting
│   └── requirements.txt
│
├── dashboard/
│   └── dashboard.py            live telemetry visualisation
│
└── results/               generated plots, metrics, tuned_config.h
```

---

## Build & Run

**Firmware**
1. Open `firmware/` in STM32CubeIDE
2. Connect ST-Link, build, flash
3. LED on PC13 should begin pulsing once electrodes are attached

**Tuning (run before flashing final constants)**
```bash
cd synthetic
pip install -r requirements.txt
python tune.py            # writes results/tuned_config.h
```
Copy `results/tuned_config.h` over `firmware/Core/Inc/config.h`, rebuild, reflash.

**Dashboard**
```bash
cd dashboard
python dashboard.py --port COM3        # or /dev/ttyUSB0
```

**Telemetry format** — one line per second over USART1 @ 115200:
```
HR:72,HRV:45,RR:14.2,TEMP:36.8,STATE:NORMAL,RISK:22,QUALITY:GOOD
```
Unavailable fields are sent as `---` rather than fabricated.

---

## Engineering Decisions

**No external pull-up resistor.** Our kit contained no 4.7 kΩ resistor for the DS18B20's
1-Wire bus. Rather than lose the sensor, we drive the pin open-drain and enable the STM32's
internal pull-up (datasheet Table 54: R_PU typ. 40 kΩ). It is weaker than specification, so
scratchpad CRC8 validation is enforced on every read — a marginal edge produces a rejected
read and an honest "unknown", never a silently corrupt temperature.

**ECG on PA4, not PA0.** Many Black Pill boards wire a user push-button to PA0. Routing an
analog biosignal through a pin with a mechanical switch to ground invites artifacts, so the
ADC input was moved to ADC1_IN4.

**Nothing blocks.** A DS18B20 conversion takes 750 ms. A naive blocking read would stall the
250 Hz ECG pipeline and destroy beat detection, so temperature acquisition is a
start → poll → read state machine. 1-Wire bit timing uses the DWT cycle counter rather than
a hardware timer, leaving all TIM peripherals available for acquisition.

**Graceful degradation is a feature, not an afterthought.** Real wearables lose electrodes
and pick up motion. When leads detach, confidence collapses, or temperature fails CRC, the
fuzzy engine still produces a state from whatever inputs remain and reports reduced
confidence. The system never invents a number it does not have.

**Event-driven heavy computation.** The FFT runs every 5 s, not every sample — a deliberate
power/accuracy trade appropriate to a battery-powered wearable.

---

## Limitations & Future Work

- Respiration extraction relies on respiratory sinus arrhythmia, which weakens with age and
  in certain cardiac conditions; a second EDR channel using R-wave amplitude modulation
  would add redundancy.
- The 32-second analysis window means the estimate lags rapid changes in breathing.
- Fuzzy membership functions are tuned on synthetic populations; clinical deployment would
  require calibration against annotated patient recordings.
- Single-lead ECG limits morphology analysis — arrhythmia classification would benefit from
  additional leads.

---

## Team

**Team:** `__`
**Members:** `__`
**Institution:** `__`

---

*Built in 24 hours at graVITas '26.*
