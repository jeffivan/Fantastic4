Inferring the Unmeasured: An AI Physiological State Estimator

graVITas '26 Hackathon — VIT | IEEE Signal Processing Society

Overview

An STM32-based system that estimates hidden physiological states using only two low-cost signals: single-lead ECG and skin temperature.

Problem Statement 4: AI-Based Physiological State Estimator Using Synthetic Patient Data.

The system estimates respiration rate from ECG using respiratory sinus arrhythmia (RSA). Breathing changes the timing between heartbeats, allowing respiration to be inferred from ECG beat-to-beat intervals without a dedicated respiration sensor.

The estimated respiration rate is combined with heart rate, HRV (RMSSD), and temperature using an on-device Mamdani fuzzy logic classifier to report a physiological state.

All inference runs on the STM32. A laptop is used only for tuning and live visualization.

Hardware

Component

Role

STM32F401CCU6

Cortex-M4 @ 84 MHz; acquisition, DSP and inference

AD8232 + electrodes

Single-lead ECG

DS18B20

Body temperature

ST-Link V2

Programming, debugging and power

USB-to-TTL

Telemetry to laptop

Connections: AD8232 output → PA4 (ADC); lead-off → PB0/PB1. DS18B20 data → PA1. USB-TTL → PA9/PA10. ST-Link SWD → PA13/PA14.

Signal Processing

1. ECG acquisition: ADC samples at 250 Hz using DMA.

2. R-peak detection: A Pan-Tompkins-style process uses bandpass filtering, derivative, squaring, moving-window integration and adaptive thresholding.

3. Respiration estimation: R-R intervals are interpolated to 4 Hz, processed using a 128-point Hann-windowed FFT, and searched in the 0.15–0.40 Hz respiratory band (9–24 breaths/min). The estimate is updated every 5 seconds.

4. Physiological state: Heart rate, RMSSD, respiration rate and temperature are passed through a Mamdani fuzzy classifier. The output is a 0–100 risk score mapped to:

Normal

Stressed

Feverish

Concerning

Synthetic Patient Data

Synthetic data is used because accurate respiration ground truth is difficult to obtain without additional measurement equipment.

Two approaches are used:

Analytic R-R synthesis: Creates mathematically controlled R-R signals with known respiration rates.

Full waveform synthesis: Generates realistic ECG waveforms with respiratory modulation and tests the complete processing pipeline.

The datasets vary respiration rate, heart rate, RSA strength, noise and beat loss (0%, 5%, 15%).

The Python tuning pipeline mirrors the firmware algorithm so tuned parameters can be transferred directly to the STM32.

Results

Final metrics should be filled after the tuning run.

Metric

Result

Respiration MAE — clean signal

___ breaths/min

Respiration MAE — 15% beat loss

___ breaths/min

Fuzzy classification accuracy

___ %

Synthetic patients evaluated

___

Live validation should also compare the estimated respiration rate with a manually counted breathing rate.

Build and Operation

Open the firmware project in STM32CubeIDE.

Connect the ST-Link and flash the STM32.

Run the Python tuning script before final deployment.

Copy the tuned configuration into the firmware and reflash.

Run the dashboard to view live telemetry.

Example telemetry:

HR:72, HRV:45, RR:14.2, TEMP:36.8, STATE:NORMAL, RISK:22, QUALITY:GOOD

Unavailable measurements are reported as --- rather than fabricated.

Key Engineering Decisions

No external DS18B20 pull-up: The internal STM32 pull-up is used, with CRC validation to reject unreliable readings.

ECG on PA4: Chosen instead of PA0 to avoid interference from the onboard button.

Non-blocking temperature reading: Prevents the 750 ms DS18B20 conversion from interrupting ECG processing.

Graceful degradation: Missing or unreliable signals reduce confidence instead of producing fabricated values.

Event-driven FFT: Respiration analysis runs every 5 seconds to reduce unnecessary processing and power use.

Limitations and Future Work

RSA-based respiration estimation can weaken in some subjects and cardiac conditions.

The 32-second analysis window causes a delay when breathing changes rapidly.

Fuzzy rules are tuned using synthetic data and require validation with real annotated patient data.

A single ECG lead limits detailed cardiac analysis.

A second ECG-derived respiration method could improve reliability.

Team

Team: Fantastic 4
Members: Dharnesh, Preetam, Nischall, Jeffrey
Institution: Christ university
