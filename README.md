# Wearable-for-at-home-Sleep-Apnea-diagnosis

This repo contains the report and a few project files from
MSc Thesis by Manikanta Varma
Submitted to The University of Nottingham.

Short description:  Multisensor wearable + dashboard for real-time screening/monitoring of Obstructive Sleep Apnoea (OSA) at home. Integrates SpO₂/HR (PPG), abdominal IMU (effort/posture), nasal thermistor (airflow surrogate), and a microphone (snoring), with on-device signal processing, dual operating modes, wireless logging, and a live interface.

Overview:
Goal. Detect/apprise sleep apnoea events with minimal setup and minimal sleep disruption; enable long-term home use and CSV export for analysis.

Core features:

Multimodal sensing: SpO₂/HR (PPG), airflow surrogate (thermistor), respiratory effort & posture (IMU), snoring (mic).
Dual operating modes:
Mode-1 screening (comprehensive logging).
Mode-2 targeted monitoring.
On-device event logic (breath/no-breath, snore flag, posture, motion) with thresholds aligned to AASM event durations (≥10 s).
Live dashboard for near real-time plots; CSV export for offline review.
Validation: end-to-end lab run with 10 healthy adult volunteers to confirm signals, logging, and alerts under realistic use.

Hardware (reference design)
MCU: ESP32-WROOM-32 (FireBeetle board), Wi-Fi, multiple sleep modes; per-node 2000 mAh Li-ion.
Sensors
PPG SpO₂/HR: MAX30102 (dual-LED IR/RED; typical setup shown with 100 Hz sampling).
IMU: MPU-6050 (effort/posture; features include envelope, RMS, zero-crossings, fast/slow ratios).
Thermistor: oronasal airflow surrogate (breath oscillation, envelope).
Microphone: I²S MEMS mic (snoring band/features).
RTC: DS3231 for accurate timestamps.

Enclosures: three strap-mounted nodes (IMU abdomen, hub + PPG at wrist/finger, mic+thermistor head unit), with on/off switches and USB-C access.
