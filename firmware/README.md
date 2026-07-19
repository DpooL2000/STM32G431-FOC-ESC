# Firmware Kernel & Control Systems

This directory contains the bare-metal C implementation of the sensorless FOC loop, executing on the STM32G431 Cortex-M4.

## Hardware Abstraction & Pin Mapping
The system is tightly coupled to the MCU hardware timers and ADCs for deterministic execution:

| Function | Signal | STM32 Pin | Peripheral / Channel |
| :--- | :--- | :--- | :--- |
| **Phase U PWM** | High / Low | PA8 / PB13 | TIM1_CH1 / TIM1_CH1N |
| **Phase V PWM** | High / Low | PA9 / PB14 | TIM1_CH2 / TIM1_CH2N |
| **Phase W PWM** | High / Low | PA10 / PB15 | TIM1_CH3 / TIM1_CH3N |
| **Phase U Current** | Injected | PB1 | ADC1_IN12 |
| **Phase V Current** | Injected | PA0 | ADC2_IN1 |
| **Phase W Current** | Injected | PA3 | ADC1_IN3 / ADC2_IN3 |
| **BEMF Sensing** | Phase U, V, W | PB0, PA1, PA7 | ADC Channels 15, 2, 3 |
| **Telemetry** | UART TX / RX | PB10 / PB11 | USART3 |

## Synchronous Execution & Timing
* **Center-Aligned PWM:** The kernel utilizes center-aligned (triangle) PWM generation rather than edge-aligned. 
* **Quiet-Zone ADC Sampling:** ADC conversions are strictly triggered at the center of the PWM cycle (the "quiet zone") to guarantee clean phase current samples free of switching noise.
* **Execution Overhead:** The core control loop executes in exactly 2728 clock cycles. At the current clock speed, total computation time is 16.05 μs, operating well within the maximum 50 μs budget of the 24kHz control loop.

## State Observer & Diagnostics
To tune the sensorless tracking, a custom high-speed telemetry pipeline pushes real-time data to a PyQtGraph visualizer:
* **Clarke Transform Validation:** Live tracking of $I_{\alpha}$ and $I_{\beta}$ currents against injected phase voltages ensures symmetrical waveform synthesis.
* **Flux Observer Synchronization:** Diagnostic tools allow real-time visualization of the origin drift and the estimated flux angle versus the open-loop command to ensure a seamless "FOC Blend" handoff.
