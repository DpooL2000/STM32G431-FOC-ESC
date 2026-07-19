# Bare-Metal Sensorless FOC Kernel

This repository contains the hardware designs and bare-metal firmware kernels for a high-frequency sensorless FOC drive engine optimized for low-inductance UAV motors.

<video width="100%" controls>
  <source src="./harware/videos/Gx012314_.m4v" type="video/mp4">
  Your browser does not support the video tag.
</video>

## System Architecture

*   **Microcontroller:** STM32G431 (170MHz Cortex-M4, utilizing internal CORDIC hardware co-processor).
*   **Target Dynamics:** 2300KV BLDC motor turning a 5-inch tri-blade propeller.
*   **Stator Identification Profile:**
    *   Phase Inductance (Ls): ~31 μH
    *   Phase Resistance (Rs): ~90 mΩ
    *   Flux Linkage (λ): ~0.00041 Wb
*   **Power Stage:** Discrete IR2101 gate drivers driving AOT418 MOSFETs, decoupled via high-speed 6N137 optocouplers for hardware shoot-through mitigation.
*   **Current Topology:** Injected low-side sensing through AD8418 current amplifiers paired with 2 mΩ shunts.

---

## Firmware Kernel Breakdown

### Core Execution Modules
*   **Pure Voltage Mode Engine:** Bypasses traditional high-lag PI current controllers to prevent phase delay at electrical frequencies exceeding 1500 Hz (30,000+ RPM).
*   **State-Space Flux Observer:** Zero-lag geometric flux estimation running at a rigid 24 kHz cycle using hardware CORDIC `get_Fast_Atan2()` calculations.
*   **Synchronous PWM Sampling:** Injected ADC conversions strictly aligned to center-count PWM timer events to read phase currents during low-side switching "quiet zones".

### The 5-Stage Spooling Sequence
1.  **OFF / CALIBRATE:** Measures operational amplifier baseline offsets.
2.  **ALIGN:** Asserts static vector to orient the rotor precisely at 0°.
3.  **OPEN_LOOP_RAMP:** Executes high-frequency blind step injection to break stator cogging torque.
4.  **FOC_BLEND:** Computes vector cross-fade from open-loop tracking to observer loop lock.
5.  **CLOSED_LOOP_FOC:** Complete handoff to pure sensorless geometric tracking.

---

## The High-RPM Non-Linear Boundary ("The Jerk")

A dedicated engineering post-mortem detailing why textbook industrial algorithms break down under microhenry, ultra-low mass environments:

*   **Cross-Coupling Destabilization:** At 40%–50% throttle, a microscopic 40 μs computational latency combined with intense reactive back-EMF cross-coupling (V = -ω * L * Iq) forces massive D-axis current (Id) spikes that derail the flux observer.
*   **Textbook Failures Documented:**
    *   **ADC Slew Limiting:** Clamping di/dt transitions starved the observer because a 31 μH coil at 12 V naturally generates an massive ~387,000 A/s rise rate.
    *   **Digital Flywheel PLLs:** Inertial filters introduced excessive mathematical phase lag, failing to track the sharp mechanical acceleration profiles of light drone props.

---

## Hardware Realities & Deadtime Distortion

*   **Pulse Devourment:** Insights on trying to execute an asymmetrical multirate loop (48 kHz PWM / 24 kHz FOC).
*   **The Math vs. Reality:** At 48 kHz, a 10% throttle command requires highly narrow 2.08 μs pulses. The gate driver's unavoidable physical deadtime (~0.5 μs) swallows a massive portion of the pulse width. This starves the stator coils of actual voltage relative to what the observer calculates, crashing the engine.
*   **Mitigation Strategy:** Reverted to a strictly synchronous 1:1 execution framework (24 kHz PWM / 24 kHz FOC) to preserve physical pulse widths.

---

## Live Telemetry Architecture

*   **The Binary Stream:** A raw, low-overhead 49-byte struct pushed at 1 kHz over a custom 2 Mbaud UART pipeline.
*   **UI Parser:** Custom PyQtGraph-driven GUI visualizing localized metrics (Id, Iq, Vd, Vq, and phase lag angle) in real-time.

---

## Repository Layout

*   **/hardware**: Layout files, optocoupler isolation circuits, and gate driver schematics.
*   **/firmware**: Clean bare-metal C modules, calibration tools, and parameter identification scripts.
