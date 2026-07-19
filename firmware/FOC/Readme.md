# Sensorless FOC Master Kernel

This directory contains the firmware evolution for a high-performance, sensorless Field Oriented Control (FOC) drive targeting the STM32G431. The control scheme utilizes a highly optimized 20kHz hard-sync loop, hardware CORDIC acceleration, and advanced observer models (Active Flux / SMO)

## Version History & Improvements

| Version / Era | Key Features & Architectural Improvements |
| :--- | :--- |
| **V2.x (Racer)** | • **Park-and-Spin State Machine:** Smooth transition from IDLE -> ALIGN -> RAMP -> CLOSED_LOOP[cite: 3].<br>• **SVPWM Modulator:** Space Vector PWM implementation for optimized voltage utilization[cite: 3].<br>• Continuous background BEMF observer tracking during open-loop spooling[cite: 3]. |
| **V3.1 - V3.2** | • **ZOH SMO / Active Flux Observer:** Transitioned to stable ZOH state feedback matrix and Active Flux calculations[cite: 3].<br>• **Lambda Measurement:** Dedicated state machine for identifying motor flux linkage (V.s/rad) at 150 rad/s[cite: 3].<br>• CORDIC-accelerated Cross-Product PLL for speed/angle tracking[cite: 3]. |
| **V3.4 - V4.2** | • **CORDIC Safety Fix:** Enforced `NARGS=1` constraints to prevent mathematical poisoning during sequential hardware reads[cite: 3].<br>• **Adaptive Feedforward:** Phase compensation mathematically subtracted to erase observer lead and lock onto the physical rotor[cite: 3].<br>• **Binary Telemetry:** High-speed, exactly packed 49-byte struct dumping at 1kHz for live data visualization[cite: 3]. |
| **V5.1.x** | • **Flux Weakening:** Dynamic lambda attenuation (reducing by 2% per 100 rad/s) for ultra-high-speed operation (>500 rad/s)[cite: 3].<br>• **Dynamic Gamma:** Adaptive observer stiffness to suppress high-frequency ADC noise at high RPMs[cite: 3].<br>• **Smooth Blending:** LERP-based `FOC_BLEND` state to seamlessly fade from open-loop voltage pushing to active throttle tracking[cite: 3]. |
| **V5.2** | • **Digital Flywheel:** Alpha-Beta Tracking filter utilizing predictive momentum and position/speed stiffness gains[cite: 3].<br>• **Predictive Phase Advance:** Compensates for the ~40µs CPU/PWM hardware delay by aiming voltage where the rotor *will* be[cite: 3].<br>• **D-Axis Feedforward:** Dynamic cross-coupling cancellation (`-omega * L * Iq`) mapped directly into the Inverse Park Transform[cite: 3]. |
