# Bare-Metal Six-Step ESC Kernel 

This directory contains the development progression of a custom, bare-metal six-step ESC for the WeAct STM32G431. The firmware evolves from a basic open/closed-loop commutator to an advanced, EMI-hardened drive with dynamic stall recovery and auto-advancing BEMF tracking.

## Version History & Improvements

| Version | Key Features & Architectural Improvements |
| :--- | :--- |
| **V1.1** | • Initial bare-metal 6-step commutator using TIM1 center-aligned PWM.<br>• Injected BEMF scanning via ADC1.<br>• ELRS CRSF protocol parsing and BLDC audio engine. |
| **V1.2** | • **Stall Protection:** Introduced a 5ms closed-loop watchdog timeout.<br>• **Recovery:** Forces a 500ms coasting period upon sync loss before allowing a restart. |
| **V1.3** | • **Soft Start:** Implemented a slew rate limiter (15V/s ramp) for smooth spooling.<br>• **V/f Override:** Calculates safe spool voltage dynamically based on step delay. |
| **V1.4** | • **Dynamic Watchdog:** Timeout adapts to motor speed (4x predicted step duration).<br>• **High-Accel Slew Rate:** Increased voltage ramp to 120 V/s for agile throttle response.<br>• **Mandatory Cooldown:** Extended the stall recovery window to a strict 2 seconds. |
| **V1.5** | • **Auto-Advance Anti-Stall:** Forces a commutation step on timeout to keep momentum alive during false positives.<br>• **Strike System:** Implemented a 4-strike error counter before declaring a true physical prop jam. |
| **V1.6** | • **EMI Bulletproofing:** Hardware UART error clearing (ORE/NE/FE/PE) to prevent stall spikes from freezing telemetry.<br>• **ADC Valley Protection:** Clamped max duty cycle to 95% to guarantee a 5% window for clean BEMF sampling.<br>• **Radio Failsafe:** Instantly kills throttle if ELRS packets are delayed by >200ms.<br>• **Phantom Strike Fix:** Increased tolerance to 6 strikes and prevents phantom errors during open-loop. |
