# Hardware Engineering & PCB Layout Documentation

Contains schematics, PCB design files, layouts, and validation procedures for the isolated gate-drive power stage.

## Hardware Engineering & PCB Layout

This directory contains the complete hardware architecture, schematics, and PCB layouts for the isolated gate-drive power stage. 

**Key Hardware Architecture:**
* **Power Stage Topology:** Discrete IR2101 gate drivers pushing AOT418 MOSFETs.
* **Galvanic Isolation:** High-speed 6N137 optocouplers to mitigate ground bounce and protect the logic layer from high *di/dt* switching noise.
* **Current Sensing:** Low-side shunt routing (2 mΩ) amplified via AD8418 for high-fidelity, synchronous PWM current reconstruction.
* **Validation:** Includes procedures for verifying physical deadtime constraints and hardware-level shoot-through prevention at 24kHz switching frequencies.
