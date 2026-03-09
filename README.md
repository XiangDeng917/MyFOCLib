# FOC Reusable C Library

A reusable C field-oriented-control (FOC) library for BLDC motors.

## AI Authorship Notice

This repository currently contains AI-generated draft documentation and AI-generated draft code comments.
Please review and update technical details before production use.

## Goals

1. Reusable control core independent from specific MCU SDK.
2. Clear hardware abstraction through a small set of porting hooks.
3. Easy extension for new sensors, boards, and control modes.

## Quick Start

```powershell
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

Detailed instructions: `docs/USAGE.md`

## Current Modules

1. Core FOC math and closed-loop flow: `foc.c`, `foc.h`
2. Motor driver logic and alignment: `src/BLDCMotor.c`
3. Magnetic sensor logic: `src/MagneticSensor.c`
4. PID and filters: `src/pid.c`, `src/my_filter.c`
5. Timing helpers: `src/delay.c`

## Project Docs

1. Usage: `docs/USAGE.md`
2. API: `docs/API_REFERENCE.md`
3. Structure plan: `docs/PROJECT_STRUCTURE.md`
4. Changelog: `docs/CHANGELOG.md`

## License

Add your preferred open-source license (MIT/BSD-3-Clause/Apache-2.0) before publishing.
