# MyFOCLib — Architecture Overview

## 1. High-Level Module Map

```
┌──────────────────────────────────────────────────────────────────┐
│                        User Application                          │
│  (examples/bldc_foc_demo.c  or  your own embedded firmware)      │
└───────────────────────────┬──────────────────────────────────────┘
                            │  calls
                            ▼
┌──────────────────────────────────────────────────────────────────┐
│                     BLDCMotor  (BLDCMotor.h/.c)                  │
│  • Owns the speed/torque control loop                            │
│  • Manages mode (IDLE / SPEED / TORQUE)                          │
│  • Exposes: init, set_target_speed, update, get_rpm, stop        │
└───────┬──────────────────────┬───────────────────────────────────┘
        │ uses                 │ uses
        ▼                      ▼
┌──────────────┐    ┌──────────────────────────────────────────────┐
│  PID (pid.h) │    │             FOC Core  (foc.h/.c)             │
│  speed loop  │    │  Clarke / Park / inverse transforms + SVM    │
└──────────────┘    └──────────────────────────────────────────────┘
        │                      │ uses
        │ uses                 ▼
┌───────────────────┐  ┌──────────────────────────────────────────┐
│ LP Filter         │  │  MagneticSensor  (MagneticSensor.h/.c)   │
│ (my_filter.h/.c)  │  │  Angle + velocity estimate from encoder  │
└───────────────────┘  └─────────────────────┬────────────────────┘
                                             │ calls (callback)
                                             ▼
                                    ┌─────────────────┐
                                    │  Hardware HAL   │
                                    │  (user-supplied │
                                    │  read_raw() fn) │
                                    └─────────────────┘
```

---

## 2. Data Flow per Control Tick

Each call to `bldc_update()` executes the following pipeline:

```
[Sensor HW] ──raw_count──► mag_sensor_update()
                                │
                          angle_rad, velocity_rads
                                │
                          × (60 / 2π × pole_pairs)
                                │
                           raw RPM
                                │
                          filter_update()  ──► current_speed (filtered RPM)
                                │
                          pid_calc(target_speed, current_speed)
                                │
                           pid_out  (duty scale -1…+1)
                                │
                          foc_park_inv()  ──► Vαβ reference
                                │
                          foc_svm(Vαβ, Vbus)
                                │
                         pwm_duty[U, V, W]
                                │
                        ► PWM Hardware (user drives registers)
```

---

## 3. Directory Layout

```
MyFOCLib/
├── include/            Public API headers (install target)
│   ├── foc.h
│   ├── BLDCMotor.h
│   ├── MagneticSensor.h
│   ├── pid.h
│   ├── my_filter.h
│   └── delay.h
├── src/                Implementation (.c files)
│   ├── foc.c
│   ├── BLDCMotor.c
│   ├── MagneticSensor.c
│   ├── pid.c
│   ├── my_filter.c
│   └── delay.c
├── tests/
│   └── unit_test_foc.c  Standalone unit tests (no framework dependency)
├── examples/
│   └── bldc_foc_demo.c  Complete runnable demo
├── docs/
│   ├── api_reference.md  Full API documentation
│   └── architecture.md   This file
├── scripts/
│   └── install.sh        Quick-install helper
├── CMakeLists.txt        Cross-platform build config
├── .clang-format         C code style definition
├── .gitignore
├── LICENSE               MIT
├── README.md
├── CONTRIBUTING.md
└── CHANGELOG.md
```

---

## 4. Porting to a New MCU

The only platform-specific code is confined to two places:

| File / function             | What to change                                |
|-----------------------------|-----------------------------------------------|
| `src/delay.c`               | Replace `nanosleep` branch with MCU timer loop|
| `mag_read_fn_t` callback    | Implement SPI/I2C read for your encoder IC    |

PWM output is intentionally **not** abstracted — write `motor.pwm_duty[]`
directly to your timer compare registers inside your main loop or ISR.

---

## 5. Control Loop Timing

The recommended sampling rate is **1 kHz** (1 ms period):

- `mag_sensor_update()` runs at 1 kHz
- `bldc_update()` runs at 1 kHz (same task)
- Speed filter `alpha ≈ 0.1` gives ~15 Hz effective bandwidth at 1 kHz

For high-performance applications, increase the rate to 4–8 kHz and
retune PID gains accordingly.
