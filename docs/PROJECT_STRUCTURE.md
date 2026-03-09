# Project Structure (Reusable C FOC Library)

## Recommended Layout

```text
foc/
  CMakeLists.txt
  README.md
  LICENSE
  CONTRIBUTING.md
  CHANGELOG.md
  docs/
    USAGE.md
    API_REFERENCE.md
    PROJECT_STRUCTURE.md
    MIGRATION.md
  include/
    foc/
      foc.h
      BLDCMotor.h
      MagneticSensor.h
      pid.h
      my_filter.h
      delay.h
  src/
    core/
      foc.c
      BLDCMotor.c
    sensor/
      MagneticSensor.c
    control/
      pid.c
      my_filter.c
    platform/
      delay.c
  port/
    interface/
      foc_port.h
    gd32/
      foc_port_gd32.c
    stm32/
      foc_port_stm32.c
  tests/
    smoke_test.c
    unit/
  examples/
    minimal/
    velocity_loop/
    angle_loop/
  .github/
    workflows/
      ci.yml
```

## Design Rules

1. Keep hardware-independent logic in `src/core`, `src/control`, and `src/sensor`.
2. Put MCU/HAL bindings only in `port/<chip_family>`.
3. Expose public API only from `include/foc/*.h`.
4. Keep one source of truth for each global/state object.
5. Make tests run on host without MCU headers.

## Suggested Next Steps

1. Introduce `foc_port.h` for all platform hooks (`setPWM`, `getRawCount`, tick, enable/disable).
2. Move headers from `src/` to `include/foc/` and keep `src/` private.
3. Add CI: Linux host build + smoke test on each push/PR.
4. Add semantic versioning tags and release notes.
