# Changelog

All notable changes to **MyFOCLib** are documented here.

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and the project adheres to
[Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [Unreleased]

_Upcoming changes that have not yet been assigned a release version._

---

## [1.0.0] — 2024-01-01

### Added

- Initial public release of the MyFOCLib static library.
- `src/foc.c` / `include/foc.h`: Clarke, inverse-Clarke, Park,
  inverse-Park coordinate transforms and space-vector modulation (SVM).
- `src/pid.c` / `include/pid.h`: Positional PID controller with
  integral anti-windup and output clamping.
- `src/my_filter.c` / `include/my_filter.h`: First-order IIR low-pass
  filter with configurable smoothing coefficient.
- `src/MagneticSensor.c` / `include/MagneticSensor.h`: 14-bit magnetic
  encoder abstraction with velocity estimation and wrap-around handling.
- `src/BLDCMotor.c` / `include/BLDCMotor.h`: FOC speed-control loop
  (sensor → filter → PID → SVM) with idle/speed/torque modes.
- `src/delay.c` / `include/delay.h`: Portable millisecond / microsecond
  delay (POSIX `nanosleep` on Linux/macOS; bare-metal loop stub for MCUs).
- `CMakeLists.txt`: Cross-platform CMake build (Linux / macOS / Windows),
  static library, example executable, CTest unit tests, install rules.
- `tests/unit_test_foc.c`: 39 unit tests covering PID, filter, Clarke
  round-trip, Park round-trip and SVM range validation.
- `examples/bldc_foc_demo.c`: Complete runnable demo (sensor init →
  motor init → PID tuning → closed-loop speed control → stop).
- `docs/api_reference.md`: Full API documentation with parameter tables
  and usage examples.
- `docs/architecture.md`: Module map, data-flow diagram and porting guide.
- `scripts/install.sh`: One-command build-and-install helper.
- `.clang-format`: C code style (4-space indent, Allman braces, snake_case).
- `.gitignore`: Covers C/CMake build artefacts, IDE files and coverage data.
- `LICENSE`: MIT licence.
- `CONTRIBUTING.md`: Code style, embedded guidelines and PR process.

---

<!-- Links generated automatically by the release tooling -->
[Unreleased]: https://github.com/XiangDeng917/MyFOCLib/compare/v1.0.0...HEAD
[1.0.0]: https://github.com/XiangDeng917/MyFOCLib/releases/tag/v1.0.0
