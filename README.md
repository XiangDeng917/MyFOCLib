# MyFOCLib

> **English** | [中文](#chinese)

A reusable **C static library** for Field-Oriented Control (FOC) of brushless
DC (BLDC) motors, suitable for bare-metal embedded targets as well as
Linux/macOS host-side simulation.

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

---

## Features

- **PID closed-loop speed control** with anti-windup and output clamping
- **Magnetic encoder abstraction** (AS5048A / AS5600 compatible) with
  automatic velocity estimation and 0/2π wrap-around handling
- **FOC coordinate transforms**: Clarke, Park, inverse Park, inverse Clarke
- **Space-Vector Modulation (SVM)** to generate three-phase PWM duty cycles
- **First-order low-pass filter** for speed/signal smoothing
- **Portable delay module** (POSIX `nanosleep` on Linux/macOS; stub for MCUs)
- Clean C11 API with full Doxygen-style documentation
- Cross-platform CMake build (Linux / macOS / Windows MinGW/MSVC)
- CTest-integrated unit tests (no external testing framework required)

---

## Quick Start

### Prerequisites

| Tool    | Minimum version |
|---------|-----------------|
| CMake   | 3.15            |
| GCC / Clang / MSVC | any recent C11-capable compiler |
| make / ninja | — |

### Clone & build

```bash
git clone https://github.com/XiangDeng917/MyFOCLib.git
cd MyFOCLib
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

### Run unit tests

```bash
ctest --test-dir build --output-on-failure
```

### Run the demo

```bash
./build/bldc_foc_demo
```

### Install to system

```bash
# default prefix /usr/local — may need sudo
./scripts/install.sh

# or custom prefix
./scripts/install.sh $HOME/.local
```

Installed files:

```
<prefix>/lib/libfoc.a
<prefix>/include/myfoclib/*.h
```

---

## Linking from Your Project

**CMake (FetchContent):**

```cmake
include(FetchContent)
FetchContent_Declare(myfoclib
    GIT_REPOSITORY https://github.com/XiangDeng917/MyFOCLib.git
    GIT_TAG        main
)
FetchContent_MakeAvailable(myfoclib)

target_link_libraries(my_app PRIVATE foc)
```

**Manual:**

```cmake
find_library(FOC_LIB foc HINTS /usr/local/lib)
target_include_directories(my_app PRIVATE /usr/local/include/myfoclib)
target_link_libraries(my_app ${FOC_LIB} m)
```

---

## Code Example — Motor Speed Control

```c
#include "BLDCMotor.h"
#include "MagneticSensor.h"

/* 1. Provide your hardware read function */
static uint16_t my_encoder_read(void) {
    return spi_read_as5048a();   /* your HAL */
}

int main(void) {
    /* 2. Init sensor: 14-bit CPR, 1 ms sample time */
    mag_sensor_t sensor;
    mag_sensor_init(&sensor, my_encoder_read, 16384, 0.001f);

    /* 3. Init motor: 7 pole pairs */
    bldc_motor_t motor;
    bldc_motor_init(&motor, 7, &sensor);

    /* 4. Tune PID and set target */
    bldc_set_speed_pid(&motor, 0.8f, 0.05f, 0.01f);
    bldc_set_target_speed(&motor, 1500.0f);   /* 1500 RPM */

    /* 5. 1 kHz control loop (e.g. in a timer ISR) */
    while (1) {
        bldc_update(&motor);
        /* Write motor.pwm_duty[0..2] to your PWM registers */
        set_pwm(motor.pwm_duty[0], motor.pwm_duty[1], motor.pwm_duty[2]);
        delay_ms(1);
    }
}
```

---

## API Reference

See [`docs/api_reference.md`](docs/api_reference.md) for the complete API.

Core functions at a glance:

| Function | Description |
|---|---|
| `pid_init()` / `pid_calc()` / `pid_reset()` | PID controller |
| `filter_init()` / `filter_update()` / `filter_reset()` | Low-pass filter |
| `mag_sensor_init()` / `mag_sensor_update()` | Magnetic encoder driver |
| `mag_sensor_get_angle()` / `mag_sensor_get_velocity()` | Read angle/speed |
| `bldc_motor_init()` / `bldc_set_target_speed()` / `bldc_update()` | Motor control |
| `bldc_get_rpm()` / `bldc_stop()` | Status and stop |
| `foc_clarke()` / `foc_park()` / `foc_svm()` | FOC transforms & SVM |

---

## Porting Notes

1. **Clock / delay**: `src/delay.c` auto-detects POSIX. On bare-metal,
   replace the `#else` branch with a loop calibrated to your MCU frequency,
   or substitute a hardware timer.
2. **Encoder read**: Provide a `uint16_t my_read(void)` function that returns
   the current absolute encoder count and pass it to `mag_sensor_init()`.
3. **PWM output**: After each `bldc_update()` call, write `motor.pwm_duty[]`
   (values 0–1) to your timer compare registers.
4. **Floating-point**: The library uses `float` throughout. Ensure your target
   has an FPU enabled, or replace with fixed-point arithmetic for soft-float
   cores (Cortex-M0/M0+).

---

## Project Structure

```
MyFOCLib/
├── include/          Public header files
├── src/              Implementation
├── tests/            Unit tests
├── examples/         Demo application
├── docs/             API reference & architecture
├── scripts/          install.sh
└── CMakeLists.txt
```

See [`docs/architecture.md`](docs/architecture.md) for a full module map.

---

## Contributing

Please read [`CONTRIBUTING.md`](CONTRIBUTING.md) before submitting a PR.

---

## Changelog

See [`CHANGELOG.md`](CHANGELOG.md).

---

## License

[MIT](LICENSE) © 2024 XiangDeng917

---

<a name="chinese"></a>

# MyFOCLib（中文说明）

适用于无刷直流（BLDC）电机磁场定向控制（FOC）的可复用 **C 静态库**，
支持裸机嵌入式目标及 Linux/macOS 主机端仿真。

## 主要特性

- **PID 闭环速度控制**（带抗积分饱和、输出限幅）
- **磁编码器抽象**（兼容 AS5048A / AS5600），自动速度估算与 0/2π 边界处理
- **FOC 坐标变换**：Clarke、Park、逆 Park、逆 Clarke
- **空间矢量调制（SVM）**，输出三相 PWM 占空比
- **一阶低通滤波器**，用于速度 / 信号平滑
- 可移植延时模块（POSIX 平台用 `nanosleep`，MCU 端提供裸机桩）
- 完整 Doxygen 风格文档
- 跨平台 CMake 构建（Linux / macOS / Windows MinGW / MSVC）
- CTest 集成单元测试（无需第三方测试框架）

## 快速开始

```bash
git clone https://github.com/XiangDeng917/MyFOCLib.git
cd MyFOCLib
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
./build/bldc_foc_demo
```

## 移植说明

1. **延时函数**：修改 `src/delay.c` 中的裸机分支，按目标 MCU 主频校准循环计数。
2. **编码器读取**：实现 `uint16_t my_read(void)` 函数，返回磁编码器绝对计数值。
3. **PWM 输出**：每次 `bldc_update()` 后，将 `motor.pwm_duty[0..2]`（0–1）写入定时器比较寄存器。
4. **浮点运算**：全库使用 `float`，建议启用硬件 FPU；软浮点核（Cortex-M0）需替换为定点运算。

## API 参考

详见 [`docs/api_reference.md`](docs/api_reference.md)。

## 许可证

[MIT](LICENSE) © 2024 XiangDeng917
