# Contributing to MyFOCLib

Thank you for taking the time to contribute!  This document explains the
coding standards, review requirements and workflow for the project.

---

## Table of Contents

1. [Code Style](#1-code-style)
2. [Embedded-Specific Guidelines](#2-embedded-specific-guidelines)
3. [Testing Requirements](#3-testing-requirements)
4. [Pull Request Process](#4-pull-request-process)
5. [Commit Message Convention](#5-commit-message-convention)
6. [Reporting Bugs](#6-reporting-bugs)

---

## 1. Code Style

All C code must conform to the style defined in [`.clang-format`](.clang-format).
Before opening a PR, auto-format every changed file:

```bash
find src include tests examples -name '*.[ch]' | xargs clang-format -i
```

Key rules (also encoded in `.clang-format`):

| Rule | Value |
|---|---|
| Indentation | 4 spaces — no tabs |
| Column limit | 100 characters |
| Brace style | Allman (opening brace on its own line for functions) |
| Naming — functions | `snake_case` (e.g. `pid_calc`, `bldc_get_rpm`) |
| Naming — types / structs | `snake_case_t` (e.g. `pid_t`, `bldc_motor_t`) |
| Naming — macros / constants | `UPPER_SNAKE_CASE` |
| Pointer style | Right-aligned: `float *ptr` not `float* ptr` |
| Header guards | `#ifndef MODULE_NAME_H` / `#define MODULE_NAME_H` / `#endif` |

---

## 2. Embedded-Specific Guidelines

MyFOCLib targets resource-constrained microcontrollers.  Please follow
these guidelines when adding or modifying code:

### Floating-point

- Prefer `float` over `double` to avoid accidental promotion on soft-FPU targets.
- Append `f` suffix to all float literals: `1.0f`, `3.14159f`.
- If adding fixed-point arithmetic, document the Q-format clearly.

### Memory allocation

- **No dynamic allocation** (`malloc`, `calloc`, etc.) inside the library.
  All state is managed by the caller via stack-allocated or statically
  allocated structs.
- Document any stack usage that is larger than 64 bytes in a function's
  Doxygen comment.

### Hardware-related code

- Hardware register access must be commented with the peripheral name,
  register name and purpose, e.g.:

  ```c
  TIM1->CCR1 = (uint32_t)(duty * TIM1->ARR);  /* Phase-U PWM duty cycle */
  ```

- Platform-specific code must be guarded with preprocessor macros so the
  portable implementation remains functional on all targets.

### Avoiding common pitfalls

- Do **not** use `printf`, `scanf` or other I/O calls inside the library
  itself; these are only acceptable in `tests/` and `examples/`.
- Avoid unbounded loops; any poll/wait loop must have a timeout.
- Initialise **all** struct members; prefer `memset(…, 0, sizeof(…))` or
  explicit field assignments.

---

## 3. Testing Requirements

### Core function modifications

Any PR that modifies a public function in `src/` **must** include or update
a corresponding test case in `tests/unit_test_foc.c`.

### Test criteria

- Tests must pass with no failures on Linux with GCC (Debug build, ASan/UBSan enabled).
- Tests must be deterministic — no randomness, no timing dependencies.
- Each `ASSERT` must have a descriptive message string.

### Running tests locally

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build
ctest --test-dir build --output-on-failure
```

---

## 4. Pull Request Process

1. **Fork** the repository and create a feature branch from `main`:
   ```
   git checkout -b feature/my-improvement
   ```

2. **Make your changes** following the code style and embedded guidelines
   above.

3. **Add / update tests** for any modified public API.

4. **Build and test** locally:
   ```bash
   cmake -B build -DCMAKE_BUILD_TYPE=Debug && cmake --build build
   ctest --test-dir build --output-on-failure
   ```

5. **Format your code**:
   ```bash
   find src include tests examples -name '*.[ch]' | xargs clang-format -i
   ```

6. **Update `CHANGELOG.md`** in the `[Unreleased]` section.

7. **Open the PR** with a clear title and description:
   - What problem does this solve?
   - What approach was taken?
   - How was it tested?
   - Any known limitations or follow-up work?

8. At least one maintainer must approve the PR before it is merged.

---

## 5. Commit Message Convention

Follow the [Conventional Commits](https://www.conventionalcommits.org/)
format:

```
<type>(<scope>): <short summary>

[optional body]

[optional footer]
```

**Types:** `feat`, `fix`, `docs`, `style`, `refactor`, `test`, `chore`

**Examples:**

```
feat(pid): add derivative filter to reduce noise
fix(filter): correct wrap-around in filter_reset
docs(api): add parameter table for foc_svm
test(pid): add anti-windup saturation test case
```

---

## 6. Reporting Bugs

Please [open an issue](https://github.com/XiangDeng917/MyFOCLib/issues/new)
and include:

- A minimal reproducible example (ideally a failing unit test).
- The compiler, OS and version you are using.
- Expected vs. actual behaviour.
- Any error messages or assertion failures.
