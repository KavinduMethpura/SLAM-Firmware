# Development & Contributing

Guidelines for contributors and maintainers.

## Coding Style

- Use `clang-format` or the existing project style for C++ files. Keep format consistent with the code in `src/` and `lib/`.
- Keep functions short and single-purpose. Avoid blocking delays in the main loop; use non-blocking timers instead.

## Adding Features

1. Fork and create a feature branch.
2. Implement and test on hardware or with unit tests where possible.
3. Open a pull request with a clear description and any hardware test steps.

## Tests

- This firmware uses manual hardware validation. If you add simulation or unit tests, include instructions in `docs/setup.md` and a small test harness.

## Issues & Bug Reports

- When reporting a bug, include: hardware used, `platformio.ini` environment, serial logs (with timestamps), and steps to reproduce.
