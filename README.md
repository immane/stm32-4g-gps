# stm32-4g-gps (Rust embedded scaffold)

This is a Rust embedded project scaffold. It contains a minimal `no_std` entry point and placeholder configuration files.

Current behavior in `src/main.rs`:

- No TCP socket command is used.
- The firmware sends HTTP POST to `http://47.107.144.252:8100/mcu-report` every 30 seconds.
- System status payload assembly is implemented in `src/system_status.rs`.
- The payload currently includes the STM32 unique ID and raw placeholders for battery/signal/phone.

- Build (ensure your cross-compilation toolchain is installed):

```
cargo build --release
cargo flash --release --chip STM32F103C8 --probe 0483:3748:
```

- Please tell me which STM32 MCU you are using (for example: `STM32F401RE`, `STM32F407VG`, `STM32G431`, etc.). Once you provide the MCU part, I will add device-specific PAC/register examples (for example: blinking the onboard LED, configuring the UART for a GPS module, etc.).

Suggested next step: reply with your MCU part number and I will add low-level (PAC) examples, a tailored `memory.x`, `.cargo/config`/target settings, and flashing instructions.
