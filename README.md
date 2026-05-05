# stm32-4g-gps

Rust `no_std` firmware for **STM32F030C8** + 4G cellular modem with integrated GPS.  
Framework: [Embassy](https://embassy.dev/) — async executor, blocking UART, embassy-time.

## Hardware

| Signal | Pin |
|--------|-----|
| USART1 TX → modem RX | PA9 |
| USART1 RX ← modem TX | PA10 |
| LED (active-low) | PC13 |
| GPS power GPIO | PA12 (via AT+CGDRT/CGSETV) |

## Firmware overview

### Startup sequence
1. Modem init: `AT` → `ATE0` → `CFUN=1` → `CGATT=1` → `QICSGP` → `NETOPEN`
2. GPS one-time init (`src/gps.rs :: init`):
   - Configure GPIO power pin
   - Clear AGPS data (`AT+MGPSGET=ALL,0`)
   - Power on GPS chip (`AT+MGPSC=1`), wait for `start up success.`
   - Set GPS mode (`AT+GPSMODE=1`)
   - Download & inject AGPS (`AT+AGNSSGET` / `AT+AGNSSSET`)
   - Warm-up GPSST query, then power off

### Main loop (every ~90 s)
- Collect system info (ATI / CSQ / CCLK via `src/system_info.rs`)
- **GPS power-save state machine**:
  - Phase 1 — *Acquisition*: GPS stays on; poll `AT+GPSST` each cycle
  - Phase 2 — *Stable* (3 consecutive fixes): enter duty-cycle mode
  - Phase 3 — *Duty-cycle*: GPS off for 2 cycles, on for 1; revert to Phase 1 after 5 consecutive no-fix
- GPSST result: raw `+GPSST: …` line forwarded as-is; off-cycle POSTs reuse the cached line with ` CACHED=1` appended
- ICCID queried while GPS is off (avoids NMEA interleaving), cached between queries
- HTTP POST: `AT$HTTPOPEN` → `AT$HTTPPARA` → `AT$HTTPRQH` → `AT$HTTPACTION` → `AT$HTTPDATA` → `AT$HTTPSEND` → `AT$HTTPCLOSE`

### Payload fields (plain text, `\r\n` separated)
```
ATI=...
CSQ=...
CCLK=...
+GPSST: <fix>,<lon>,<lat>,...   (or with CACHED=1 suffix)
ICCID=...
GPS_PS=0|1
```

## Build & flash

```sh
cargo check --target thumbv6m-none-eabi
cargo flash --release --chip STM32F030C8 --probe 0483:3748
```

Target: `thumbv6m-none-eabi` (Cortex-M0, no FPU).
