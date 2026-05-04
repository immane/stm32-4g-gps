#![no_std]
#![no_main]

mod gps;
mod system_info;

use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::mode::Blocking;
use embassy_stm32::usart::{Config as UartConfig, Uart};
use embassy_stm32::Config;
use embassy_time::{Duration, Timer};

async fn send_and_wait(uart: &mut Uart<'static, Blocking>, cmd: &[u8], wait_ms: u64) {
    let _ = uart.blocking_write(cmd);
    Timer::after(Duration::from_millis(wait_ms)).await;
}

fn append_bytes(buf: &mut [u8], idx: &mut usize, data: &[u8]) {
    let remain = buf.len().saturating_sub(*idx);
    let copy = data.len().min(remain);
    buf[*idx..*idx + copy].copy_from_slice(&data[..copy]);
    *idx += copy;
}

const GPS_STABLE_FIX_TARGET: u8 = 3;
const GPS_NO_FIX_RECOVER_THRESHOLD: u8 = 5;
const GPS_TRACK_OFF_CYCLES: u8 = 2; // each loop is ~30s, so 2 => ~60s off

#[embassy_executor::task]
async fn blink_task(mut led: Output<'static>) {
    loop {
        let _ = led.set_low();
        Timer::after(Duration::from_millis(300)).await;
        let _ = led.set_high();
        Timer::after(Duration::from_millis(700)).await;
    }
}


#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let config = Config::default();
    let p = embassy_stm32::init(config);

    // LED blink on PC13 (active low on Blue Pill)
    let led = Output::new(p.PC13, Level::High, Speed::Low);
    spawner.spawn(blink_task(led)).unwrap();

    // USART1: PA9=TX -> modem RX, PA10=RX <- modem TX.
    let uart_config = UartConfig::default(); // 115200 8N1
    let mut uart = Uart::new_blocking(p.USART1, p.PA10, p.PA9, uart_config).unwrap();

    // Give modem some time after power-up/reset.
    Timer::after(Duration::from_millis(1500)).await;

    // Required startup sequence before HTTP flow.
    send_and_wait(&mut uart, b"AT\r\n", 300).await;
    send_and_wait(&mut uart, b"ATE0\r\n", 300).await;
    send_and_wait(&mut uart, b"AT+CFUN=1\r\n", 800).await;
    send_and_wait(&mut uart, b"AT+CGATT=1\r\n", 1500).await;
    send_and_wait(&mut uart, b"AT+QICSGP=1,1,\"\",\"\",\"\"\r\n", 1000).await;
    send_and_wait(&mut uart, b"AT+NETOPEN\r\n", 5000).await;

    // One-time GPS startup flow.
    let _ = gps::init(&mut uart);

    // GPS runtime state: start in continuous-search mode for stable acquisition.
    let mut gps_powered = false;
    let mut gps_stable_fix_count = 0u8;
    let mut gps_no_fix_streak = 0u8;
    let mut gps_track_off_cycles = 0u8;
    let mut power_save_enabled = false;

    // Cache ICCID and refresh while GPS is off.
    let mut cached_iccid = [0u8; 24];
    let mut cached_iccid_len = 0usize;

    // Cache last valid GPSST line so off-cycle POSTs still include GPS data.
    let mut cached_gpsst = [0u8; 192];
    let mut cached_gpsst_len = 0usize;

    loop {
        // --- Collect system info via RX ---
        let mut payload = [0u8; 768];
        let mut payload_len = system_info::collect(&mut uart, &mut payload);

        // If we are in power-save tracking mode, keep GPS off for configured cycles.
        if power_save_enabled && !gps_powered && gps_track_off_cycles > 0 {
            gps_track_off_cycles -= 1;
        }

        // Ensure GPS is powered when a query is due.
        if !gps_powered && (!power_save_enabled || gps_track_off_cycles == 0) {
            gps::power_on(&mut uart);
            gps_powered = true;
            // Give the receiver a short warm-up window before asking GPSST.
            Timer::after(Duration::from_secs(15)).await;
        }

        // --- Collect GPS position and append to payload ---
        // True only when a live poll succeeded this cycle.
        let mut gpsst_fresh = false;
        if gps_powered {
            let rec = gps::poll(&mut uart);
            if rec.has_data {
                // First field after "+GPSST: " is fix type; '0' means no fix.
                let fix = rec.gpsst[..rec.gpsst_len]
                    .windows(8)
                    .find(|w| *w == b"+GPSST: " || *w == b"+GPSST:\0")
                    .and_then(|_| {
                        let after = rec.gpsst[..rec.gpsst_len]
                            .iter()
                            .position(|&b| b == b':')
                            .map(|p| p + 1)?;
                        let val = rec.gpsst[after..rec.gpsst_len]
                            .iter()
                            .find(|&&b| b != b' ')?;
                        Some(*val != b'0')
                    })
                    .unwrap_or(false);

                if fix {
                    gps_stable_fix_count = gps_stable_fix_count.saturating_add(1);
                    gps_no_fix_streak = 0;
                    if gps_stable_fix_count >= GPS_STABLE_FIX_TARGET {
                        power_save_enabled = true;
                    }
                } else {
                    gps_stable_fix_count = 0;
                    gps_no_fix_streak = gps_no_fix_streak.saturating_add(1);
                    if gps_no_fix_streak >= GPS_NO_FIX_RECOVER_THRESHOLD {
                        power_save_enabled = false;
                        gps_track_off_cycles = 0;
                    }
                }

                // Update cache with fresh data.
                let copy = rec.gpsst_len.min(cached_gpsst.len());
                cached_gpsst[..copy].copy_from_slice(&rec.gpsst[..copy]);
                cached_gpsst_len = copy;
                gpsst_fresh = true;
            } else {
                gps_stable_fix_count = 0;
                gps_no_fix_streak = gps_no_fix_streak.saturating_add(1);
                if gps_no_fix_streak >= GPS_NO_FIX_RECOVER_THRESHOLD {
                    power_save_enabled = false;
                    gps_track_off_cycles = 0;
                }
            }
        }

        // Always emit GPSST (fresh or cached); if no data ever, emit UNKNOWN.
        if cached_gpsst_len > 0 {
            append_bytes(&mut payload, &mut payload_len, &cached_gpsst[..cached_gpsst_len]);
            if !gpsst_fresh {
                append_bytes(&mut payload, &mut payload_len, b" CACHED=1");
            }
            append_bytes(&mut payload, &mut payload_len, b"\r\n");
        } else {
            append_bytes(&mut payload, &mut payload_len, b"GPS_FIX=UNKNOWN\r\n");
        }

        // In power-save mode, power off after query and keep off for some loops.
        if power_save_enabled && gps_powered {
            gps::power_off(&mut uart);
            gps_powered = false;
            gps_track_off_cycles = GPS_TRACK_OFF_CYCLES;
        }

        // Refresh ICCID when GPS is off to avoid NMEA interleaving.
        if !gps_powered {
            let (iccid, iccid_len) = gps::read_iccid(&mut uart);
            if iccid_len > 0 {
                cached_iccid[..iccid_len].copy_from_slice(&iccid[..iccid_len]);
                cached_iccid_len = iccid_len;
            }
        }

        append_bytes(&mut payload, &mut payload_len, b"ICCID=");
        append_bytes(&mut payload, &mut payload_len, &cached_iccid[..cached_iccid_len]);
        append_bytes(&mut payload, &mut payload_len, b"\r\n");

        append_bytes(&mut payload, &mut payload_len, b"GPS_PS=");
        if power_save_enabled {
            append_bytes(&mut payload, &mut payload_len, b"1\r\n");
        } else {
            append_bytes(&mut payload, &mut payload_len, b"0\r\n");
        }

        // Build "AT$HTTPRQH=Content-Length, <N>\r\n"
        let mut rqh_cmd = [0u8; 48];
        let rqh_prefix = b"AT$HTTPRQH=Content-Length, ";
        rqh_cmd[..rqh_prefix.len()].copy_from_slice(rqh_prefix);
        let mut p = rqh_prefix.len();
        p += system_info::usize_to_ascii(payload_len, &mut rqh_cmd[p..]);
        rqh_cmd[p] = b'\r'; rqh_cmd[p + 1] = b'\n';
        let rqh_len = p + 2;

        // Build "AT$HTTPDATA=<N>\r\n"
        let mut hdata_cmd = [0u8; 24];
        let hdata_prefix = b"AT$HTTPDATA=";
        hdata_cmd[..hdata_prefix.len()].copy_from_slice(hdata_prefix);
        let mut q = hdata_prefix.len();
        q += system_info::usize_to_ascii(payload_len, &mut hdata_cmd[q..]);
        hdata_cmd[q] = b'\r'; hdata_cmd[q + 1] = b'\n';
        let hdata_len = q + 2;

        // --- HTTP POST flow (no RX checks) ---
        send_and_wait(&mut uart, b"AT$HTTPOPEN\r\n", 300).await;
        send_and_wait(&mut uart, b"AT$HTTPPARA=http://test.nightkper.com,80,0,0\r\n", 300).await;
        send_and_wait(&mut uart, &rqh_cmd[..rqh_len], 300).await;
        send_and_wait(&mut uart, b"AT$HTTPACTION=1\r\n", 1000).await;
        send_and_wait(&mut uart, &hdata_cmd[..hdata_len], 500).await;
        send_and_wait(&mut uart, &payload[..payload_len], 500).await;
        send_and_wait(&mut uart, b"AT$HTTPSEND\r\n", 500).await;
        send_and_wait(&mut uart, b"AT$HTTPDATA=0\r\n", 300).await;
        send_and_wait(&mut uart, b"AT$HTTPSEND\r\n", 5000).await;
        send_and_wait(&mut uart, b"AT$HTTPCLOSE\r\n", 1000).await;

        Timer::after(Duration::from_secs(60)).await;
    }
}
