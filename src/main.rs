#![no_std]
#![no_main]

use core::fmt::Write;

use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::usart::{Config as UartConfig, Uart};
use embassy_stm32::Config;
use embassy_time::{Duration, Instant, Timer};

// ── Minimal byte-slice formatter ─────────────────────────────────────────────

struct Buf<'a> {
    b: &'a mut [u8],
    n: usize,
}

impl<'a> Buf<'a> {
    fn new(b: &'a mut [u8]) -> Self {
        Self { b, n: 0 }
    }
    fn as_bytes(&self) -> &[u8] {
        &self.b[..self.n]
    }
    fn len(&self) -> usize {
        self.n
    }
    fn push(&mut self, s: &[u8]) {
        let room = self.b.len() - self.n;
        let take = s.len().min(room);
        self.b[self.n..self.n + take].copy_from_slice(&s[..take]);
        self.n += take;
    }
}

impl Write for Buf<'_> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.push(s.as_bytes());
        Ok(())
    }
}

// ── USART1 non-blocking RX via PAC RXNE ──────────────────────────────────────

fn usart1_rx() -> Option<u8> {
    let r = embassy_stm32::pac::USART1;
    if r.sr().read().rxne() {
        Some(r.dr().read().dr() as u8)
    } else {
        None
    }
}

/// Discard any stale bytes in the RX FIFO.
async fn rx_flush() {
    Timer::after(Duration::from_millis(20)).await;
    while usart1_rx().is_some() {}
}

/// Read bytes until `OK\r\n` / `ERROR\r\n` or timeout; returns slice into `buf`.
async fn recv<'a>(buf: &'a mut [u8], timeout_ms: u64) -> &'a [u8] {
    let dl = Instant::now() + Duration::from_millis(timeout_ms);
    let mut pos = 0;
    loop {
        if Instant::now() >= dl {
            break;
        }
        if let Some(b) = usart1_rx() {
            if pos < buf.len() {
                buf[pos] = b;
                pos += 1;
            }
            if pos >= 4 && &buf[pos - 4..pos] == b"OK\r\n" {
                break;
            }
            if pos >= 7 && &buf[pos - 7..pos] == b"ERROR\r\n" {
                break;
            }
        } else {
            Timer::after(Duration::from_micros(200)).await;
        }
    }
    &buf[..pos]
}

/// Wait for a literal byte sequence (e.g. `b">>"`) with KMP-style matching.
async fn wait_prompt(prompt: &[u8], timeout_ms: u64) -> bool {
    let dl = Instant::now() + Duration::from_millis(timeout_ms);
    let plen = prompt.len();
    let mut matched = 0usize;
    while Instant::now() < dl {
        if let Some(b) = usart1_rx() {
            if b == prompt[matched] {
                matched += 1;
                if matched == plen {
                    return true;
                }
            } else {
                matched = if b == prompt[0] { 1 } else { 0 };
            }
        } else {
            Timer::after(Duration::from_micros(200)).await;
        }
    }
    false
}

// ── Response parsers ──────────────────────────────────────────────────────────

/// Parse `+GPSST: fix,cn,lon,alt,lat;...`
/// Returns `(fix_ok, lon, alt, lat)` as byte slices into `resp`.
fn parse_gpsst(resp: &[u8]) -> Option<(bool, &[u8], &[u8], &[u8])> {
    let tag = b"+GPSST:";
    let i = resp.windows(tag.len()).position(|w| w == tag)? + tag.len();
    let i = i + resp[i..].iter().take_while(|&&b| b == b' ').count();
    let end = i
        + resp[i..]
            .iter()
            .position(|&b| b == b';' || b == b'\r')
            .unwrap_or(resp[i..].len());
    let line = &resp[i..end];
    let mut fields = line.splitn(6, |&b: &u8| b == b',');
    let fix = fields.next()?.trim_ascii().first() == Some(&b'1');
    let _cn = fields.next()?;
    let lon = fields.next()?.trim_ascii();
    let alt = fields.next()?.trim_ascii();
    let lat = fields.next()?.trim_ascii();
    Some((fix, lon, alt, lat))
}

/// Parse `+CSQ: rssi,ber` — returns rssi as bytes.
fn parse_rssi(resp: &[u8]) -> &[u8] {
    let tag = b"+CSQ:";
    if let Some(i) = resp.windows(tag.len()).position(|w| w == tag) {
        let i = i + tag.len();
        let i = i + resp[i..].iter().take_while(|&&b| b == b' ').count();
        let e = i + resp[i..].iter().position(|&b| b == b',').unwrap_or(resp[i..].len());
        return &resp[i..e];
    }
    b"99"
}

// ── LED blink task ────────────────────────────────────────────────────────────

#[embassy_executor::task]
async fn blink_task(mut led: Output<'static>) {
    loop {
        led.set_low();
        Timer::after(Duration::from_millis(300)).await;
        led.set_high();
        Timer::after(Duration::from_millis(700)).await;
    }
}

// ── Main ──────────────────────────────────────────────────────────────────────

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Config::default());

    // PC13 LED (active-low on Blue Pill)
    spawner
        .spawn(blink_task(Output::new(p.PC13, Level::High, Speed::Low)))
        .unwrap();

    // USART1 full-duplex
    //   PA9  = TX → connect to 4G module RX
    //   PA10 = RX ← connect to 4G module TX  (add this wire!)
    let mut uart =
        Uart::new_blocking(p.USART1, p.PA10, p.PA9, UartConfig::default()).unwrap();

    // Scratch buffers
    let mut rbuf = [0u8; 384]; // AT response reader
    let mut cbuf = [0u8; 128]; // AT command builder

    // JSON body is built into this fixed buffer; 256 bytes is enough for one record.
    let mut jbuf = [0u8; 256];

    // Helper: send a literal byte slice
    macro_rules! tx {
        ($s:expr) => {
            uart.blocking_write($s).ok();
        };
    }

    // Helper: build an AT command via write! then send it
    macro_rules! tx_fmt {
        ($($arg:tt)*) => {{
            let mut b = Buf::new(&mut cbuf);
            let _ = write!(b, $($arg)*);
            let len = b.len();
            let mut arr = [0u8; 128];
            arr[..len].copy_from_slice(b.as_bytes());
            uart.blocking_write(&arr[..len]).ok();
        }};
    }

    // ── Initialise module ────────────────────────────────────────────────────
    tx!(b"AT\r\n");
    recv(&mut rbuf, 500).await;

    tx!(b"ATE0\r\n"); // echo off
    recv(&mut rbuf, 500).await;

    // APN — leave blank; module uses the SIM-provisioned APN for China SIMs
    tx!(b"AT+QICSGP=1,1,\"\",\"\",\"\"\r\n");
    recv(&mut rbuf, 800).await;

    // Open packet network (ERROR:902 = already open — both outcomes are fine)
    tx!(b"AT+NETOPEN\r\n");
    recv(&mut rbuf, 6000).await;

    // Enable GPS receiver
    // If your antenna is an active (LNA-powered) type, uncomment these three lines:
    //   tx!(b"AT+CGDRT=12,1\r\n");  recv(&mut rbuf, 500).await;
    //   tx!(b"AT+CGSETV=12,1\r\n"); recv(&mut rbuf, 500).await;
    //   tx!(b"AT+CGGETV=12\r\n");   recv(&mut rbuf, 500).await;
    tx!(b"AT+MGPSC=1\r\n");
    recv(&mut rbuf, 2000).await;

    // Give the GPS receiver a few seconds to start up
    Timer::after(Duration::from_secs(5)).await;

    // ── Report loop (every 30 s) ─────────────────────────────────────────────
    loop {
        // 1. Signal quality (AT+CSQ) ─────────────────────────────────────────
        rx_flush().await;
        tx!(b"AT+CSQ\r\n");
        let rssi_owned: ([u8; 8], usize) = {
            let resp = recv(&mut rbuf, 1500).await;
            let r = parse_rssi(resp);
            let n = r.len().min(8);
            let mut arr = [b'9'; 8];
            arr[..n].copy_from_slice(&r[..n]);
            (arr, n)
        };

        // 2. GPS position (AT+GPSST) ─────────────────────────────────────────
        rx_flush().await;
        tx!(b"AT+GPSST\r\n");
        // Copy parsed values out of rbuf before the next recv() call overwrites it.
        let gps: Option<(bool, [u8; 16], usize, [u8; 16], usize, [u8; 12], usize)> = {
            let resp = recv(&mut rbuf, 3000).await;
            parse_gpsst(resp).map(|(fix, lon, alt, lat)| {
                let ll = lat.len().min(16);
                let ol = lon.len().min(16);
                let al = alt.len().min(12);
                let mut lb = [0u8; 16];
                let mut ob = [0u8; 16];
                let mut ab = [0u8; 12];
                lb[..ll].copy_from_slice(&lat[..ll]);
                ob[..ol].copy_from_slice(&lon[..ol]);
                ab[..al].copy_from_slice(&alt[..al]);
                (fix, lb, ll, ob, ol, ab, al)
            })
        };

        // 3. Build JSON body ──────────────────────────────────────────────────
        //
        // GPS-fix example:
        //   {"fix":1,"lat":22.61,"lon":113.83,"alt":23.33,"rssi":15}
        //
        // No-fix example:
        //   {"fix":0,"rssi":15,"msg":"no_gps"}
        //
        let json_len = {
            let mut j = Buf::new(&mut jbuf);
            match gps {
                Some((true, lb, ll, ob, ol, ab, al)) => {
                    j.push(b"{\"fix\":1");
                    j.push(b",\"lat\":");
                    j.push(&lb[..ll]);
                    j.push(b",\"lon\":");
                    j.push(&ob[..ol]);
                    j.push(b",\"alt\":");
                    j.push(&ab[..al]);
                    j.push(b",\"rssi\":");
                    j.push(&rssi_owned.0[..rssi_owned.1]);
                    j.push(b"}");
                }
                _ => {
                    j.push(b"{\"fix\":0");
                    j.push(b",\"rssi\":");
                    j.push(&rssi_owned.0[..rssi_owned.1]);
                    j.push(b",\"msg\":\"no_gps\"}");
                }
            }
            j.len()
        };
        // jbuf[..json_len] now holds the ready JSON body.

        // 4. HTTP POST ────────────────────────────────────────────────────────
        //
        // Uses the large-data path (AT$HTTPACTION=1) so the JSON bytes are
        // fed as raw data after the ">>" prompt — no AT quoting issues with
        // the curly braces and inner quotes of JSON.

        rx_flush().await;

        tx!(b"AT$HTTPOPEN\r\n");
        recv(&mut rbuf, 1500).await;

        tx!(b"AT$HTTPPARA=http://47.107.144.252/location-report,8100\r\n");
        recv(&mut rbuf, 1000).await;

        tx!(b"AT$HTTPRQH=Content-Type,application/json\r\n");
        recv(&mut rbuf, 500).await;

        tx_fmt!("AT$HTTPRQH=Content-Length,{}\r\n", json_len);
        recv(&mut rbuf, 500).await;

        // Kick off the POST request
        tx!(b"AT$HTTPACTION=1\r\n");
        recv(&mut rbuf, 3000).await;

        // Send body in one chunk; module responds with ">>" when ready
        tx_fmt!("AT$HTTPDATA={}\r\n", json_len);
        if wait_prompt(b">>", 3000).await {
            // Copy JSON to a local array — jbuf borrow must be clean here
            let mut body = [0u8; 256];
            body[..json_len].copy_from_slice(&jbuf[..json_len]);
            uart.blocking_write(&body[..json_len]).ok();
            Timer::after(Duration::from_millis(100)).await;

            tx!(b"AT$HTTPSEND\r\n");
            recv(&mut rbuf, 2000).await;
        }

        // End-of-body sentinel required by the module
        tx!(b"AT$HTTPDATA=0\r\n");
        wait_prompt(b">>", 1000).await;
        tx!(b"AT$HTTPSEND\r\n");
        recv(&mut rbuf, 2000).await;

        tx!(b"AT$HTTPCLOSE\r\n");
        recv(&mut rbuf, 1000).await;

        // Wait before next report
        Timer::after(Duration::from_secs(30)).await;
    }
}
