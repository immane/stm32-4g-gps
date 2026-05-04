use embassy_stm32::mode::Blocking;
use embassy_stm32::pac;
use embassy_stm32::usart::Uart;
use embassy_time::{Duration, Instant};

/// Raw GPS status line returned by [`poll`].
pub struct GpsRecord {
    /// True when a `+GPSST:` line was received.
    pub has_data: bool,
    /// Raw `+GPSST: …` line as ASCII bytes.
    pub gpsst: [u8; 192],
    pub gpsst_len: usize,
}

// ── low-level helpers ─────────────────────────────────────────────────────────

fn drain_rx() {
    let deadline = Instant::now() + Duration::from_millis(200);
    while Instant::now() < deadline {
        let sr = pac::USART1.sr().read();
        // Read DR whenever RXNE or ORE is set; both bits require a DR read to clear.
        if sr.rxne() || sr.ore() {
            let _ = pac::USART1.dr().read().dr();
        }
    }
}

fn read_until_marker(out: &mut [u8], marker: &[u8], timeout_ms: u64) -> usize {
    let deadline = Instant::now() + Duration::from_millis(timeout_ms);
    let mut n = 0usize;
    while n < out.len() {
        if Instant::now() >= deadline { break; }
        let sr = pac::USART1.sr().read();
        if sr.rxne() || sr.ore() {
            let b = pac::USART1.dr().read().dr() as u8;
            // Discard the byte when ORE was set — it may be corrupt.
            if sr.ore() { continue; }
            out[n] = b;
            n += 1;
            if n >= marker.len() && &out[n - marker.len()..n] == marker { break; }
        }
    }
    n
}

fn send(uart: &mut Uart<'static, Blocking>, cmd: &[u8]) {
    let _ = uart.blocking_write(cmd);
}

/// Scan incoming bytes line-by-line; return the first line starting with `+GPSST:`.
fn read_gpsst_line(out: &mut [u8], timeout_ms: u64) -> usize {
    let deadline = Instant::now() + Duration::from_millis(timeout_ms);
    let mut line = [0u8; 192];
    let mut len = 0usize;
    while Instant::now() < deadline {
        let sr = pac::USART1.sr().read();
        if sr.rxne() || sr.ore() {
            let b = pac::USART1.dr().read().dr() as u8;
            // If ORE was set the byte may be the overwritten one; skip it.
            if sr.ore() { len = 0; continue; }
            if b == b'\r' || b == b'\n' {
                if len > 0 {
                    if line[..len].starts_with(b"+GPSST:") {
                        let copy = len.min(out.len());
                        out[..copy].copy_from_slice(&line[..copy]);
                        return copy;
                    }
                    len = 0;
                }
            } else if len < line.len() {
                line[len] = b;
                len += 1;
            }
        }
    }
    0
}

/// Query AT+ICCID; return raw digit bytes.
fn get_iccid(uart: &mut Uart<'static, Blocking>) -> ([u8; 24], usize) {
    let mut buf = [0u8; 128];
    send(uart, b"AT+ICCID\r\n");
    let n = read_until_marker(&mut buf, b"OK\r\n", 3000);

    // Find "+ICCID:" prefix then collect digits.
    let mut out = [0u8; 24];
    let prefix = b"+ICCID:";
    if let Some(p) = buf[..n].windows(prefix.len()).position(|w| w == prefix) {
        let mut i = p + prefix.len();
        while i < n && (buf[i] == b' ' || buf[i] == b'\t') { i += 1; }
        let mut len = 0usize;
        while i < n && len < out.len() {
            if buf[i].is_ascii_digit() { out[len] = buf[i]; len += 1; i += 1; }
            else if len > 0 { break; }
            else { i += 1; }
        }
        if len > 0 { return (out, len); }
    }
    (out, 0)
}

// ── public API ────────────────────────────────────────────────────────────────

/// One-time init: GPIO power pin, clear AGPS, power on, download & inject AGPS,
/// initial GPSST query, then power off.
pub fn init(uart: &mut Uart<'static, Blocking>) {
    drain_rx();
    let mut tmp = [0u8; 256];

    send(uart, b"AT+CGDRT=12,1\r\n");
    send(uart, b"AT+CGSETV=12,1\r\n");
    send(uart, b"AT+CGGETV=12\r\n");
    read_until_marker(&mut tmp, b"OK\r\n", 2000);

    send(uart, b"AT+MGPSGET=ALL,0\r\n");
    read_until_marker(&mut tmp, b"OK\r\n", 2000);

    send(uart, b"AT+MGPSC=1\r\n");
    read_until_marker(&mut tmp, b"OK\r\n", 2000);
    read_until_marker(&mut tmp, b"start up success.", 30_000);
    drain_rx();

    send(uart, b"AT+GPSMODE=1\r\n");
    read_until_marker(&mut tmp, b"OK\r\n", 2000);

    send(uart, b"AT+AGNSSGET=pos.asrmicro.com\r\n");
    read_until_marker(&mut tmp, b"AGPS download success.", 30_000);

    send(uart, b"AT+AGNSSSET\r\n");
    read_until_marker(&mut tmp, b"AGPS send success.", 10_000);

    // Discard initial GPSST; used only to warm up the receiver.
    send(uart, b"AT+GPSST\r\n");
    read_until_marker(&mut tmp, b"OK\r\n", 5000);

    send(uart, b"AT+MGPSC=0\r\n");
    read_until_marker(&mut tmp, b"poweroff success.", 10_000);
}

/// Power on GPS chip and wait for startup URC.
pub fn power_on(uart: &mut Uart<'static, Blocking>) {
    let mut tmp = [0u8; 128];
    send(uart, b"AT+MGPSC=1\r\n");
    read_until_marker(&mut tmp, b"OK\r\n", 2000);
    read_until_marker(&mut tmp, b"start up success.", 30_000);
}

/// Power off GPS chip and wait for poweroff URC.
pub fn power_off(uart: &mut Uart<'static, Blocking>) {
    let mut tmp = [0u8; 128];
    send(uart, b"AT+MGPSC=0\r\n");
    read_until_marker(&mut tmp, b"poweroff success.", 10_000);
}

/// Query AT+GPSST while GPS is already powered on.
/// Returns the raw `+GPSST:…` line inside a `GpsRecord`.
pub fn poll(uart: &mut Uart<'static, Blocking>) -> GpsRecord {
    let mut rec = GpsRecord {
        has_data: false,
        gpsst: [0u8; 192],
        gpsst_len: 0,
    };
    // Drain stale NMEA bytes and clear any ORE condition before sending the
    // command, otherwise the first byte(s) of the "+GPSST:" response may be
    // discarded while RXNE is still set from the NMEA stream.
    drain_rx();
    send(uart, b"AT+GPSST\r\n");
    let n = read_gpsst_line(&mut rec.gpsst, 10_000);
    rec.has_data = n > 0;
    rec.gpsst_len = n;
    rec
}

/// Query AT+ICCID (call while GPS is off to avoid NMEA interleaving).
pub fn read_iccid(uart: &mut Uart<'static, Blocking>) -> ([u8; 24], usize) {
    drain_rx();
    get_iccid(uart)
}

