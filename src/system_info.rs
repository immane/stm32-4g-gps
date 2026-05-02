use embassy_stm32::mode::Blocking;
use embassy_stm32::pac;
use embassy_stm32::usart::Uart;
use embassy_time::{Duration, Instant};

/// Discard any bytes already sitting in the USART1 RX register.
/// Runs for ~50 ms to flush any in-flight startup responses.
fn drain_rx() {
    let deadline = Instant::now() + Duration::from_millis(50);
    while Instant::now() < deadline {
        if pac::USART1.sr().read().rxne() {
            let _ = pac::USART1.dr().read().dr();
        }
    }
}

/// Read bytes from USART1 RX until "OK\r\n" is seen or `timeout_ms` elapses.
/// Does NOT call blocking_read; polls RXNE so it never hangs.
fn read_until_ok(out: &mut [u8], timeout_ms: u64) -> usize {
    let deadline = Instant::now() + Duration::from_millis(timeout_ms);
    let mut n = 0usize;
    while n < out.len() {
        if Instant::now() >= deadline {
            break;
        }
        if pac::USART1.sr().read().rxne() {
            let byte = pac::USART1.dr().read().dr() as u8;
            out[n] = byte;
            n += 1;
            if n >= 4 && &out[n - 4..n] == b"OK\r\n" {
                break;
            }
        }
    }
    n
}

/// Query ATI, AT+ICCID, AT+CSQ, AT+CCLK? and write raw responses into `out`.
/// Returns the number of bytes written.
pub fn collect(uart: &mut Uart<'static, Blocking>, out: &mut [u8]) -> usize {
    // Flush any startup-phase responses that were never read.
    drain_rx();

    let cmds: &[&[u8]] = &[
        b"ATI\r\n",
        b"AT+ICCID\r\n",
        b"AT+CSQ\r\n",
        b"AT+CCLK?\r\n",
    ];

    let mut idx = 0usize;
    let mut tmp = [0u8; 192];

    for cmd in cmds {
        let _ = uart.blocking_write(cmd);
        // 1500 ms per command — enough time for slow modem responses.
        let n = read_until_ok(&mut tmp, 1500);
        let copy = n.min(out.len().saturating_sub(idx));
        out[idx..idx + copy].copy_from_slice(&tmp[..copy]);
        idx += copy;
        // Clear tmp for next command.
        for b in tmp.iter_mut() {
            *b = 0;
        }
    }

    idx
}

/// Write `n` as decimal ASCII digits into `buf`. Returns number of bytes written.
pub fn usize_to_ascii(mut n: usize, buf: &mut [u8]) -> usize {
    if buf.is_empty() {
        return 0;
    }
    if n == 0 {
        buf[0] = b'0';
        return 1;
    }
    let mut tmp = [0u8; 10];
    let mut len = 0usize;
    while n > 0 && len < tmp.len() {
        tmp[len] = b'0' + (n % 10) as u8;
        n /= 10;
        len += 1;
    }
    let out_len = len.min(buf.len());
    for i in 0..out_len {
        buf[i] = tmp[len - 1 - i];
    }
    out_len
}
