#![no_std]
#![no_main]

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

    loop {
        // --- Collect system info via RX ---
        let mut payload = [0u8; 512];
        let payload_len = system_info::collect(&mut uart, &mut payload);

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

        Timer::after(Duration::from_secs(30)).await;
    }
}
