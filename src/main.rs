#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::mode::Blocking;
use embassy_stm32::usart::{Config as UartConfig, UartTx};
use embassy_stm32::Config;
use embassy_time::{Duration, Timer};

async fn send_and_wait(tx: &mut UartTx<'static, Blocking>, cmd: &[u8], wait_ms: u64) {
    let _ = tx.blocking_write(cmd);
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

    // USART1 TX only: PA9 -> modem RX.
    let uart_config = UartConfig::default(); // 115200 8N1
    let mut tx = UartTx::new_blocking(p.USART1, p.PA9, uart_config).unwrap();

    // Give modem some time after power-up/reset.
    Timer::after(Duration::from_millis(1500)).await;

    // Required startup sequence before HTTP demo flow.
    send_and_wait(&mut tx, b"AT\r\n", 300).await;
    send_and_wait(&mut tx, b"ATE0\r\n", 300).await;
    send_and_wait(&mut tx, b"AT+CFUN=1\r\n", 800).await;
    send_and_wait(&mut tx, b"AT+CGATT=1\r\n", 1500).await;
    send_and_wait(&mut tx, b"AT+QICSGP=1,1,\"\",\"\",\"\"\r\n", 1000).await;
    send_and_wait(&mut tx, b"AT+NETOPEN\r\n", 5000).await;

    loop {
        // Strictly follow the user's demo order. No RX parsing or checks.
        send_and_wait(&mut tx, b"AT$HTTPOPEN\r\n", 300).await;
        send_and_wait(&mut tx, b"AT$HTTPPARA=http://test.nightkper.com,80,0,0\r\n", 300).await;
        send_and_wait(&mut tx, b"AT$HTTPRQH=Content-Length, 13\r\n", 300).await;
        send_and_wait(&mut tx, b"AT$HTTPACTION=1\r\n", 1000).await;
        send_and_wait(&mut tx, b"AT$HTTPDATA=13\r\n", 500).await;
        send_and_wait(&mut tx, b"Hello, world!", 500).await;
        send_and_wait(&mut tx, b"AT$HTTPSEND\r\n", 500).await;
        send_and_wait(&mut tx, b"AT$HTTPDATA=0\r\n", 300).await;
        send_and_wait(&mut tx, b"AT$HTTPSEND\r\n", 500).await;

        Timer::after(Duration::from_secs(30)).await;
    }
}
