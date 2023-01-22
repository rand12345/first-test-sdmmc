#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(alloc_error_handler)]
#![feature(default_alloc_error_handler)]
// extern crate alloc;

use core::ops::Deref;

use bxcan::filter::Mask32;
use bxcan::Fifo;
// use can_bit_timings::can_timings_bxcan;
use defmt::{debug, error, info, panic, warn};
use embassy_executor::Spawner;
use embassy_futures::yield_now;
use embassy_stm32::can::Can;
use embassy_stm32::dma::NoDma;
use embassy_stm32::gpio::{Input, Pull};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::peripherals::*;
use embassy_stm32::spi::{Config, Spi};
use embassy_stm32::time::mhz;

use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, RawMutex, ThreadModeRawMutex};
use embassy_sync::channel::{Channel, Receiver, Sender};
use embassy_time::{Duration, Timer};
use embedded_alloc::Heap;

// use embedded_hal::can::Frame;
// use embedded_hal::can::Frame;
use embedded_sdmmc::SdMmcSpi;
use heapless::{Deque, String, Vec};
use static_cell::StaticCell;
// use embedded_sdmmc::TimeSource;
// use kangoo_battery::*;
use {defmt_rtt as _, panic_probe as _};

#[global_allocator]
static HEAP: Heap = Heap::empty();

// static RX_FRAME: BmsChannelMutexRx =
//     embassy_sync::mutex::Mutex::new(Deque::<heapless::Vec<u8, 512>, 1>::new());
// pub type BmsChannelMutexRx =
//     embassy_sync::mutex::Mutex<ThreadModeRawMutex, Deque<heapless::Vec<u8, 512>, 1>>;

static CHANNEL: StaticCell<Channel<ThreadModeRawMutex, heapless::Vec<u8, 512>, 3>> =
    StaticCell::new();

#[embassy_executor::task]
async fn run_sdcard(
    mut spi_dev: SdMmcSpi<Spi<'static, SPI3, NoDma, NoDma>, Output<'static, PE0>>,
    switch1: PE4,
    led: PA6,
    file_name: String<12>,
    receiver: Receiver<'static, ThreadModeRawMutex, heapless::Vec<u8, 512>, 3>,
) {
    let mut led = Output::new(led, Level::High, Speed::VeryHigh);
    let switch1 = Input::new(switch1, Pull::Up);
    match spi_dev.acquire().await {
        Ok(block) => {
            use embedded_sdmmc::{Controller, Mode, VolumeIdx};

            let mut sd_controller: embedded_sdmmc::Controller<
                embedded_sdmmc::BlockSpi<
                    embassy_stm32::spi::Spi<embassy_stm32::peripherals::SPI3, NoDma, NoDma>,
                    Output<embassy_stm32::peripherals::PE0>,
                >,
                Clock,
                4,
                4,
            > = Controller::new(block, Clock);
            info!("OK! Card size...");
            match sd_controller.device().card_size_bytes().await {
                Ok(size) => info!("{}", size),
                Err(e) => warn!("Err: {:?}", e),
            }
            info!("Volume 0...");
            match sd_controller.get_volume(embedded_sdmmc::VolumeIdx(0)).await {
                Ok(v) => info!("{:?}", v),
                Err(e) => info!("Err: {:?}", e),
            }

            let mut volume = match sd_controller.get_volume(VolumeIdx(0)).await {
                Ok(volume) => volume,
                Err(e) => {
                    defmt::panic!("Error getting volume: {:?}", e);
                }
            };

            let dir = match sd_controller.open_root_dir(&volume) {
                Ok(d) => d,
                Err(_) => panic!("Open filesystem failed"),
            };
            let mut file = match sd_controller
                .open_file_in_dir(&mut volume, &dir, &file_name, Mode::ReadWriteCreateOrAppend)
                .await
            {
                Ok(f) => f,
                Err(_) => panic!("Open file failed"),
            };

            info!("Entering file write loop. Press K0 to close file");

            loop {
                // yield_now().await;
                if switch1.is_low() {
                    info!("Ending capture");
                    break;
                }

                let buffer = receiver.recv().await;
                led.set_high();
                if sd_controller
                    .write(&mut volume, &mut file, &buffer)
                    .await
                    .is_err()
                {
                    error!("File write failed")
                };

                led.set_low();
                // info!("Wrote {} bytes", buffer.len());
            }
            if sd_controller.close_file(&volume, file).is_err() {
                error!("Close file failed")
            };
            info!("Closed file");
            sd_controller.close_dir(&volume, dir);
            info!("Closed dir");
            loop {
                yield_now().await;
            }
        }
        Err(_) => defmt::panic!("sd card did not init"),
    };
}

#[embassy_executor::task]
async fn run_can(
    mut can: Can<'static, CAN1>,
    led: PA7,
    switch: PE3,
    sender: Sender<'static, ThreadModeRawMutex, heapless::Vec<u8, 512>, 3>,
) {
    use core::fmt::Write;

    let switch = Input::new(switch, Pull::Up);
    let mut led = Output::new(led, Level::High, Speed::VeryHigh);
    can.modify_filters()
        .enable_bank(0, Fifo::Fifo0, Mask32::accept_all());

    can.modify_config()
        .set_bit_timing(0x001a0005) // http://www.bittiming.can-wiki.info/  500k @ 42MHz
        .set_loopback(false) // Receive own frames
        .set_silent(false)
        .enable();

    let mut buffer: Vec<u8, 512> = heapless::Vec::new();
    let mut go = false;
    let timestamp = embassy_time::Instant::now();
    let mut st: String<90> = String::new();
    let mut enable = false;

    warn!("Press K1 to start can");
    loop {
        // don't forget awaits
        // yield_now().await;

        if switch.is_low() && enable {
            enable = false;
            warn!("Can disabled");
            delay_ms(200).await;
            continue;
        }
        if switch.is_low() && !enable {
            enable = true;
            warn!("Can enabled");
            delay_ms(200).await;
            continue;
        }
        if !enable {
            continue;
        }

        if !go {
            go = true;

            info!("Go")
        }
        'inner: {
            // yield_now().await;
            led.set_high();

            // let frame = match nb::block!(can.receive()) {
            //     Ok(f) => f,
            //     Err(_) => break 'inner,
            // };

            let frame = match nb::block!(can.receive()) {
                Ok(f) => f,
                Err(_) => break 'inner,
            };
            led.set_low();
            if frame.dlc() == 0 || frame.dlc() > 8 {
                break 'inner;
            }
            let id = match frame.id() {
                bxcan::Id::Standard(id) => id.as_raw() as u32,
                bxcan::Id::Extended(id) => id.as_raw(),
            };
            if id == 0 {
                break 'inner;
            }
            let data = match frame.data() {
                Some(d) => d.deref(),
                None => break 'inner,
            };
            if let Err(e) = writeln!(
                &mut st,
                "{},{:02x},{:?}",
                timestamp.elapsed().as_millis(),
                id,
                data
            ) {
                warn!("String error {}", defmt::Debug2Format(&e));
            };
            if buffer
                .capacity()
                .saturating_sub(buffer.len())
                .saturating_sub(st.len())
                == 0
            {
                // let mut rx = RX_FRAME.lock().await;
                if let Ok(output) = Vec::from_slice(&buffer) {
                    // if rx.push_back(output).is_err() {
                    //     error!("Push error: queue len {}", rx.len())
                    // }
                    sender.send(output).await;
                    // info!("Sent {} bytes", buffer.len())
                }
                // led2.set_high();
                buffer.clear();
            }
            buffer.extend_from_slice(st.as_bytes()).unwrap();
            // info!("{} buf.len {}", defmt::Debug2Format(&st), buffer.len());

            // led.set_low();
            st.clear();
            break 'inner;
        }
    }
}

async fn delay_ms(ms: u64) {
    Timer::after(Duration::from_millis(ms)).await
}
// #[entry]
#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let mut config = embassy_stm32::Config::default();
    config.rcc.sys_ck = Some(mhz(168));
    let mut p = embassy_stm32::init(config);
    // let mut led1 = Output::new(p.PA6, Level::High, Speed::Low);
    // let mut led2 = Output::new(p.PA7, Level::High, Speed::Low);
    // let mut swtich1 = Input::new(p.PE4, Pull::Up);
    // let mut swtich2 = Input::new(p.PE3, Pull::Up);
    debug!("Can test");

    let mut cs = Output::new(p.PE0, Level::High, Speed::VeryHigh);

    let mut spi = embassy_stm32::spi::Spi::new(
        p.SPI3,
        p.PC10,
        p.PC12,
        p.PC11,
        NoDma,
        NoDma,
        embassy_stm32::time::Hertz(12_000_000),
        Config::default(),
    );
    let mut spi_dev = embedded_sdmmc::SdMmcSpi::new(spi, cs);

    let rx_pin = Input::new(&mut p.PD0, Pull::Up);
    core::mem::forget(rx_pin);

    let can = Can::new(p.CAN1, p.PD0, p.PD1);
    let channel = CHANNEL.init(Channel::new());
    info!("Starting Can");
    defmt::unwrap!(spawner.spawn(run_can(can, p.PA7, p.PE3, channel.sender())));

    info!("Starting SDCard");
    defmt::unwrap!(spawner.spawn(run_sdcard(
        spi_dev,
        p.PE4,
        p.PA6,
        "CAN.TXT".into(),
        channel.receiver()
    )));

    loop {
        Timer::after(Duration::from_secs(1)).await
    }
}

struct Clock;

impl embedded_sdmmc::TimeSource for Clock {
    fn get_timestamp(&self) -> embedded_sdmmc::Timestamp {
        embedded_sdmmc::Timestamp {
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}
