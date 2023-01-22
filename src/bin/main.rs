#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(alloc_error_handler)]
#![feature(default_alloc_error_handler)]
// extern crate alloc;

use core::ops::Deref;

use bxcan::filter::Mask32;
use bxcan::Fifo;
use defmt::{debug, error, info, panic, unwrap, warn};
use embassy_executor::Spawner;
use embassy_futures::yield_now;
use embassy_stm32::can::Can;
use embassy_stm32::dma::NoDma;
use embassy_stm32::gpio::{Input, Pull};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::sdmmc::Sdmmc;
use embassy_stm32::spi::{Config, Spi};
use embassy_stm32::time::{mhz, Hertz};
use embassy_stm32::{interrupt, peripherals::*};

use embassy_sync::blocking_mutex::raw::{
    CriticalSectionRawMutex, NoopRawMutex, ThreadModeRawMutex,
};
use embassy_sync::channel::{Channel, Receiver, Sender};
use embassy_time::{Duration, Timer};
use embedded_alloc::Heap;
use embedded_sdmmc::{Controller, Mode, VolumeIdx};
// use embedded_sdmmc_async::{
//     sdmmc, sdmmc_proto, BlockDevice, BlockSpi, Controller, Mode, SdMmcSpi, TimeSource, Timestamp,
//     VolumeIdx,
// };
use heapless::{Deque, String, Vec};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

#[global_allocator]
static HEAP: Heap = Heap::empty();

// static RX_FRAME: BmsChannelMutexRx =
//     embassy_sync::mutex::Mutex::new(Deque::<heapless::Vec<u8, 512>, 1>::new());
// pub type BmsChannelMutexRx =
//     embassy_sync::mutex::Mutex<ThreadModeRawMutex, Deque<heapless::Vec<u8, 512>, 1>>;
const CHANNELDEPTH: usize = 1;
const CHANNELBLOCKSIZE: usize = 1024;
const BUFFERSIZE: usize = CHANNELBLOCKSIZE + 256;

static CHANNEL: StaticCell<
    Channel<NoopRawMutex, heapless::Vec<u8, CHANNELBLOCKSIZE>, CHANNELDEPTH>,
> = StaticCell::new();

#[embassy_executor::task]
async fn run_sdcard(
    sdmmc: Sdmmc<'static, SDIO, DMA2_CH3>,
    switch1: PE4,
    led: PA6,
    file_name: String<12>,
    receiver: Receiver<'static, NoopRawMutex, heapless::Vec<u8, CHANNELBLOCKSIZE>, CHANNELDEPTH>,
) {
    let mut led = Output::new(led, Level::High, Speed::VeryHigh);
    let switch1 = Input::new(switch1, Pull::Up);

    let mut sd_controller = embedded_sdmmc::Controller::new(sdmmc, Clock2);

    led.set_high();
    let mut volume = match sd_controller.get_volume(VolumeIdx(0)).await {
        Ok(volume) => volume,
        Err(e) => {
            defmt::panic!("Error getting volume: {}", defmt::Debug2Format(&e));
        }
    };
    led.set_low();
    let dir = match sd_controller.open_root_dir(&volume) {
        Ok(d) => d,
        Err(e) => panic!("Open filesystem failed: {}", defmt::Debug2Format(&e)),
    };
    let mut file = match sd_controller
        .open_file_in_dir(&mut volume, &dir, &file_name, Mode::ReadWriteCreateOrAppend)
        .await
    {
        Ok(f) => f,
        Err(_) => panic!("Open file failed"),
    };
    led.set_low(); // set low enables led, high is off
    info!("Entering file write loop. Press K0 to close file");

    loop {
        yield_now().await;
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
    }
    if sd_controller.close_file(&volume, file).is_err() {
        error!("Close file failed")
    };
    info!("Closed file");
    sd_controller.close_dir(&volume, dir);
    info!("Closed dir");
    loop {
        delay_ms(2).await;
    }
}

#[embassy_executor::task]
async fn run_can(
    mut can: Can<'static, CAN1>,
    led: PA7,
    switch: PE3,
    sender: Sender<'static, NoopRawMutex, heapless::Vec<u8, CHANNELBLOCKSIZE>, CHANNELDEPTH>,
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

    let mut buffer: Vec<u8, BUFFERSIZE> = heapless::Vec::new();
    let mut go = false;
    let timestamp = embassy_time::Instant::now();
    let mut st: String<90> = String::new();
    let mut enable = false;

    warn!("Press K1 to start can");
    loop {
        // don't forget awaits

        led.set_low();
        yield_now().await;

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
            let frame = match nb::block!(can.receive()) {
                Ok(f) => f,
                Err(_) => break 'inner,
            };

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

            led.set_high();
            if let Err(e) = writeln!(
                &mut st,
                "{},{:02x},{:?}",
                timestamp.elapsed().as_millis(),
                id,
                data
            ) {
                warn!("String error {}", defmt::Debug2Format(&e));
            };
            if buffer.len() > CHANNELBLOCKSIZE {
                // info!("B: Buf cap {}, buf len {}", buffer.capacity(), buffer.len());
                let pop: Vec<u8, CHANNELBLOCKSIZE> =
                    Vec::from_slice(&buffer[0..CHANNELBLOCKSIZE]).unwrap();
                let len = buffer.capacity().saturating_sub(buffer.len());
                buffer.truncate(len);

                // info!(
                //     "A; Buf cap {}, buf len {}, pop len {}",
                //     buffer.capacity(),
                //     buffer.len(),
                //     pop.len()
                // );

                sender.send(pop).await;
            }
            buffer.extend_from_slice(st.as_bytes()).unwrap();
            st.clear();
            break 'inner;
        }
    }
}

async fn delay_ms(ms: u64) {
    Timer::after(Duration::from_millis(ms)).await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let mut config = embassy_stm32::Config::default();
    config.rcc.sys_ck = Some(mhz(168));
    let mut p = embassy_stm32::init(config);
    debug!("Can test");

    let irq = interrupt::take!(SDIO);

    // let cs = Output::new(p.PE0, Level::High, Speed::VeryHigh);
    let mut sdmmc = Sdmmc::new_4bit(
        p.SDIO,
        irq,
        p.DMA2_CH3,
        p.PC12,
        p.PD2,
        p.PC8,
        p.PC9,
        p.PC10,
        p.PC11,
        Default::default(),
    );
    loop {
        match sdmmc.init_card(mhz(1)).await {
            Ok(_) => break,
            Err(_) => delay_ms(1).await,
        }
    }
    // unwrap!(sdmmc.init_card(mhz(10)).await);

    let rx_pin = Input::new(&mut p.PD0, Pull::Up);
    core::mem::forget(rx_pin);

    let can = Can::new(p.CAN1, p.PD0, p.PD1);
    let channel = CHANNEL.init(Channel::new());
    info!("Starting Can");
    defmt::unwrap!(spawner.spawn(run_can(can, p.PA7, p.PE3, channel.sender())));

    info!("Starting SDCard");
    defmt::unwrap!(spawner.spawn(run_sdcard(
        sdmmc,
        p.PE4,
        p.PA6,
        "CAN.TXT".into(),
        channel.receiver()
    )));

    loop {
        Timer::after(Duration::from_secs(1)).await
    }
}

// struct Clock;

// impl TimeSource for Clock {
//     fn get_timestamp(&self) -> Timestamp {
//         Timestamp {
//             year_since_1970: 0,
//             zero_indexed_month: 0,
//             zero_indexed_day: 0,
//             hours: 0,
//             minutes: 0,
//             seconds: 0,
//         }
//     }
// }

struct Clock2;

impl embedded_sdmmc::TimeSource for Clock2 {
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
