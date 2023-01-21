#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(alloc_error_handler)]
#![feature(default_alloc_error_handler)]
// extern crate alloc;

use core::ops::Deref;

// use alloc::format;
use bxcan::filter::Mask32;
use bxcan::{Fifo, Frame, StandardId};
use can_bit_timings::can_timings_bxcan;
// use cortex_m_rt::entry;
use defmt::{debug, error, info, panic, warn};
use embassy_executor::Spawner;
// use embassy_stm32::can::bxcan::filter::Mask32;
// use embassy_stm32::can::bxcan::{Fifo, Frame};
use embassy_stm32::can::Can;
use embassy_stm32::dma::NoDma;
use embassy_stm32::gpio::{self, Input, Pull};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::spi::Config;
// use embassy_stm32::spi::;
use embassy_stm32::time::mhz;

use embedded_alloc::Heap;

// use embedded_hal::can::Frame;
// use embedded_hal::can::Frame;
use embedded_sdmmc::{Controller, VolumeIdx};
use heapless::{String, Vec};
// use embedded_sdmmc::TimeSource;
use kangoo_battery::*;
use {defmt_rtt as _, panic_probe as _};

// mod file_reader;

// mod file_reader;

#[global_allocator]
static HEAP: Heap = Heap::empty();

// #[entry]
#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let mut config = embassy_stm32::Config::default();
    config.rcc.sys_ck = Some(mhz(168));
    let mut p = embassy_stm32::init(config);
    let mut led1 = Output::new(p.PA6, Level::High, Speed::Low);
    let mut led2 = Output::new(p.PA7, Level::High, Speed::Low);
    let mut swtich1 = Input::new(p.PE4, Pull::Up);
    let mut swtich2 = Input::new(p.PE3, Pull::Up);
    debug!("Can test");

    // The next two lines are a workaround for testing without transceiver.
    // To synchronise to the bus the RX input needs to see a high level.
    // Use `mem::forget()` to release the borrow on the pin but keep the
    // pull-up resistor enabled.
    let rx_pin = Input::new(&mut p.PD0, Pull::Up);
    core::mem::forget(rx_pin);

    let mut can = Can::new(p.CAN1, p.PD0, p.PD1);

    can.modify_filters()
        .enable_bank(0, Fifo::Fifo0, Mask32::accept_all());

    let bittimings = can_timings_bxcan!(42.mhz(), 500.khz());
    debug!("500k {}", bittimings);
    debug!("500k 168mhz/42mhz const {}", 0x001a0005); // macro state 0x300003 ???

    can.modify_config()
        .set_bit_timing(0x001a0005) // http://www.bittiming.can-wiki.info/  250k
        // .set_bit_timing(bittimings) // crate -> can-bit-timings
        .set_loopback(false) // Receive own frames
        .set_silent(false)
        .enable();

    /* SD card */

    // let irq = interrupt::take!(SDIO);
    // let clk = p.PC12;
    // let cmd = p.PD2;
    // let d0 = p.PC8;
    // let cs = p.PC11;

    // let mut sdmmc = Sdmmc::new_4bit(
    //     p.SDIO,
    //     irq,
    //     // NoDma,
    //     p.DMA2_CH3,
    //     p.PC12,
    //     p.PD2,
    //     p.PC8,
    //     p.PC9,
    //     p.PC10,
    //     p.PC11,
    //     Default::default(),
    // );

    let mut cs = Output::new(p.PE0, Level::High, Speed::VeryHigh);

    let mut spi = embassy_stm32::spi::Spi::new(
        p.SPI3,
        p.PC10,
        p.PC12,
        p.PC11,
        NoDma,
        NoDma,
        embassy_stm32::time::Hertz(10_000_000),
        Config::default(),
    );

    // use embassy_stm32::sdmmc::
    // Should print 400kHz for initialization
    // info!("Configured clock: {}", sdmmc.clock().0);
    // // info!("Configured clock: {}", sdmmc. );
    // let sd = unwrap!(sdmmc.init_card(embassy_stm32::time::mhz(25)).await);

    let mut spi_dev = embedded_sdmmc::SdMmcSpi::new(spi, cs);

    match spi_dev.acquire().await {
        Ok(block) => {
            use embassy_stm32::spi::Spi; // spi::SpiBus;
            use embedded_hal::digital::v2::OutputPin;
            use embedded_hal_async::spi::{SpiBus, SpiDevice};
            use embedded_sdmmc::{
                BlockSpi, Controller, File, Mode, TimeSource, Timestamp, Volume, VolumeIdx,
            };

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

            let mut file_buffer: [u8; 512];
            let mut file: Option<File> = None;
            let mut volume: Option<Volume> = None;
            let write_index: usize = 0;
            let file_name = "can.txt";

            let mut volume = match sd_controller.get_volume(VolumeIdx(0)).await {
                Ok(volume) => volume,
                Err(e) => {
                    defmt::panic!("Error getting volume: {:?}", e);
                }
            };

            let dir = sd_controller.open_root_dir(&volume).unwrap();
            let file = sd_controller
                .open_file_in_dir(&mut volume, &dir, file_name, Mode::ReadWriteCreateOrAppend)
                .await
                .unwrap();

            // fill the file_buffer
            // sd_controller
            //     .read(&volume, &mut file, &mut file_buffer)
            //     .await
            //     .unwrap();

            let mut file = file;
            let mut volume = volume;
            let mut buffer: Vec<u8, 512> = heapless::Vec::new();
            let mut go = false;
            let mut timestamp = embassy_time::Instant::now();
            loop {
                if swtich1.is_low() {
                    info!("Ending capture");
                    break;
                }
                if !swtich2.is_low() && !go {
                    continue;
                }
                if !go {
                    go = true;
                    timestamp = embassy_time::Instant::now();
                    info!("Go")
                }
                use core::fmt::Write;
                'inner: {
                    led1.set_high();

                    let mut st: String<90> = String::new();
                    if let Ok(frame) = nb::block!(can.receive()) {
                        led1.set_low();
                        let id = match frame.id() {
                            bxcan::Id::Standard(id) => id.as_raw() as u32,
                            bxcan::Id::Extended(id) => id.as_raw(),
                        };
                        if let Err(e) = writeln!(
                            &mut st,
                            "{},{:02x},{:?}",
                            timestamp.elapsed().as_millis(),
                            id,
                            frame.data().unwrap().deref()
                        ) {
                            warn!("String error {}", defmt::Debug2Format(&e));
                        };
                        if buffer
                            .capacity()
                            .saturating_sub(buffer.len())
                            .saturating_sub(st.len())
                            == 0
                        {
                            led2.set_high();
                            sd_controller
                                .write(&mut volume, &mut file, &buffer)
                                .await
                                .unwrap();
                            buffer.clear();
                            led2.set_low();
                        }
                        buffer.extend_from_slice(st.as_bytes()).unwrap();

                        info!("{} buf.len {}", defmt::Debug2Format(&st), buffer.len());
                        break 'inner;
                    } else {
                        led1.set_low();
                    };
                }
            }
            led1.set_high();
            led2.set_high();
            sd_controller.close_file(&volume, file).unwrap();
            info!("Closed file");
            sd_controller.close_dir(&volume, dir);
            info!("Closed dir");
            loop {}
            // let file = file_reader::FileReader::new(cont, "can.txt");
        }
        Err(_) => defmt::panic!("sd card did not init"),
    };

    // let controller = embedded_sdmmc::Controller::new(c, Clock);
    // let card = unwrap!(sdmmc.card());
    // let mut controller: embedded_sdmmc::Controller<
    //     Sdmmc<embassy_stm32::peripherals::SDIO>,
    //     Clock,
    //     4,
    //     4,
    // > = embedded_sdmmc::Controller::new(, Clock);

    // info!("Card: {:#?}", Debug2Format(card));

    /*
    INFO  Card: "Card { card_type: SDHC, ocr: OCR: Operation Conditions Register { Voltage Window (mV): (2700, 3600), S18A (UHS-I only): false, Over 2TB flag (SDUC only): false, UHS-II Card: false, Card Capacity Status (CSS): \"SDHC/SDXC/SDUC\", Busy: false }, rca: 1, cid: CID: Card Identification { Manufacturer ID: 27, OEM ID: \"SM\", Product Name: \"00000\", Product Revision: 16, Product Serial Number: 1894894521, Manufacturing Date: (7, 2013) }, csd: CSD: Card Specific Data { Transfer Rate: 50, Block Count: 30702592, Card Size (bytes): 15719727104, Read I (@min VDD): 100 mA, Write I (@min VDD): 100 mA, Read I (@max VDD): 200 mA, Write I (@max VDD): 200 mA, Erase Size (Blocks): 1 }, scr: SCR: SD CARD Configuration Register { Version: V3, 1-bit width: true, 4-bit width: true }, status: SD Status { Bus Width: One, Secured Mode: false, SD Memory Card Type: 0, Protected Area Size (B): 0, Speed Class: 0, Video Speed Class: 0, Application Performance Class: 0, Move Performance (MB/s): 0, AU Size: 0, Erase Size (units of AU): 0, Erase Timeout (s): 0, Discard Support: false } }"
    */

    let mut bms_validated = Bms::new([CellValue::new(0u16, false); 96]);

    let mut data = Data::default();
    let mut update_solax = false;

    info!("Entering loop");
    // let mut i: u32 = 0;
    let frame = embassy_stm32::can::bxcan::Frame::new_data(
        embassy_stm32::can::bxcan::Id::Standard(
            embassy_stm32::can::bxcan::StandardId::new(0x99).unwrap(),
        ),
        [0, 0, 0, 0, 0, 0, 0, 0],
    );

    let mut stopwatch = embassy_time::Instant::now();
    let sec = embassy_time::Duration::from_secs(5);
    let mut counter = 0u32;
    loop {
        while stopwatch.elapsed() < sec {
            led1.set_high();
            if let Err(e) = nb::block!(can.receive()) {
                led1.set_low();
                warn!("BMS frame rx error {:?}", e)
            } else {
                counter += 1;
                led1.set_low();
            };
        }
        counter /= 5;
        info!("{}fps", counter);
        counter = 0;
        stopwatch = embassy_time::Instant::now();
        continue;
        if true {
            led1.set_high();
            let frame: Frame = match nb::block!(can.receive()) {
                Ok(f) => f,
                Err(e) => {
                    debug!("{}", e);
                    continue;
                }
            };
            led1.set_low();
            // info!("loopback frame {=u8}", unwrap!(rx_frame.data())[0]);
            // if i == u32::MAX {
            //     break;
            // }
            // i += 1;
            // led.set_high();

            let id = if let bxcan::Id::Standard(id) = frame.id() {
                id.as_raw()
            } else {
                continue;
            };
            if id != 0x7bb {
                update_solax = match data.rapid_data_processor(frame) {
                    Ok(true) => true, // data can be parsed without diag data is true - use a different validity checker
                    Ok(false) => false,
                    Err(e) => {
                        info!("Rapid data parsing error: {:?}", e);
                        continue;
                    }
                }
                //else process diag data
            } else {
                update_solax = match data.diag_data_processor(frame) {
                    Ok(None) => match bms_validated.update_bms_data(&data) {
                        Ok(_) => true,
                        Err(e) => {
                            warn!("BMS data update error: {:?}", e);
                            false
                        }
                    },
                    Ok(Some(next_tx_frame)) => {
                        // let mut tx = tx.lock().await;
                        if let Err(e) = nb::block!(can.transmit(&next_tx_frame)) {
                            // if let Err(e) = tx.push_front(next_tx_frame) {
                            warn!("BMS frame push TX error {:?}", e)
                        };
                        continue; // break
                    }
                    Err(e) => {
                        warn!("BMS diag error: {:?}", e);
                        false
                    }
                };
            }
            if update_solax {
                info!("BMS: soc {}", bms_validated.soc);
            }
        }
        // info!("{}fps", i);
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
