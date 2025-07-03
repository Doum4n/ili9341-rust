use alloc::rc::Rc;
use embedded_hal::digital::OutputPin;
use core::cell::RefCell;
use esp_idf_hal::{
    delay::{Delay, FreeRtos},
    gpio::{PinDriver, *},
    peripherals::Peripherals,
    prelude::*,
    spi::{SpiDeviceDriver, SpiDriver},
};
use esp_idf_svc::log::EspLogger;
use ili9341::{DisplaySize240x320, Ili9341, Orientation};
use log::*;
use slint::platform::{
    software_renderer::{MinimalSoftwareWindow, Rgb565Pixel},
    Platform, WindowEvent,
};
use std::sync::Arc;
use std::time::{Duration, Instant};
use std::thread::Builder;

extern crate alloc;

mod config;
mod display;
mod touch;
mod platform;
mod utils;

use config::Config;
use display::{DisplayWrapper, Scroller};
use platform::Ili9341Platform;
use utils::{GpioOutput, PartialBuffer};

slint::include_modules!();

static mut BUFFER: [Rgb565Pixel; 320 * 240] = [Rgb565Pixel(0); 320 * 240];

fn create_slint_app() -> AppWindow {
    let ui = AppWindow::new().expect("Failed to load UI");
    let ui_handle = ui.as_weak();
    ui.on_request_increase_value(move || {
        let ui = ui_handle.unwrap();
        ui.set_counter(ui.get_counter() + 1);
    });
    ui
}

fn main() -> anyhow::Result<()> {
    EspLogger::initialize_default();
    info!("Starting program");

    let config = Config::new();
    let peripherals = Peripherals::take().unwrap();
    info!("Peripherals taken");

    let display_spi_pins = (
        peripherals.pins.gpio14,
        peripherals.pins.gpio13,
        peripherals.pins.gpio12,
    );
    let dc = PinDriver::output(peripherals.pins.gpio2)?;
    let cs_display = PinDriver::output(peripherals.pins.gpio15)?;
    let (dc_copy, cs_copy) = unsafe { (core::ptr::read(&dc as *const _), core::ptr::read(&cs_display as *const _)) };

    let spi_config = esp_idf_hal::spi::config::Config::new().baudrate(config.spi_baudrate).data_mode(embedded_hal::spi::Mode {
        polarity: embedded_hal::spi::Polarity::IdleLow,
        phase: embedded_hal::spi::Phase::CaptureOnFirstTransition,
    });
    info!("SPI config created with {} Hz baudrate", config.spi_baudrate.0);

    let spi_driver = SpiDriver::new(
        peripherals.spi2,
        display_spi_pins.0,
        display_spi_pins.1,
        Some(display_spi_pins.2),
        &Default::default(),
    )?;
    info!("SPI driver initialized");

    let mut bl = PinDriver::output(peripherals.pins.gpio18)?;
    bl.set_high()?;
    info!("Backlight pin set high");

    let _main_thread = Builder::new().stack_size(60_000).spawn(move || {
        let spi_display_driver = Arc::new(spi_driver);
        let spi_display = RefCell::new(SpiDeviceDriver::new(Arc::clone(&spi_display_driver), None::<Gpio5>, &spi_config).unwrap());
        let clone_spi_display = RefCell::new(SpiDeviceDriver::new(Arc::clone(&spi_display_driver), None::<Gpio5>, &spi_config).unwrap());
        let touch_irq = PinDriver::input(peripherals.pins.gpio4).unwrap();
        let cs_touch = peripherals.pins.gpio5;

        let spi_touch = SpiDeviceDriver::new(Arc::clone(&spi_display_driver), None::<Gpio5>, &spi_config).unwrap();
        info!("SPI touch device created");

        let display_interface = display_interface_spi::SPIInterface::new(spi_display.into_inner(), dc);
        let mut delay = Delay::new_default();
        let gpio2_output = PinDriver::output(peripherals.pins.gpio16).unwrap();
        let mut reset = GpioOutput::new(gpio2_output);

        reset.set_low().unwrap();
        delay.delay_ms(200);
        reset.set_high().unwrap();
        delay.delay_ms(200);
        info!("Reset sequence completed");

        let lcd = Ili9341::new(
            display_interface,
            reset,
            &mut delay,
            Orientation::Landscape,
            DisplaySize240x320,
        )
        .unwrap();

        let touch = touch::XPT2046::new(touch_irq, GpioOutput::new(PinDriver::output(cs_touch).unwrap()), spi_touch).unwrap();
        let buffer_provider = DisplayWrapper::new(lcd, unsafe { &mut BUFFER }, clone_spi_display.into_inner(), dc_copy, cs_copy);

        let platform = Ili9341Platform::new(buffer_provider, touch);
        slint::platform::set_platform(Box::new(platform)).unwrap();
        slint::run_event_loop().unwrap();
    })?;

    loop {
        FreeRtos::delay_ms(1000);
    }
}
