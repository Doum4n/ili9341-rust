use core::convert::Infallible;

use display_interface_spi::SPIInterface;
use embedded_graphics::{
    geometry::Point as EgPoint,
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
};
use embedded_hal::{
    digital::{OutputPin, PinState},
    spi::*,
};
use esp_idf_hal::{
    delay::{Delay, FreeRtos},
    gpio::{PinDriver, *},
    ledc::{config::TimerConfig, LedcDriver, LedcTimerDriver},
    peripherals::Peripherals,
    prelude::*,
    spi::{config::Config, *},
};
use esp_idf_svc::log::EspLogger;
use ili9341::{DisplaySize240x320, Ili9341, Orientation};
use log::*;
use std::{
    thread::{self, Builder},
    time::Instant,
};

struct Gpio2Output<'a> {
    pin: PinDriver<'a, Gpio16, Output>,
}

impl embedded_hal::digital::ErrorType for Gpio2Output<'_> {
    type Error = Infallible;
}

impl<'a> Gpio2Output<'a> {
    pub fn new(pin: PinDriver<'a, Gpio16, Output>) -> Self {
        Gpio2Output { pin }
    }
}

impl<'a> OutputPin for Gpio2Output<'a> {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        let _ = self.pin.set_low();
        info!("Reset pin set low");
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        let _ = self.pin.set_high();
        info!("Reset pin set high");
        Ok(())
    }

    fn set_state(&mut self, state: PinState) -> Result<(), Self::Error> {
        match state {
            PinState::High => self.set_high(),
            PinState::Low => self.set_low(),
        }
    }
}

fn main() -> anyhow::Result<()> {
    EspLogger::initialize_default();
    info!("Starting program");

    const HOR_RES: u32 = 320;
    const VER_RES: u32 = 240;

    let peripherals = Peripherals::take().unwrap();
    info!("Peripherals taken");

    let display_spi_pins = (
        peripherals.pins.gpio14, // SCK
        peripherals.pins.gpio13, // MOSI
        peripherals.pins.gpio12, // MISO
    );
    let dc = peripherals.pins.gpio2;
    let cs_display = peripherals.pins.gpio15;

    let spi_config = Config::new().baudrate(10.MHz().into()).data_mode(Mode {
        polarity: Polarity::IdleLow,
        phase: Phase::CaptureOnFirstTransition,
    });
    info!("SPI config created with 10 MHz baudrate");

    let spi_driver = SpiDriver::new(
        peripherals.spi2,
        display_spi_pins.0,
        display_spi_pins.1,
        Some(display_spi_pins.2),
        &Default::default(),
    )?;
    info!("SPI driver initialized");

    // Option 1: PWM backlight
    // let mut channel = LedcDriver::new(
    //     peripherals.ledc.channel0,
    //     LedcTimerDriver::new(
    //         peripherals.ledc.timer0,
    //         &TimerConfig::new().frequency(25.kHz().into()),
    //     )
    //     .unwrap(),
    //     peripherals.pins.gpio18,
    // )?;
    // channel.set_duty(channel.get_max_duty() / 2)?;
    // info!("Backlight initialized with 50% duty");

    // Option 2: Backlight always on
    let mut bl = PinDriver::output(peripherals.pins.gpio18).unwrap();
    bl.set_high().unwrap();
    info!("Backlight pin set high");

    let _main_thread = Builder::new().stack_size(60000).spawn(move || {
        let mut spi_display =
            SpiDeviceDriver::new(&spi_driver, Some(cs_display), &spi_config).unwrap();
        info!("SPI display device created");

        let display_interface = SPIInterface::new(&mut spi_display, PinDriver::output(dc).unwrap());
        let mut delay = Delay::new_default();

        let gpio2_output = PinDriver::output(peripherals.pins.gpio16).unwrap();
        let mut reset = Gpio2Output::new(gpio2_output);

        // Manual reset sequence
        reset.set_low().unwrap();
        delay.delay_ms(200);
        reset.set_high().unwrap();
        delay.delay_ms(200);
        info!("Reset sequence completed");

        let mut lcd = match Ili9341::new(
            display_interface,
            &mut reset,
            &mut delay,
            Orientation::LandscapeFlipped,
            DisplaySize240x320,
        ) {
            Ok(lcd) => {
                info!("Display initialized successfully");
                lcd
            }
            Err(e) => {
                error!("Display initialization failed: {:?}", e);
                return;
            }
        };

        // Clear once with visible color
        lcd.clear(Rgb565::BLUE).unwrap();
        info!("Screen cleared with blue");

        // Button state test
        let mut button_state = false;
        let mut last_toggle = Instant::now();
        let button_area = Rectangle::new(EgPoint::new(30, 80), Size::new(80, 80));

        loop {
            if last_toggle.elapsed().as_secs() >= 1 {
                button_state = !button_state;
                last_toggle = Instant::now();
                info!("Button state toggled to {}", button_state);
            }

            let color = if button_state {
                Rgb565::GREEN
            } else {
                Rgb565::RED
            };

            /*if let Err(e) =*/ let _ = lcd.fill_contiguous(&button_area, std::iter::repeat(color).take(80 * 80)); /*{
                error!("Failed to draw button: {:?}", e);
            } else {
                info!("Button drawn with {}", if button_state { "green" } else { "red" });
            }*/

            FreeRtos::delay_ms(50);
        }
    })?;

    loop {
        FreeRtos::delay_ms(1000);
    }
}
