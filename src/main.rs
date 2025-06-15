use core::convert::Infallible;
use display_interface_spi::SPIInterface;
use embedded_hal::{digital::OutputPin, spi::{Error, Mode, Phase, Polarity}};
use esp_idf_hal::{
    delay::{Delay, FreeRtos},
    gpio::{PinDriver, *},
    peripherals::Peripherals,
    prelude::*,
    spi::{config::Config, *},
};
use esp_idf_svc::log::EspLogger;
use ili9341::{DisplaySize240x320, Ili9341, Orientation};
use log::*;
use slint::platform::{software_renderer::{MinimalSoftwareWindow, Rgb565Pixel}, Platform, WindowEvent};
use std::{cell::{RefCell, UnsafeCell}, rc::Rc, thread::{self, Builder}, time::{Duration, Instant}};
use embedded_graphics::{geometry::Point as EgPoint, pixelcolor::Rgb565, prelude::{RgbColor, Size}, primitives::Rectangle};
use embedded_graphics::draw_target::DrawTarget;

extern crate alloc;


pub struct Gpio2Output<'a, T>
where
    T: Pin,
{
    pin: PinDriver<'a, T, Output>,
}

impl<'a, T> embedded_hal::digital::ErrorType for Gpio2Output<'a, T>
where
    T: Pin,
{
    type Error = GpioError;
}

impl<'a, T> Gpio2Output<'a, T> where T: Pin{
    pub fn new(pin: PinDriver<'a, T, Output>) -> Self {
        Gpio2Output { pin }
    }
}

impl<'a, T> OutputPin for Gpio2Output<'a, T>
where
    T: Pin,
{
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.pin.set_low().unwrap(); // hoặc dùng `?` nếu bạn muốn propagate lỗi
        info!("Reset pin set low");
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.pin.set_high().unwrap();
        info!("Reset pin set high");
        Ok(())
    }

    fn set_state(&mut self, state: embedded_hal::digital::PinState) -> Result<(), Self::Error> {
        match state {
            embedded_hal::digital::PinState::High => self.set_high(),
            embedded_hal::digital::PinState::Low => self.set_low(),
        }
    }
}


slint::include_modules!();

fn create_slint_app() -> AppWindow {
    let ui = AppWindow::new().expect("Failed to load UI");

    let ui_handle = ui.as_weak();
    ui.on_request_increase_value(move || {
        let ui = ui_handle.unwrap();
        ui.set_counter(ui.get_counter() + 1);
    });
    ui
}

// Backend tùy chỉnh cho Slint để tích hợp với ILI9341
struct Ili9341Platform {
    window: alloc::rc::Rc<slint::platform::software_renderer::MinimalSoftwareWindow>,
    start_time: Instant,
}

impl Platform for Ili9341Platform {
    fn create_window_adapter(&self) -> Result<alloc::rc::Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError> {
        Ok(self.window.clone() as alloc::rc::Rc<dyn slint::platform::WindowAdapter>)
    }

    fn duration_since_start(&self) -> Duration {
        Instant::now().duration_since(self.start_time)
    }
}

// Bộ đệm tĩnh
static mut BUFFER: UnsafeCell<[Rgb565Pixel; (320 * 240) as usize]> = UnsafeCell::new([Rgb565Pixel(0); (320 * 240) as usize]);

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

    let spi_config = Config::new()
        .baudrate(10.MHz().into())
        .data_mode(Mode {
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

    let mut bl = PinDriver::output(peripherals.pins.gpio18).unwrap();
    bl.set_high().unwrap();
    info!("Backlight pin set high");

    let _main_thread = Builder::new().stack_size(60000).spawn(move || {
        let spi_display = SpiDeviceDriver::new(&spi_driver, Some(cs_display), &spi_config).unwrap();
        info!("SPI display device created");

        let mut touch_irq = PinDriver::input(peripherals.pins.gpio4).unwrap();
        // let _ = touch_irq.set_pull(Pull::Up);
        // touch_irq.set_interrupt_type(InterruptType::NegEdge).unwrap();
        // touch_irq.enable_interrupt().unwrap();
        let cs_touch = peripherals.pins.gpio5; // Assuming you have another CS pin for the touch controller

        let spi_touch = SpiDeviceDriver::new(&spi_driver, None::<Gpio5>, &spi_config).unwrap();
        info!("SPI touch device created");

        let display_interface = SPIInterface::new(spi_display, PinDriver::output(dc).unwrap());
        let mut delay = Delay::new_default();

        let gpio2_output = PinDriver::output(peripherals.pins.gpio16).unwrap();
        let mut reset = Gpio2Output::new(gpio2_output);

        // Manual reset sequence
        reset.set_low().unwrap();
        delay.delay_ms(200);
        reset.set_high().unwrap();
        delay.delay_ms(200);
        info!("Reset sequence completed");

        let mut lcd = Ili9341::new(
            display_interface,
            reset,
            &mut delay,
            Orientation::LandscapeFlipped,
            DisplaySize240x320,
        ).unwrap();

        let mut touch = xpt2046::XPT2046::new(touch_irq, Gpio2Output::new(PinDriver::output(cs_touch).unwrap()), spi_touch).unwrap();
        // Khởi tạo cửa sổ Slint
        let window = slint::platform::software_renderer::MinimalSoftwareWindow::new(Default::default());
        window.set_size(slint::PhysicalSize::new(320, 240));
        let platform = Ili9341Platform {
            window: window.clone(),
            start_time: Instant::now(),
        };

        slint::platform::set_platform(Box::new(platform)).unwrap();
        let _ui = create_slint_app();

        // Lấy bộ đệm tĩnh
        let buffer = &mut [slint::platform::software_renderer::Rgb565Pixel(0); 320];
        let mut last_touch = None;

        loop {
            slint::platform::update_timers_and_animations();

            // Vẽ giao diện lên bộ đệm
            window.draw_if_needed(|renderer| {
                use embedded_graphics_core::prelude::*;
                struct DisplayWrapper<'a, T>(
                    &'a mut T,
                    &'a mut [slint::platform::software_renderer::Rgb565Pixel],
                );
                impl<T: DrawTarget<Color = embedded_graphics_core::pixelcolor::Rgb565>>
                    slint::platform::software_renderer::LineBufferProvider for DisplayWrapper<'_, T>
                {
                    type TargetPixel = slint::platform::software_renderer::Rgb565Pixel;
                    fn process_line(
                        &mut self,
                        line: usize,
                        range: core::ops::Range<usize>,
                        render_fn: impl FnOnce(&mut [Self::TargetPixel]),
                    ) {
                        let rect = embedded_graphics_core::primitives::Rectangle::new(
                            Point::new(range.start as _, line as _),
                            Size::new(range.len() as _, 1),
                        );
                        render_fn(&mut self.1[range.clone()]);
                        // NOTE! this is not an efficient way to send pixel to the screen, but it is kept simple on this template.
                        // It would be much faster to use the DMA to send pixel in parallel.
                        // See the example in https://github.com/slint-ui/slint/blob/master/examples/mcu-board-support/pico_st7789.rs 
                        self.0
                            .fill_contiguous(
                                &rect,
                                self.1[range.clone()].iter().map(|p| {
                                    embedded_graphics_core::pixelcolor::raw::RawU16::new(p.0).into()
                                }),
                            )
                            .map_err(drop)
                            .unwrap();
                    }
                }
                renderer.render_by_line(DisplayWrapper(&mut lcd, buffer));
            });

            FreeRtos::delay_ms(50);

            let button = slint::platform::PointerEventButton::Left;
            if let Some(event) = touch
                .read()
                .map_err(|_| ())
                .unwrap()
                .map(|point| {
                    info!("Touch point: {:?}", point); // ← In ra tọa độ
                    let position =
                        slint::PhysicalPosition::new((point.0 * 320.) as _, (point.1 * 240.) as _)
                            .to_logical(window.scale_factor());
                    match last_touch.replace(position) {
                        Some(_) => WindowEvent::PointerMoved { position },
                        None => WindowEvent::PointerPressed { position, button },
                    }
                })
                .or_else(|| {
                    last_touch.take().map(|position| WindowEvent::PointerReleased { position, button })
                })
            {
                window.dispatch_event(event);
                // Don't go to sleep after a touch event that forces a redraw
                continue;
            }

            if window.has_active_animations() {
                continue;
            }
        }
    });

    loop {
        FreeRtos::delay_ms(1000);
    }
}

pub mod xpt2046 {
    use std::borrow::Borrow;

    use embedded_hal::spi::SpiDevice;
    use embedded_hal::digital::{InputPin, OutputPin};
    use esp_idf_hal::gpio::{Output, Pin};
    use esp_idf_hal::spi::config::Config;
    use esp_idf_hal::{
        gpio::PinDriver,
        spi::{SpiDeviceDriver, SpiDriver},
        units::Hertz,
    };
    use anyhow::Result;
    use log::info;

    pub struct XPT2046<IRQ: InputPin + 'static, CS: OutputPin, SPI: SpiDevice<u8>> {
        irq: IRQ,
        cs: CS,
        spi: SPI,
        pressed: bool,
    }

    impl<PinE, IRQ: InputPin<Error = PinE>, CS: OutputPin<Error = PinE>, SPI: SpiDevice<u8>>
        XPT2046<IRQ, CS, SPI>
    {
        pub fn new(irq: IRQ, mut cs: CS, spi: SPI) -> Result<Self, PinE> {
            cs.set_high()?;
            Ok(Self { irq, cs, spi, pressed: false })
        }

        pub fn read(&mut self) -> Result<Option<(f32, f32)>, Error<PinE, SPI::Error>> {
            const PRESS_THRESHOLD: i32 = -25_000;
            const RELEASE_THRESHOLD: i32 = -30_000;
            let threshold = if self.pressed { RELEASE_THRESHOLD } else { PRESS_THRESHOLD };
            self.pressed = false;

            if self.irq.is_low().map_err(Error::Pin)? {
                const CMD_X_READ: u8 = 0b10010000;
                const CMD_Y_READ: u8 = 0b11010000;
                const CMD_Z1_READ: u8 = 0b10110000;
                const CMD_Z2_READ: u8 = 0b11000000;

                // Điều chỉnh phạm vi hợp lý
                const MIN_X: u32 = 0;   // Giá trị nhỏ nhất thực tế
                const MAX_X: u32 = 4095; // Giá trị lớn nhất của ADC 12-bit
                const MIN_Y: u32 = 0;
                const MAX_Y: u32 = 4095;

                self.cs.set_low().map_err(Error::Pin)?;

                macro_rules! xchg {
                    ($byte:expr) => {{
                        let mut buffer = [$byte, 0, 0];
                        self.spi.transfer_in_place(&mut buffer).map_err(Error::Transfer)?;
                        ((buffer[1] as u32) << 4) | ((buffer[2] as u32) >> 4) // 12-bit ADC
                    }};
                }

                let z1 = xchg!(CMD_Z1_READ);
                let z2 = xchg!(CMD_Z2_READ);
                let z = z1 as i32 - z2 as i32;

                if z < threshold {
                    xchg!(0);
                    self.cs.set_high().map_err(Error::Pin)?;
                    return Ok(None);
                }

                xchg!(CMD_X_READ | 1); // Dummy read

                let mut point = (0u32, 0u32);
                for _ in 0..10 {
                    let y = xchg!(CMD_Y_READ);
                    let x = xchg!(CMD_X_READ);
                    point.0 += x;
                    point.1 += y;
                }

                let z1 = xchg!(CMD_Z1_READ);
                let z2 = xchg!(CMD_Z2_READ);
                let z = z1 as i32 - z2 as i32;

                xchg!(0);
                self.cs.set_high().map_err(Error::Pin)?;

                if z < RELEASE_THRESHOLD {
                    return Ok(None);
                }

                point.0 /= 10;
                point.1 /= 10;
                info!("Raw averaged point: ({}, {})", point.0, point.1); // Ghi log giá trị thô
                self.pressed = true;
                Ok(Some((
                    point.0.saturating_sub(MIN_X) as f32 / (MAX_X - MIN_X) as f32,
                    point.1.saturating_sub(MIN_Y) as f32 / (MAX_Y - MIN_Y) as f32,
                )))
            } else {
                Ok(None)
            }
        }
    }

    pub enum Error<PinE, TransferE> {
        Pin(PinE),
        Transfer(TransferE),
        InternalError,
    }

    // // Hàm set_spi_freq từ esp_idf_hala
    // pub fn set_spi_freq<'d, T: Pin + OutputPin>(
    //     spi_driver: &SpiDriver<'d>,
    //     cs_pin: Option<PinDriver<'d, T, Output>>,
    //     freq: u32,
    // ) -> Result<SpiDeviceDriver<'d>> {
    //     let baudrate = Hertz(freq);
    //     let spi_config = Config::new().baudrate(baudrate);
    //     let spi_device = SpiDeviceDriver::new(spi_driver, cs_pin, &spi_config)?;
    //     Ok(spi_device)
    // }
}