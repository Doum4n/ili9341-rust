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

// Configuration constants
struct Config {
    hor_res: u32,
    ver_res: u32,
    spi_baudrate: Hertz,
    display_size: slint::PhysicalSize,
}

impl Config {
    const fn new() -> Self {
        Config {
            hor_res: 320,
            ver_res: 240,
            spi_baudrate: Hertz(10_000_000),
            display_size: slint::PhysicalSize::new(320, 240),
        }
    }
}

// Display module
mod display {
    use super::*;
    use embedded_dma::ReadBuffer;
    use embedded_graphics::{
        geometry::Point as EgPoint,
        pixelcolor::Rgb565,
        prelude::{Size},
        primitives::Rectangle,
    };

    pub struct DisplayWrapper<'a, D> {
        display: Rc<RefCell<D>>,
        buffer: &'static mut [Rgb565Pixel],
        spi: RefCell<SpiDeviceDriver<'static, Arc<SpiDriver<'static>>>>,
        dc: PinDriver<'a, Gpio2, Output>,
        cs: PinDriver<'a, Gpio15, Output>,
        scroller: Scroller,
    }

    impl<'a, D> DisplayWrapper<'a, D>
    where
        D: embedded_graphics::draw_target::DrawTarget<Color = Rgb565>,
    {
        pub fn new(
            display: D,
            buffer: &'static mut [Rgb565Pixel],
            spi: SpiDeviceDriver<'static, Arc<SpiDriver<'static>>>,
            dc: PinDriver<'a, Gpio2, Output>,
            cs: PinDriver<'a, Gpio15, Output>,
        ) -> Self {
            DisplayWrapper {
                display: Rc::new(RefCell::new(display)),
                buffer,
                spi: RefCell::new(spi),
                dc,
                cs,
                scroller: Scroller::new(0, 240, 0),
            }
        }

        pub fn init_scroll_area(&mut self, tfa: u16, vsa: u16, bfa: u16) -> Result<(), anyhow::Error> {
            self.scroller = Scroller::new(tfa, vsa, bfa);
            set_vertical_scroll_area(&mut self.spi.borrow_mut(), &mut self.dc, &mut self.cs, tfa, vsa, bfa)?;
            Ok(())
        }

        pub fn scroll_vertically(&mut self, num_lines: i16) -> Result<(), anyhow::Error> {
            info!("Scrolling by {} lines", num_lines);
            let max_offset = self.scroller.tfa + self.scroller.vsa + self.scroller.bfa - 1;
            let new_offset = if num_lines >= 0 {
                self.scroller.top_offset.saturating_add(num_lines as u16) % (max_offset + 1)
            } else {
                self.scroller.top_offset.saturating_sub((-num_lines) as u16) % (max_offset + 1)
            };
            self.scroller.top_offset = new_offset;
            info!("New offset: {}", new_offset);
            set_vertical_scroll_offset(&mut self.spi.borrow_mut(), &mut self.dc, &mut self.cs, new_offset)?;
            Ok(())
        }
    }

    impl<'a, D> slint::platform::software_renderer::LineBufferProvider for &mut DisplayWrapper<'a, D>
    where
        D: embedded_graphics::draw_target::DrawTarget<Color = Rgb565>,
    {
        type TargetPixel = Rgb565Pixel;

        fn process_line(
            &mut self,
            line: usize,
            range: core::ops::Range<usize>,
            render_fn: impl FnOnce(&mut [Self::TargetPixel]),
        ) {
            render_fn(&mut self.buffer[range.clone()]);
            let partial_buffer = PartialBuffer(self.buffer, range.clone());
            let (buffer_ptr, buffer_len) = unsafe { partial_buffer.read_buffer() };
            let pixel_data = unsafe { core::slice::from_raw_parts(buffer_ptr, buffer_len) };

            let rect = Rectangle::new(EgPoint::new(range.start as _, line as _), Size::new(range.len() as _, 1));
            set_display_window(&mut self.spi.borrow_mut(), &mut self.dc, &mut self.cs, &rect).unwrap();
            self.cs.set_low().unwrap();
            self.dc.set_high().unwrap();
            self.spi.borrow_mut().write(pixel_data).unwrap();
            self.cs.set_high().unwrap();
        }
    }

    pub fn set_display_window<'d, T: std::borrow::Borrow<SpiDriver<'d>>>(
        spi: &mut SpiDeviceDriver<'d, T>,
        dc: &mut PinDriver<Gpio2, Output>,
        cs: &mut PinDriver<Gpio15, Output>,
        rect: &Rectangle,
    ) -> Result<(), anyhow::Error> {
        cs.set_low()?;
        dc.set_low()?;
        spi.write(&[0x2A])?;
        dc.set_high()?;
        let x_start = rect.top_left.x as u16;
        let x_end = (rect.top_left.x + rect.size.width as i32 - 1) as u16;
        spi.write(&[(x_start >> 8) as u8, x_start as u8, (x_end >> 8) as u8, x_end as u8])?;
        cs.set_high()?;

        cs.set_low()?;
        dc.set_low()?;
        spi.write(&[0x2B])?;
        dc.set_high()?;
        let y_start = rect.top_left.y as u16;
        let y_end = (rect.top_left.y + rect.size.height as i32 - 1) as u16;
        spi.write(&[(y_start >> 8) as u8, y_start as u8, (y_end >> 8) as u8, y_end as u8])?;
        cs.set_high()?;

        cs.set_low()?;
        dc.set_low()?;
        spi.write(&[0x2C])?;
        cs.set_high()?;
        Ok(())
    }

    pub fn set_vertical_scroll_area<'d, T: std::borrow::Borrow<SpiDriver<'d>>>(
        spi: &mut SpiDeviceDriver<'d, T>,
        dc: &mut PinDriver<Gpio2, Output>,
        cs: &mut PinDriver<Gpio15, Output>,
        tfa: u16,
        vsa: u16,
        bfa: u16,
    ) -> Result<(), anyhow::Error> {
        cs.set_low()?;
        dc.set_low()?;
        spi.write(&[0x33])?;
        dc.set_high()?;
        spi.write(&[(tfa >> 8) as u8, tfa as u8, (vsa >> 8) as u8, vsa as u8, (bfa >> 8) as u8, bfa as u8])?;
        cs.set_high()?;
        Ok(())
    }

    pub fn set_vertical_scroll_offset<'d, T: std::borrow::Borrow<SpiDriver<'d>>>(
        spi: &mut SpiDeviceDriver<'d, T>,
        dc: &mut PinDriver<Gpio2, Output>,
        cs: &mut PinDriver<Gpio15, Output>,
        top_offset: u16,
    ) -> Result<(), anyhow::Error> {
        info!("Setting scroll offset: {}", top_offset);
        cs.set_low()?;
        dc.set_low()?;
        spi.write(&[0x37])?;
        dc.set_high()?;
        spi.write(&[(top_offset >> 8) as u8, top_offset as u8])?;
        cs.set_high()?;
        Ok(())
    }
}

// Touch module
mod touch {
    use super::*;
    use embedded_hal::digital::{InputPin, OutputPin};
    use embedded_hal::spi::SpiDevice;

    pub struct XPT2046<IRQ: InputPin + 'static, CS: OutputPin, SPI: SpiDevice<u8>> {
        irq: IRQ,
        cs: CS,
        spi: SPI,
        pressed: bool,
    }

    impl<PinE, IRQ: InputPin<Error = PinE>, CS: OutputPin<Error = PinE>, SPI: SpiDevice<u8>> XPT2046<IRQ, CS, SPI> {
        pub fn new(irq: IRQ, mut cs: CS, spi: SPI) -> Result<Self, PinE> {
            cs.set_high()?;
            Ok(Self {
                irq,
                cs,
                spi,
                pressed: false,
            })
        }

        pub fn read(&mut self) -> Result<Option<(f32, f32)>, Error<PinE, SPI::Error>> {
            const PRESS_THRESHOLD: i32 = -25_000;
            const RELEASE_THRESHOLD: i32 = -30_000;
            const MIN_X: u32 = 200;
            const MAX_X: u32 = 1900;
            const MIN_Y: u32 = 300;
            const MAX_Y: u32 = 1900;

            let threshold = if self.pressed { RELEASE_THRESHOLD } else { PRESS_THRESHOLD };
            self.pressed = false;

            if self.irq.is_low().map_err(Error::Pin)? {
                const CMD_X_READ: u8 = 0b10010000;
                const CMD_Y_READ: u8 = 0b11010000;
                const CMD_Z1_READ: u8 = 0b10110000;
                const CMD_Z2_READ: u8 = 0b11000000;

                self.cs.set_low().map_err(Error::Pin)?;
                macro_rules! xchg {
                    ($byte:expr) => {{
                        let mut buffer = [$byte, 0, 0];
                        self.spi.transfer_in_place(&mut buffer).map_err(Error::Transfer)?;
                        ((buffer[1] as u32) << 4) | ((buffer[2] as u32) >> 4)
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

                xchg!(CMD_X_READ | 1);
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
                info!("Raw averaged point: ({}, {})", point.0, point.1);
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

    #[derive(Debug)]
    pub enum Error<PinE, TransferE> {
        Pin(PinE),
        Transfer(TransferE),
        InternalError,
    }
}

// Platform module
mod platform {
    use super::*;
    use embedded_graphics::draw_target::DrawTarget;

    pub struct Ili9341Platform<'a, D, Touch> {
        window: RefCell<Option<Rc<MinimalSoftwareWindow>>>,
        buffer_provider: RefCell<super::display::DisplayWrapper<'a, D>>,
        touch: RefCell<Touch>,
        start_time: Instant,
    }

    impl<'a, D, IRQ, CS, PinE, SPI> Platform for Ili9341Platform<'a, D, super::touch::XPT2046<IRQ, CS, SPI>>
    where
        IRQ: embedded_hal::digital::InputPin<Error = PinE>,
        CS: embedded_hal::digital::OutputPin<Error = PinE>,
        SPI: embedded_hal::spi::SpiDevice<u8>,
        D: DrawTarget<Color = embedded_graphics_core::pixelcolor::Rgb565>,
    {
        fn create_window_adapter(
            &self,
        ) -> Result<Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError> {
            let window = MinimalSoftwareWindow::new(slint::platform::software_renderer::RepaintBufferType::ReusedBuffer);
            self.window.replace(Some(window.clone()));
            Ok(window)
        }

        fn duration_since_start(&self) -> Duration {
            Instant::now().duration_since(self.start_time)
        }

        fn run_event_loop(&self) -> Result<(), slint::PlatformError> {
            let mut last_touch = None;
            let mut last_y: Option<f32> = None;
            let ui = super::create_slint_app();
            let ui_handle = ui.as_weak();

            self.window.borrow().as_ref().unwrap().set_size(super::Config::new().display_size);
            self.buffer_provider
                .borrow_mut()
                .init_scroll_area(0, 240, 0)
                .map_err(|_| slint::PlatformError::Other("Failed to init scroll area".into()))?;

            loop {
                slint::platform::update_timers_and_animations();
                if let Some(window) = self.window.borrow().clone() {
                    window.draw_if_needed(|renderer| {
                        let mut buffer_provider = self.buffer_provider.borrow_mut();
                        renderer.render_by_line(&mut *buffer_provider);
                    });

                    let button = slint::platform::PointerEventButton::Left;
                    if let Some(event) = self
                        .touch
                        .borrow_mut()
                        .read()
                        .map_err(|_| slint::PlatformError::Other("Touch read error".into()))?
                        .map(|point| {
                            info!("Touch point: {:?}", point);
                            let position = slint::PhysicalPosition::new((point.0 * 320.0) as _, (point.1 * 240.0) as _)
                                .to_logical(self.window.borrow().as_ref().unwrap().scale_factor());

                            if let Some(last_y_pos) = last_y {
                                let delta_y = point.1 - last_y_pos;
                                if delta_y.abs() > 0.01 {
                                    let num_lines = (delta_y * 50.0) as i16;
                                    self.buffer_provider
                                        .borrow_mut()
                                        .scroll_vertically(-num_lines)
                                        .map_err(|_| slint::PlatformError::Other("Scroll error".into()))
                                        .unwrap();
                                    if let Some(ui) = ui_handle.upgrade() {
                                        let current_offset = ui.get_scroll_offset();
                                        ui.set_scroll_offset(current_offset + num_lines as f32);
                                    }
                                }
                            }
                            last_y = Some(point.1);

                            match last_touch.replace(position) {
                                Some(_) => WindowEvent::PointerMoved { position },
                                None => WindowEvent::PointerPressed { position, button },
                            }
                        })
                        .or_else(|| last_touch.take().map(|position| WindowEvent::PointerReleased { position, button }))
                    {
                        self.window.borrow().as_ref().unwrap().dispatch_event(event);
                        continue;
                    }

                    if self.window.borrow().as_ref().unwrap().has_active_animations() {
                        continue;
                    }
                }
                FreeRtos::delay_ms(1);
            }
        }
    }

    impl<'a, D, Touch> Ili9341Platform<'a, D, Touch> {
        pub fn new(buffer_provider: super::display::DisplayWrapper<'a, D>, touch: Touch) -> Self {
            Ili9341Platform {
                window: Default::default(),
                buffer_provider: RefCell::new(buffer_provider),
                touch: RefCell::new(touch),
                start_time: Instant::now(),
            }
        }
    }
}

// Utility structs
pub struct GpioOutput<'a, T>
where
    T: Pin,
{
    pin: PinDriver<'a, T, Output>,
}

impl<'a, T> embedded_hal::digital::ErrorType for GpioOutput<'a, T>
where
    T: Pin,
{
    type Error = GpioError;
}

impl<'a, T> GpioOutput<'a, T>
where
    T: Pin,
{
    pub fn new(pin: PinDriver<'a, T, Output>) -> Self {
        GpioOutput { pin }
    }
}

impl<'a, T> embedded_hal::digital::OutputPin for GpioOutput<'a, T>
where
    T: Pin,
{
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.pin.set_low()?;
        info!("Reset pin set low");
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.pin.set_high()?;
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

pub struct Scroller {
    top_offset: u16,
    tfa: u16,
    vsa: u16,
    bfa: u16,
}

impl Scroller {
    pub fn new(tfa: u16, vsa: u16, bfa: u16) -> Scroller {
        Scroller { top_offset: tfa, tfa, vsa, bfa }
    }
}

struct PartialBuffer<'a>(&'a mut [Rgb565Pixel], core::ops::Range<usize>);

unsafe impl<'a> embedded_dma::ReadBuffer for PartialBuffer<'a> {
    type Word = u8;

    unsafe fn read_buffer(&self) -> (*const Self::Word, usize) {
        let act_slice = &self.0[self.1.clone()];
        (act_slice.as_ptr() as *const u8, act_slice.len() * core::mem::size_of::<Rgb565Pixel>())
    }
}

unsafe impl<'a> embedded_dma::WriteBuffer for PartialBuffer<'a> {
    type Word = u8;

    unsafe fn write_buffer(&mut self) -> (*mut Self::Word, usize) {
        let act_slice = &mut self.0[self.1.clone()];
        (act_slice.as_mut_ptr() as *mut u8, act_slice.len() * core::mem::size_of::<Rgb565Pixel>())
    }
}

static mut BUFFER: [Rgb565Pixel; 320 * 240] = [Rgb565Pixel(0); 320 * 240];

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
        let mut touch_irq = PinDriver::input(peripherals.pins.gpio4).unwrap();
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
        let buffer_provider = display::DisplayWrapper::new(lcd, unsafe { &mut BUFFER }, clone_spi_display.into_inner(), dc_copy, cs_copy);

        let platform = platform::Ili9341Platform::new(buffer_provider, touch);
        slint::platform::set_platform(Box::new(platform)).unwrap();
        slint::run_event_loop().unwrap();
    })?;

    loop {
        FreeRtos::delay_ms(1000);
    }
}