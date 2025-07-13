use super::*;
use esp_idf_hal::gpio::{GpioError, Output, PinDriver, Pin};
use slint::platform::software_renderer::Rgb565Pixel;

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

pub struct PartialBuffer<'a> {
    pub buffer: &'a mut [Rgb565Pixel],
    pub range: core::ops::Range<usize>,
}

unsafe impl<'a> embedded_dma::ReadBuffer for PartialBuffer<'a> {
    type Word = u8;

    unsafe fn read_buffer(&self) -> (*const Self::Word, usize) {
        let act_slice = &self.buffer[self.range.clone()];
        (act_slice.as_ptr() as *const u8, act_slice.len() * core::mem::size_of::<Rgb565Pixel>())
    }
}

unsafe impl<'a> embedded_dma::WriteBuffer for PartialBuffer<'a> {
    type Word = u8;

    unsafe fn write_buffer(&mut self) -> (*mut Self::Word, usize) {
        let act_slice = &mut self.buffer[self.range.clone()];
        (act_slice.as_mut_ptr() as *mut u8, act_slice.len() * core::mem::size_of::<Rgb565Pixel>())
    }
}
