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
