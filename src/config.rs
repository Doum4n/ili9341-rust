use esp_idf_hal::prelude::*;
use slint::PhysicalSize;

pub struct Config {
    pub hor_res: u32,
    pub ver_res: u32,
    pub spi_baudrate: Hertz,
    pub display_size: PhysicalSize,
}

impl Config {
    pub const fn new() -> Self {
        Config {
            hor_res: 320,
            ver_res: 240,
            spi_baudrate: Hertz(10_000_000),
            display_size: PhysicalSize::new(320, 240),
        }
    }
}
