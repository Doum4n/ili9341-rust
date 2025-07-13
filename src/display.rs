use super::*;
use embedded_dma::ReadBuffer;
use embedded_graphics::{
    geometry::Point as EgPoint, pixelcolor::Rgb565, prelude::Size, primitives::Rectangle,
};

pub struct DisplayWrapper<'a, D> {
    pub display: Rc<RefCell<D>>,
    pub buffer: &'static mut [Rgb565Pixel],
    pub spi: RefCell<SpiDeviceDriver<'static, Arc<SpiDriver<'static>>>>,
    pub dc: PinDriver<'a, Gpio2, Output>,
    pub cs: PinDriver<'a, Gpio15, Output>,
    pub scroller: Scroller,
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
        set_vertical_scroll_area(
            &mut self.spi.borrow_mut(),
            &mut self.dc,
            &mut self.cs,
            tfa,
            vsa,
            bfa,
        )?;
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
        set_vertical_scroll_offset(
            &mut self.spi.borrow_mut(),
            &mut self.dc,
            &mut self.cs,
            new_offset,
        )?;
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
        // render_fn(&mut self.buffer[range.clone()]);
        // let rect = Rectangle::new(EgPoint::new(range.start as _, line as _), Size::new(range.len() as _, 1));
        // self.display
        //     .borrow_mut()
        //     .fill_contiguous(
        //         &rect,
        //         self.buffer[range.clone()].iter().map(|p| {
        //             embedded_graphics_core::pixelcolor::raw::RawU16::new(p.0).into()
        //         }),
        //     )
        //     .map_err(drop)
        //     .unwrap();

        render_fn(&mut self.buffer[range.clone()]);
        let partial_buffer = PartialBuffer{buffer: self.buffer, range: range.clone()};
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
    spi.write(&[0x2A])?; // Column Address Set (2Ah)
    dc.set_high()?;
    let x_start = rect.top_left.x as u16;
    let x_end = (rect.top_left.x + rect.size.width as i32 - 1) as u16;
    spi.write(&[
        (x_start >> 8) as u8,
        x_start as u8,
        (x_end >> 8) as u8,
        x_end as u8,
    ])?;
    cs.set_high()?;

    cs.set_low()?;
    dc.set_low()?;
    spi.write(&[0x2B])?; // Page Address Set (2Bh) *row*
    dc.set_high()?;
    let y_start = rect.top_left.y as u16;
    let y_end = (rect.top_left.y + rect.size.height as i32 - 1) as u16;
    spi.write(&[
        (y_start >> 8) as u8,
        y_start as u8,
        (y_end >> 8) as u8,
        y_end as u8,
    ])?;
    cs.set_high()?;

    cs.set_low()?;
    dc.set_low()?;
    spi.write(&[0x2C])?;
    cs.set_high()?; // Memory Write (2Ch)
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
    spi.write(&[
        (tfa >> 8) as u8,
        tfa as u8,
        (vsa >> 8) as u8,
        vsa as u8,
        (bfa >> 8) as u8,
        bfa as u8,
    ])?;
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

pub struct Scroller {
    pub top_offset: u16,
    pub tfa: u16,
    pub vsa: u16,
    pub bfa: u16,
}

impl Scroller {
    pub fn new(tfa: u16, vsa: u16, bfa: u16) -> Scroller {
        Scroller {
            top_offset: tfa,
            tfa,
            vsa,
            bfa,
        }
    }
}
