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

        self.window.borrow().as_ref().unwrap().set_size(super::config::Config::new().display_size);
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
