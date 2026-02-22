#![no_std]

use embedded_hal::spi::SpiBus;
use esp_hal::delay::Delay;
use esp_hal::dma_buffers;
use esp_hal::gpio::interconnect::PeripheralOutput;
use esp_hal::gpio::{Input, InputConfig, InputPin, Level, Output, OutputConfig, OutputPin};
use esp_hal::spi::Mode;
use esp_hal::spi::master::{Config, Instance, Spi};
use esp_hal::time::Rate;

pub struct EpaperPort<'d> {
    spi: Spi<'d, esp_hal::Blocking>,
    dc: Output<'d>,
    rst: Output<'d>,
    busy: Input<'d>,
    width: u16,
    height: u16,
    scale_max_width: u16,
    scale_max_height: u16,
}

impl<'d> EpaperPort<'d> {
    pub fn new(
        spi: impl Instance + 'd,
        sck: impl PeripheralOutput<'d>,
        cs: impl PeripheralOutput<'d>,
        mosi: impl PeripheralOutput<'d>,
        dc: impl OutputPin + 'd,
        rst: impl OutputPin + 'd,
        busy: impl InputPin + 'd,
        width: u16,
        height: u16,
        scale_max_width: u16,
        scale_max_height: u16,
    ) -> anyhow::Result<Self> {
        let spi_config = Config::default()
            .with_mode(Mode::_0)
            .with_frequency(Rate::from_mhz(40));
        let spi = Spi::new(spi, spi_config)?
            .with_sck(sck)
            .with_mosi(mosi)
            .with_cs(cs);
        let dc = Output::new(dc, Level::Low, OutputConfig::default());
        let rst = Output::new(rst, Level::High, OutputConfig::default());
        let busy = Input::new(busy, InputConfig::default());

        let mut port = EpaperPort {
            spi,
            dc,
            rst,
            busy,
            width,
            height,
            scale_max_width,
            scale_max_height,
        };
        port.init();
        Ok(port)
    }

    fn send_command(&mut self, cmd: u8) {
        self.dc.set_low();
        self.spi.write(&[cmd]).ok();
    }

    fn send_data(&mut self, data: u8) {
        self.dc.set_high();
        self.spi.write(&[data]).ok();
    }

    fn send_data_buf(&mut self, data: &[u8]) {
        self.dc.set_high();
        self.spi.write(data).ok();
    }

    fn wait_busy(&mut self) {
        // HIGH = display ready, LOW = display still processing
        while self.busy.is_low() {}
    }

    fn reset(&mut self) {
        let delay = Delay::new();
        self.rst.set_high();
        delay.delay_millis(50);
        self.rst.set_low();
        delay.delay_millis(20);
        self.rst.set_high();
        delay.delay_millis(50);
    }

    fn init(&mut self) {
        self.reset();

        self.send_command(0xAA);
        self.send_data_buf(&[0x49, 0x55, 0x20, 0x08, 0x09, 0x18]);

        self.send_command(0x01);
        self.send_data(0x3F);

        self.send_command(0x00);
        self.send_data_buf(&[0x5F, 0x69]);

        self.send_command(0x03);
        self.send_data_buf(&[0x00, 0x54, 0x00, 0x44]);

        self.send_command(0x05);
        self.send_data_buf(&[0x40, 0x1F, 0x1F, 0x2C]);

        self.send_command(0x06);
        self.send_data_buf(&[0x6F, 0x1F, 0x17, 0x49]);

        self.send_command(0x08);
        self.send_data_buf(&[0x6F, 0x1F, 0x1F, 0x22]);

        self.send_command(0x30);
        self.send_data(0x03);

        self.send_command(0x50);
        self.send_data(0x3F);

        self.send_command(0x60);
        self.send_data_buf(&[0x02, 0x00]);

        // Resolution: 800 x 480 (0x0320 x 0x01E0)
        self.send_command(0x61);
        self.send_data_buf(&[0x03, 0x20, 0x01, 0xE0]);

        self.send_command(0x84);
        self.send_data(0x01);

        self.send_command(0xE3);
        self.send_data(0x2F);

        // Power on and wait for display ready
        self.send_command(0x04);
        self.wait_busy();
    }

    fn turn_on_display(&mut self) {
        self.send_command(0x04);
        self.wait_busy();
        self.send_command(0x06);
        self.send_data_buf(&[0x6F, 0x1F, 0x17, 0x49]);
        self.send_command(0x12); // display refresh
        self.send_data(0x00);
        self.wait_busy();
        self.send_command(0x02); // power off
        self.send_data(0x00);
        self.wait_busy();
    }

    /// Sends a checkerboard pattern to the display: every pixel alternates between
    /// black and white, and the pattern inverts on each row.
    ///
    /// Pixel encoding: 4bpp, 2 pixels per byte (high nibble = even x, low nibble = odd x).
    /// Black=0x0, White=0x1.
    /// - Even rows: byte 0x01 (black at even x, white at odd x)
    /// - Odd rows:  byte 0x10 (white at even x, black at odd x)
    pub fn display_checkerboard(&mut self) {
        let row_bytes = (self.width / 2) as usize;

        self.send_command(0x10);
        for row in 0..self.height {
            let byte = if row % 2 == 0 { 0x01u8 } else { 0x10u8 };
            let mut remaining = row_bytes;
            while remaining > 0 {
                let chunk_size = remaining.min(64);
                let chunk = [byte; 64];
                self.send_data_buf(&chunk[..chunk_size]);
                remaining -= chunk_size;
            }
        }

        self.turn_on_display();
    }
}

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        let result = 2 + 2;
        assert_eq!(result, 4);
    }
}
