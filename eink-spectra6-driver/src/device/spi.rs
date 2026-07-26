use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::spi::SpiBus;
use std::time::Instant;

use super::{HEIGHT, PX, ROW_BYTES, Spectra6Screen};

pub struct Spectra6SpiDriver<SPI, DC, CS, RST, BUSY, DELAY> {
    spi: SPI,
    dc: DC,
    cs: CS,
    rst: RST,
    busy: BUSY,
    delay: DELAY,
    frame_buffer: [u8; PX],
}

impl<SPI, DC, CS, RST, BUSY, DELAY> Spectra6Screen
    for Spectra6SpiDriver<SPI, DC, CS, RST, BUSY, DELAY>
where
    SPI: SpiBus<u8>,
    DC: OutputPin,
    CS: OutputPin,
    RST: OutputPin,
    BUSY: InputPin,
    DELAY: DelayNs,
{
    fn render(&mut self) {
        let start = Instant::now();
        self.begin_pixels();

        // The Linux spidev character device rejects any single write() larger than
        // its `bufsiz` (default 4096 bytes on Raspberry Pi OS) with -EMSGSIZE, so the
        // full buffer must be streamed in bounded chunks rather than in one write.
        for chunk in self.frame_buffer.chunks(64) {
            self.spi.write(chunk).ok();
        }

        self.end_pixels();
        self.turn_on_display();
        let duration = start.elapsed();
        tracing::info!(
            component = "spectra6-spi-driver",
            duration_ms = duration.as_millis(),
            func = "render",
            "rendered frame buffer to display",
        );
    }

    fn set_buffer(&mut self, byte_offset: usize, data: &[u8]) {
        let end = (byte_offset + data.len()).min(self.frame_buffer.len());
        let len_written = end - byte_offset;
        let data = &data[..len_written];
        if data.is_empty() {
            tracing::warn!(
                component = "spectra6-spi-driver",
                byte_offset = byte_offset,
                data_len = data.len(),
                func = "set_buffer",
                "no data to write to frame buffer",
            );
            return;
        }

        self.frame_buffer[byte_offset..end].copy_from_slice(data);
    }

    fn set_pixel_row(
        &mut self,
        idx: u16,
        data: [u8; ROW_BYTES as usize],
        start_px: u16,
        end_px: u16,
    ) {
        if idx < HEIGHT {
            // First compute row ranges (in bytes)
            let data_subset = &data[(start_px as usize / 2)..(end_px as usize / 2)];

            let offset = (idx as usize) * (ROW_BYTES as usize) + (start_px as usize / 2);
            let first_byte = self.frame_buffer[offset];
            let last_byte = self.frame_buffer[offset + (data_subset.len() - 1)];
            let bytes_to_write = data_subset.len();

            self.frame_buffer[offset..offset + bytes_to_write].copy_from_slice(&data_subset);
            if start_px % 2 == 1 {
                // If the start pixel is odd, we need to preserve the first nibble of the first byte
                self.frame_buffer[offset] =
                    (first_byte & 0xF0) | (self.frame_buffer[offset] & 0x0F);
            }
            if end_px % 2 == 1 {
                // If the end pixel is odd, we need to preserve the second nibble of the last byte
                self.frame_buffer[offset + (bytes_to_write - 1)] =
                    (self.frame_buffer[offset + (bytes_to_write - 1)] & 0xF0) | (last_byte & 0x0F);
            }
        } else {
            tracing::warn!(
                component = "spectra6-spi-driver",
                row_idx = idx,
                func = "set_pixel_row",
                "row index out of bounds",
            );
        }
    }

    fn clear(&mut self) {
        // 0x11 fills both nibbles with white
        self.frame_buffer.fill(0x11);
    }
}

impl<SPI, DC, CS, RST, BUSY, DELAY> Spectra6SpiDriver<SPI, DC, CS, RST, BUSY, DELAY>
where
    SPI: SpiBus<u8>,
    DC: OutputPin,
    CS: OutputPin,
    RST: OutputPin,
    BUSY: InputPin,
    DELAY: DelayNs,
{
    /// Creates a new `Spectra6SpiDriver` from pre-configured peripherals.
    ///
    /// The caller is responsible for constructing `spi` (an SPI bus with SCK and MOSI
    /// already attached), `dc`/`cs`/`rst` as output pins, `busy` as an input pin, and
    /// a `delay` source. Runs the display init sequence and clears to white before
    /// returning.
    pub fn new(spi: SPI, dc: DC, cs: CS, rst: RST, busy: BUSY, delay: DELAY) -> Self {
        let mut port = Spectra6SpiDriver {
            spi,
            dc,
            cs,
            rst,
            busy,
            delay,
            frame_buffer: [0u8; PX],
        };
        port.init();
        port.clear();
        port
    }

    fn send_command(&mut self, cmd: u8) {
        self.dc.set_low().ok();
        self.cs.set_low().ok();
        self.spi.write(&[cmd]).ok();
        self.cs.set_high().ok();
    }

    fn send_data(&mut self, data: u8) {
        self.dc.set_high().ok();
        self.cs.set_low().ok();
        self.spi.write(&[data]).ok();
        self.cs.set_high().ok();
    }

    fn send_data_buf(&mut self, data: &[u8]) {
        self.dc.set_high().ok();
        self.cs.set_low().ok();
        self.spi.write(data).ok();
        self.cs.set_high().ok();
    }

    // Opens DTM (0x10) and holds CS low for the duration of a large pixel data transfer.
    fn begin_pixels(&mut self) {
        self.send_command(0x10);
        self.dc.set_high().ok();
        self.cs.set_low().ok();
    }

    fn end_pixels(&mut self) {
        self.cs.set_high().ok();
    }

    fn wait_busy(&mut self) {
        // HIGH = display ready, LOW = display still processing
        while self.busy.is_low().unwrap_or(false) {
            self.delay.delay_ms(10);
        }
    }

    fn reset(&mut self) {
        self.rst.set_high().ok();
        self.delay.delay_ms(50);
        self.rst.set_low().ok();
        self.delay.delay_ms(20);
        self.rst.set_high().ok();
        self.delay.delay_ms(50);
    }

    fn send_sequence_label(&mut self) {
        self.send_command(0xAA);
        self.send_data_buf(&[0x49, 0x55, 0x20, 0x08, 0x09, 0x18]);
    }

    fn send_power_setting(&mut self) {
        self.send_command(0x01);
        self.send_data(0x3F);
    }

    fn send_panel_setting(&mut self) {
        self.send_command(0x00);
        self.send_data_buf(&[0x53, 0x69]);
    }

    fn send_power_off_sequence(&mut self) {
        self.send_command(0x03);
        self.send_data_buf(&[0x00, 0x54, 0x00, 0x44]);
    }

    fn send_booster_start_a(&mut self) {
        self.send_command(0x05);
        self.send_data_buf(&[0x40, 0x1F, 0x1F, 0x2C]);
    }

    fn send_booster_start_b(&mut self) {
        self.send_command(0x06);
        self.send_data_buf(&[0x6F, 0x1F, 0x17, 0x49]);
    }

    fn send_booster_start_c(&mut self) {
        self.send_command(0x08);
        self.send_data_buf(&[0x6F, 0x1F, 0x1F, 0x22]);
    }

    fn send_pll_setting(&mut self) {
        self.send_command(0x30);
        self.send_data(0x03);
    }

    fn send_vcom_data_interval(&mut self) {
        self.send_command(0x50);
        self.send_data(0x3F);
    }

    fn send_tcon_setting(&mut self) {
        self.send_command(0x60);
        self.send_data_buf(&[0x02, 0x00]);
    }

    fn send_resolution(&mut self) {
        self.send_command(0x61);
        self.send_data_buf(&[0x03, 0x20, 0x01, 0xE0]);
    }

    fn send_tvdcs(&mut self) {
        self.send_command(0x84);
        self.send_data(0x01);
    }

    fn send_power_saving(&mut self) {
        self.send_command(0xE3);
        self.send_data(0x2F);
    }

    fn send_power_on(&mut self) {
        self.send_command(0x04);
    }

    fn send_display_refresh(&mut self) {
        self.send_command(0x12);
        self.send_data(0x00);
    }

    fn send_power_off(&mut self) {
        self.send_command(0x02);
        self.send_data(0x00);
    }

    fn init(&mut self) {
        self.reset();
        self.wait_busy();
        self.delay.delay_ms(50);

        self.send_sequence_label();
        self.send_power_setting();
        self.send_panel_setting();
        self.send_power_off_sequence();
        self.send_booster_start_a();
        self.send_booster_start_b();
        self.send_booster_start_c();
        self.send_pll_setting();
        self.send_vcom_data_interval();
        self.send_tcon_setting();
        self.send_resolution();
        self.send_tvdcs();
        self.send_power_saving();

        self.send_power_on();
        self.wait_busy();
    }

    fn turn_on_display(&mut self) {
        self.send_power_on();
        self.wait_busy();
        self.send_booster_start_b();
        self.send_display_refresh();
        self.wait_busy();
        self.send_power_off();
        self.wait_busy();
    }
}
