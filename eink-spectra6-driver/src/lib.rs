#![no_std]

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::spi::SpiBus;

const WIDTH: u16 = 800;
const HEIGHT: u16 = 480;
const PX: usize = WIDTH as usize * HEIGHT as usize;

pub struct Coordinates {
    pub x: u16,
    pub y: u16,
}

pub struct EpaperPort<SPI, DC, CS, RST, BUSY, DELAY> {
    spi: SPI,
    dc: DC,
    cs: CS,
    rst: RST,
    busy: BUSY,
    delay: DELAY,
    frame_buffer: [u8; PX],
}

impl<SPI, DC, CS, RST, BUSY, DELAY> EpaperPort<SPI, DC, CS, RST, BUSY, DELAY>
where
    SPI: SpiBus<u8>,
    DC: OutputPin,
    CS: OutputPin,
    RST: OutputPin,
    BUSY: InputPin,
    DELAY: DelayNs,
{
    /// Creates a new `EpaperPort` from pre-configured peripherals.
    ///
    /// The caller is responsible for constructing `spi` (an SPI bus with SCK and MOSI
    /// already attached), `dc`/`cs`/`rst` as output pins, `busy` as an input pin, and
    /// a `delay` source. Runs the display init sequence and clears to white before
    /// returning.
    pub fn new(spi: SPI, dc: DC, cs: CS, rst: RST, busy: BUSY, delay: DELAY) -> Self {
        let mut port = EpaperPort {
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
        self.send_data_buf(&[0x5F, 0x69]);
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

    pub fn render(&mut self) {
        self.begin_pixels();

        self.spi.write(&self.frame_buffer[..]).ok();

        self.end_pixels();
        self.turn_on_display();
    }

    /// Overlays a rectangular region of solid color onto the frame buffer, leaving
    /// every pixel outside the rectangle untouched. Call `render()` to push the
    /// updated buffer to the display.
    ///
    /// Coordinates are clamped to the display bounds. `color` is a 4-bit palette code:
    /// Black(0x0), White(0x1), Yellow(0x2), Red(0x3), Blue(0x5), Green(0x6).
    pub fn fill_rect(&mut self, x: u16, y: u16, width: u16, height: u16, color: u8) {
        let x0 = x as usize;
        let y0 = y as usize;
        let x1 = (x + width).min(WIDTH) as usize;
        let y1 = (y + height).min(HEIGHT) as usize;

        let row_bytes = (WIDTH / 2) as usize;

        for row in y0..y1 {
            let row_start = row * row_bytes;
            for px in x0..x1 {
                let idx = row_start + px / 2;
                if px % 2 == 0 {
                    self.frame_buffer[idx] = (self.frame_buffer[idx] & 0x0F) | (color << 4);
                } else {
                    self.frame_buffer[idx] = (self.frame_buffer[idx] & 0xF0) | color;
                }
            }
        }
    }

    /// Clears the display to solid white. Called automatically on initialisation.
    pub fn clear(&mut self) {
        let (w, h) = (WIDTH, HEIGHT);
        self.fill_rect(0, 0, w, h, 0x1);
    }

    /// Sends a pattern to the display that exercises all six available colors.
    ///
    /// Pixel (x, y) is assigned color PALETTE[(x + y) % 6], producing diagonal
    /// stripes one pixel wide. Each row is offset by one color relative to the
    /// previous, so every color appears in every column and every row.
    ///
    /// The six colors in palette order: Black(0x0), White(0x1), Yellow(0x2),
    /// Red(0x3), Blue(0x5), Green(0x6).
    ///
    /// Pixel encoding: 4bpp, 2 pixels per byte (high nibble = even x, low nibble = odd x).
    /// The 6-pixel color cycle maps to a 3-byte pattern that repeats across each row.
    ///
    /// Replaces the entire frame buffer. Call `render()` to push it to the display.
    pub fn display_checkerboard(&mut self) {
        // 4-bit color codes for the six available pigments, in cycle order.
        const PALETTE: [u8; 6] = [0x0, 0x1, 0x2, 0x3, 0x5, 0x6];

        let row_bytes = (WIDTH / 2) as usize;

        for row in 0..HEIGHT as usize {
            // Phase shifts the palette by one per row, creating the diagonal.
            let p = row % 6;

            // Six consecutive pixels pack into exactly 3 bytes, then repeat.
            let pattern = [
                (PALETTE[p] << 4) | PALETTE[(p + 1) % 6],
                (PALETTE[(p + 2) % 6] << 4) | PALETTE[(p + 3) % 6],
                (PALETTE[(p + 4) % 6] << 4) | PALETTE[(p + 5) % 6],
            ];

            let row_start = row * row_bytes;
            for i in 0..row_bytes {
                self.frame_buffer[row_start + i] = pattern[i % 3];
            }
        }
    }

    /// Renders a Mexican flag with a brown circle in place of the coat of arms, plus
    /// a "MEXICO" label below.
    ///
    /// Layout on an 800×480 display:
    ///   - White background outside the flag.
    ///   - Flag (x=[100,700], y=[50,370]) with a 2 px black border, three equal vertical
    ///     stripes: Green | White | Red.
    ///   - A brown circle (r=65) centered in the white stripe at (400, 210). Brown is
    ///     approximated by a Red/Black checkerboard dither pattern.
    ///   - "MEXICO" in black block letters (5×7 font, 10× scale) centered below the flag.
    ///
    /// Replaces the entire frame buffer. Call `render()` to push it to the display.
    pub fn display_sample(&mut self) {
        const BLACK: u8 = 0x0;
        const WHITE: u8 = 0x1;
        const RED: u8 = 0x3;
        const GREEN: u8 = 0x6;

        let w = WIDTH as i32;
        let h = HEIGHT as i32;

        // ── Flag bounds ──────────────────────────────────────────────────────
        let flag_x0: i32 = 100;
        let flag_x1: i32 = 700;
        let flag_y0: i32 = 50;
        let flag_y1: i32 = 370;
        // Three equal vertical stripes.
        let green_x1: i32 = flag_x0 + (flag_x1 - flag_x0) / 3; // 300
        let white_x1: i32 = flag_x0 + 2 * (flag_x1 - flag_x0) / 3; // 500

        // ── Brown circle (coat of arms placeholder) ──────────────────────────
        // Brown is not in the palette; approximate with a Red/Black checkerboard dither.
        let circle_cx: i32 = 400;
        let circle_cy: i32 = 210;
        let circle_r2: i32 = 65 * 65;

        // ── "MEXICO" text (5×7 font, 10× scale) ─────────────────────────────
        // Bit 4 of each row byte = leftmost column pixel.
        const GLYPHS: [[u8; 7]; 6] = [
            [
                0b10001, 0b11011, 0b10101, 0b10001, 0b10001, 0b10001, 0b10001,
            ], // M
            [
                0b11111, 0b10000, 0b10000, 0b11110, 0b10000, 0b10000, 0b11111,
            ], // E
            [
                0b10001, 0b01010, 0b01010, 0b00100, 0b01010, 0b01010, 0b10001,
            ], // X
            [
                0b11111, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b11111,
            ], // I
            [
                0b01111, 0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b01111,
            ], // C
            [
                0b01110, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01110,
            ], // O
        ];
        let scale: i32 = 10;
        let char_w: i32 = 5 * scale; // 50 px
        let char_h: i32 = 7 * scale; // 70 px
        let gap: i32 = scale; // 10 px between glyphs
        let text_w: i32 = 6 * char_w + 5 * gap; // 350 px
        let text_x0: i32 = (w - text_w) / 2; // 225
        let text_y0: i32 = 388;

        for y in 0..h {
            let color = |x: i32| -> u8 {
                // ── Outside the flag: white background or "MEXICO" label ─────
                if x < flag_x0 || x >= flag_x1 || y < flag_y0 || y >= flag_y1 {
                    let tx = x - text_x0;
                    let ty = y - text_y0;
                    if tx >= 0 && ty >= 0 && ty < char_h {
                        let li = (tx / (char_w + gap)) as usize;
                        if li < 6 {
                            let lx = tx - li as i32 * (char_w + gap);
                            if lx < char_w {
                                let gc = (lx / scale) as usize;
                                let gr = (ty / scale) as usize;
                                if gc < 5 && (GLYPHS[li][gr] >> (4 - gc)) & 1 == 1 {
                                    return BLACK;
                                }
                            }
                        }
                    }
                    return WHITE;
                }

                // ── 2 px black border ────────────────────────────────────────
                if y < flag_y0 + 2 || y >= flag_y1 - 2 || x < flag_x0 + 2 || x >= flag_x1 - 2 {
                    return BLACK;
                }

                // ── Brown circle (Red/Black checkerboard dither) ─────────────
                let dx = x - circle_cx;
                let dy = y - circle_cy;
                if dx * dx + dy * dy <= circle_r2 {
                    return if (x + y) % 2 == 0 { RED } else { BLACK };
                }

                // ── Base stripe color ────────────────────────────────────────
                if x < green_x1 {
                    GREEN
                } else if x < white_x1 {
                    WHITE
                } else {
                    RED
                }
            };

            let row_bytes = (WIDTH / 2) as usize;
            let row_start = y as usize * row_bytes;
            for i in 0..row_bytes {
                let x = (i * 2) as i32;
                self.frame_buffer[row_start + i] = (color(x) << 4) | color(x + 1);
            }
        }
    }
}
