//! This module contains the lower level controls for the eink display. This is where the protocol
//! defined in `protocol.md` is largely implemented. We only really provide a very bare interface
//! to control the eink display, with frame buffer manipulation and rendering capabilities. Drawing
//! more complex graphics is done in wrappers around this module.

pub mod spi;
pub use spi::Spectra6SpiDriver;

pub const WIDTH: u16 = 800;
pub const ROW_BYTES: u16 = WIDTH / 2;
pub const HEIGHT: u16 = 480;
const PX: usize = ROW_BYTES as usize * HEIGHT as usize;

pub trait Spectra6Screen {
    /// Call this function to render what you've accumulated in the frame buffer to the eink
    /// display.
    fn render(&mut self);

    /// Set a section of the buffer manually. Please note the convention this screen uses when
    /// drawing on the frame buffer:
    ///
    /// - The screen is [`Self::WIDTH`] by [`Self::HEIGHT`] pixels in size (we render oriented with
    ///   the screen laying on its side).
    /// - Each byte contains two pixels, with each nibble representing a single 4-bit colour code
    /// - The colours are, in order are Black (0x0), White (0x1), Yellow (0x2), Red (0x3), Blue
    ///   (0x5) and Green (0x6). Notice it skips 0x4
    ///
    /// If the provided data goes over the length of the buffer, we will ignore the rest of the data
    /// silently.
    ///
    /// # Arguments
    /// * `byte_offset`  - Offset into the frame buffer to start writing at. This is in bytes, not
    ///                    pixels
    /// * `data`         - The data to write into the frame buffer
    fn set_buffer(&mut self, byte_offset: usize, data: &[u8]);

    // Set a row of pixels in the frame buffer. This might make it easier to render if you're just
    // trying to fill out a row at a time. Please read the documentation for [`Self::set_buffer`]
    // to understand the pixel format and colour codes, but TL;DR, each byte contains two pixels,
    // and each nibble represents a single 4-bit colour code.
    //
    // # Arguments
    // * `idx`: The row index to set, starting at 0 for the top row, up to [`HEIGHT`] - 1
    // * `data`: An array of bytes representing the pixel data for the row
    // * `start_px`: The starting pixel index to write from the data buffer
    // * `end_px`: The ending pixel index to write from the data buffer (exclusive)
    fn set_pixel_row(
        &mut self,
        row_idx: u16,
        data: [u8; ROW_BYTES as usize],
        start_px: u16,
        end_px: u16,
    );

    /// Clears the display to solid white. Called automatically on initialisation.
    fn clear(&mut self);
}
