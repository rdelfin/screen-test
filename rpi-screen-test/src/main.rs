use std::time::Duration;

use eink_spectra6_driver::EpaperDisplay;
use eink_spectra6_driver::device::Spectra6SpiDriver;
use gpio_cdev::{Chip, LineRequestFlags};
use linux_embedded_hal::spidev::{SpiModeFlags, SpidevOptions};
use linux_embedded_hal::{CdevPin, Delay, SpidevBus};

fn setup_tracing() {
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::INFO)
        .init();
}

fn main() -> anyhow::Result<()> {
    setup_tracing();

    let mut chip = Chip::new("/dev/gpiochip0")?;
    let _pwr = chip
        .get_line(27)?
        .request(LineRequestFlags::OUTPUT, 1, "pwr")?;

    let dc = CdevPin::new(
        chip.get_line(25)?
            .request(LineRequestFlags::OUTPUT, 0, "dc")?,
    )?;
    let cs = CdevPin::new(
        chip.get_line(8)?
            .request(LineRequestFlags::OUTPUT, 1, "cs")?,
    )?;
    let rst = CdevPin::new(
        chip.get_line(17)?
            .request(LineRequestFlags::OUTPUT, 1, "rst")?,
    )?;
    let busy = CdevPin::new(
        chip.get_line(24)?
            .request(LineRequestFlags::INPUT, 0, "busy")?,
    )?;

    let mut spi = SpidevBus::open("/dev/spidev0.0")?;
    spi.configure(
        &SpidevOptions::new()
            .bits_per_word(8)
            .max_speed_hz(4_000_000) // Waveshare reference uses 4 MHz
            .mode(SpiModeFlags::SPI_MODE_0 | SpiModeFlags::SPI_NO_CS)
            .build(),
    )?;
    let mut epaper: EpaperDisplay<_, _, _, _, _, _> =
        EpaperDisplay::new(Spectra6SpiDriver::new(spi, dc, cs, rst, busy, Delay));

    tracing::info!("Printing double rectangles to the screen");
    epaper.fill_rect(100, 100, 200, 200, 0x6);
    epaper.fill_rect(150, 150, 200, 200, 0x5);
    epaper.render();

    std::thread::sleep(Duration::from_secs(5));

    tracing::info!("Printing checkerboard to the screen");
    epaper.display_checkerboard();
    epaper.render();

    std::thread::sleep(Duration::from_secs(5));

    tracing::info!("Printing sample image to the screen");
    epaper.display_sample();
    epaper.render();
    Ok(())
}
