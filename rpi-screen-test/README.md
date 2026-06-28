# rpi-screen-test

Drives a [Waveshare 7.3inch E Ink Spectra 6 (E6)](https://www.waveshare.com/wiki/7.3inch_e-Paper_HAT_(E)_Manual)
display (800×480, SPD1656 controller) from a Raspberry Pi Zero via SPI, using the
`eink-spectra6-driver` crate.

Tested on the [RPi Zero PhotoPainter](https://www.waveshare.com/wiki/RPi_Zero_PhotoPainter).

## Raspberry Pi configuration

### 1. Free GPIO 8 from the SPI kernel driver

With the default `dtparam=spi=on`, the kernel claims GPIO 8 (CE0) and GPIO 7 (CE1) as
hardware chip-selects. The driver controls CS in software, so this conflicts. Add the
following line to `/boot/firmware/config.txt` **after** the `dtparam=spi=on` line to
reassign CE0 to an unused GPIO (26), freeing GPIO 8 for manual control:

```
dtoverlay=spi0-1cs,cs0_pin=26
```

Reboot after making this change.

### 2. Enable the onboard RTC (RPi Zero PhotoPainter only)

The PhotoPainter board has a DS3231 RTC chip. To enable it, add the following to the
end of `/boot/firmware/config.txt`:

```
dtoverlay=i2c-rtc,ds3231
```

Then reboot. This is only needed if you intend to use the RTC for scheduled wake-ups or
timekeeping; it has no effect on display operation.

### 3. Enable SPI

SPI must be enabled (`dtparam=spi=on` in `/boot/firmware/config.txt`). You can verify
the device is present with:

```bash
ls /dev/spi*
# expected: /dev/spidev0.0
```

## Pin wiring (BCM numbering)

| Signal | BCM GPIO | Physical pin |
|--------|----------|--------------|
| MOSI   | 10       | 19           |
| SCLK   | 11       | 23           |
| CS     | 8        | 24           |
| DC     | 25       | 22           |
| RST    | 17       | 11           |
| BUSY   | 24       | 18           |
| PWR    | 27       | 13           |
