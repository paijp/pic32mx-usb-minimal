# PIC32MX USB minimal samples

Minimal single-file USB examples for the Microchip PIC32MX270F256B,
written in C90 with no USB library. Three working samples are
provided:

- [`keyboard/`](keyboard/) - a USB device: HID boot keyboard that
  sends the letter `a` once per second
- [`cdc/`](cdc/) - a USB device: CDC-ACM serial device that appears
  as `/dev/ttyACM0` on Linux and echoes received bytes back in upper
  case
- [`host/`](host/) - a USB host: enumerates a full-speed USB printer
  (class 0x07) or a low-speed HID boot keyboard (class 0x03)

Each sample is a single `.c` file with no external dependencies beyond
the XC32 toolchain.

## File naming

The digits at the end of each source filename (e.g. `kb0007.c`,
`cdc0007.c`, `usbhost0014.c`) are build numbers. They are incremented
each time the file is revised so that older and newer versions can
coexist. The newest build number in each directory is the current
version.

## Hardware

- PIC32MX270F256B (28-pin DIP/SOIC/SSOP), 4 MHz external crystal
- VUSB3V3 decoupled with 10 nF + 4.7 uF to GND
- D+/D- routed to a USB connector
- PGED/PGEC brought out for programming (PICkit 3/4, ICD 3/4, SNAP)

The host sample additionally requires VBUS (5 V) to be supplied to the
connected device; see [`host/README.md`](host/README.md) for details.

Clock configuration used by all samples:

- 4 MHz crystal -> SYSCLK 40 MHz, PBCLK 40 MHz
- Separate USB PLL -> 48 MHz for the USB module

## Building

Any recent XC32 (MPLAB X) toolchain will do. From the command line, in
any subdirectory:

```
xc32-gcc -mprocessor=32MX270F256B -std=gnu90 -O1 \
    -o out.elf kb0007.c
xc32-bin2hex out.elf
```

Flash the resulting `.hex` with MPLAB IPE or your programmer of choice.

The Dockerfile at https://github.com/paijp/mplabx can be used to build
in a containerized environment.

## License

Apache License 2.0. See [LICENSE](LICENSE) for the full text.

## Acknowledgements

Two open-source USB stacks for PIC microcontrollers were used as
architectural references during development. No code from either
project is included in this repository, but the solutions to several
PIC32MX-specific issues (in particular the requirement to use full
ping-pong buffering on this peripheral, the correct ordering of
`U1PWRC` during power-on initialisation, and - for the host sample -
the bus reset timing, ping-pong management, and the need to re-apply
`U1EP0`/`U1ADDR` before every token) came from reading these sources:

- [MicrochipTech/mla_usb](https://github.com/MicrochipTech/mla_usb)
  (Apache 2.0)
- [signal11/m-stack](https://github.com/signal11/m-stack)
  (Apache 2.0 / GPL-3.0 / LGPL-3.0)

Thanks to the authors and maintainers of both projects.

## Authors

The code in this repository was developed by paijp in collaboration
with Anthropic's Claude (Claude Opus 4.6). The requirements, hardware
bring-up, oscilloscope debugging, and final design decisions were made
by paijp; Claude drafted and iterated on the source code based on that
feedback. All testing was performed on real hardware.
