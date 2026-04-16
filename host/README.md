# USB host sample

A minimal USB host for PIC32MX270F256B.  On each attach the host
enumerates the connected device and dispatches to one of two
built-in drivers based on the interface class reported in the
configuration descriptor:

- **Full-speed USB printer** (class `0x07`) – enumerates the device,
  selects the configuration, and holds the bulk OUT endpoint ready
  for the application to push print data.  The sample itself only
  performs enumeration; the bulk-OUT transfer helper is left for the
  application to drive.
- **Low-speed HID boot keyboard** (class `0x03`, subclass `0x01`,
  protocol `0x01`) – enumerates the device, issues
  `SET_PROTOCOL(Boot)`, then polls the interrupt IN endpoint every
  ~10 ms and converts new key-presses to ASCII via a small built-in
  `keycode_to_char()` table.

Any other device class is accepted onto the bus but left idle until
detach.

The digits at the end of the source filename are a build number.  The
highest number is the current version.

## Background task hook

All busy-wait loops in the driver (delays, attach/detach waits, token
completion waits) call the function pointer `usb_polltask` on every
iteration when it is non-NULL.  This lets the application perform
background work – for example driving a display, sampling sensors, or
feeding a watchdog – while the USB stack is waiting for a hardware
event.

```c
extern void (*usb_polltask)(void);

/* Example: blink an LED and feed the watchdog */
static void my_background(void)
{
    led_update();
    wdt_feed();
}

int main(void)
{
    usb_polltask = my_background;
    usb_init();
    ...
}
```

The callee must return promptly; it shares the same thread as the
USB stack and must not block.

Once a device is attached and enumerated, the host does not require
any function to be called within a hard real-time deadline.  SOF
tokens (full-speed) and keep-alive pulses (low-speed) are generated
by the USB hardware automatically, so the application may take as
long as it needs between calls into the USB driver without risking a
bus timeout.

## Hardware notes

In addition to the common hardware listed in the top-level
[README](../README.md):

- **VBUS (5 V)** must be supplied to the USB connector so that the
  attached device can power up.  The PIC32MX270F256B cannot source
  VBUS itself; a simple high-side switch or a permanently-on 5 V rail
  is sufficient.
- A host-side 15 kΩ pull-down on D+ and D− is provided internally by
  the PIC32 USB module when configured as host (`U1OTGCON`), so no
  external pull-downs are required.
- Make sure VUSB3V3 still has its 10 nF + 4.7 µF decoupling; the USB
  transceiver is shared between device and host modes.

## Design notes

The driver is a single-threaded polling loop with no interrupts.
The main loop runs:

```
wait-attach → bus reset → enumerate → dispatch → wait-detach
```

Key PIC32MX-specific details that are easy to get wrong:

- Only four BDT entries are used (IN/OUT × EVEN/ODD).  In host mode
  the endpoint number is selected per transaction through `U1TOK`, so
  the BDT does not need to be indexed by endpoint.
- `U1EP0` and `U1ADDR` must be re-applied before every token.  The
  driver does this inside `token_send()`.
- `RETRYDIS` is cleared for control transfers (hardware retries per
  the USB spec) and set for bulk/interrupt transfers so that NAKs
  surface immediately and can be retried in software.
- Full ping-pong buffering is used and tracked explicitly via
  `g_pp_in` / `g_pp_out`; the `PPBRST` bit is toggled at the start
  of every session via `reset_ping_pong()`.

## Application hook

Both branches of the dispatcher are written so the application can
drop in its own handling:

- Printer branch: replace the idle wait with calls to
  `usb_bulk_write()`.  A sample call is shown in a comment at the
  dispatch site in `main()`.
- Keyboard branch: the comment `/* Application hook */` marks the
  point where a decoded ASCII character is available; forward it to
  a UART, a ring buffer, or wherever the application needs it.

## Coding style

The source follows QEMU coding conventions (every indented statement
is braced, opening brace on the same line as the control statement,
`lower_case_with_underscores` for variables and functions, `CamelCase`
for type names, declarations at the top of each block) with one
project-local deviation: indentation uses one tab character per level
instead of four spaces.  The code targets C90.

## License

Apache License 2.0.  See [../LICENSE](../LICENSE).
