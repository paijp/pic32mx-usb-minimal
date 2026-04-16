# CDC-ACM sample

A minimal USB CDC-ACM serial device for PIC32MX270F256B. Appears as
`/dev/ttyACM0` on Linux and echoes received bytes back in upper case.

The digits at the end of the source filename are a build number. The
highest number is the current version.

## Usage

Plug the device in. `dmesg` should show something like:

```
cdc_acm 1-1:1.0: ttyACM0: USB ACM device
```

Then try it with any terminal program at any baud rate (the device
ignores the line coding):

```
picocom /dev/ttyACM0 -b 9600
```

Type characters. Lower-case letters come back upper-cased; other bytes
pass through unchanged.

## API

The driver exposes five functions to the application:

```c
void     cdc_init(void);
void     cdc_poll(void);
uint8_t  cdc_is_configured(void);
uint16_t cdc_recv(uint8_t *buf, uint16_t max);
uint16_t cdc_send(const uint8_t *buf, uint16_t len);
```

- `cdc_poll()` must be called regularly from `main()`; it drives the
  USB state machine without interrupts.
- `cdc_recv()` returns the size of the next received packet.
  - If `max < n` (the packet is bigger than the caller is willing to
    accept), `buf` is not touched and the packet stays, so the caller
    can retry later with a larger buffer.
  - If `max >= n` and `buf != NULL`, the packet is copied into `buf`
    and the OUT buffer is released back to the host.
  - If `max >= n` and `buf == NULL`, the packet is discarded (the OUT
    buffer is released without copying). Useful for dropping an
    unwanted packet.
  - Returns 0 if no packet is waiting.
- `cdc_send()` transmits up to 64 bytes in one call and returns the
  number of bytes actually sent, or 0 if the IN buffer is busy. If
  more than 64 bytes are offered, only the first 64 are sent.
  - If `len == 0`, no data is transmitted. Instead, the function
    returns the number of bytes that could be accepted right now
    (0 if the IN buffer is busy, 64 otherwise). This lets the caller
    check for available space before committing to a send.

The `main()` in the source file shows the expected pattern: drive
`cdc_poll()` every iteration, then try `cdc_recv` / `cdc_send`.

## Known limitations

- Bulk-only CDC, no notifications on EP1 IN
- No flow control beyond the natural NAK that arises when the OUT
  buffers are full and the application has not yet called `cdc_recv()`
- Line coding (baud rate, parity etc.) is accepted but ignored
- The VID/PID pair used (`0x04D8` / `0x000A`) is Microchip's public
  CDC sample. Do not ship a product with these values; obtain your
  own.

## License

Apache License 2.0. See [../LICENSE](../LICENSE).
