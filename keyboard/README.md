# HID boot keyboard sample

A minimal USB HID boot keyboard for PIC32MX270F256B. Once enumerated,
it sends the letter `a` once every ~1 second.

The digits at the end of the source filename are a build number. The
highest number is the current version.

## API

The driver exposes four functions to the application:

```c
void    kb_init(void);
void    kb_poll(void);
uint8_t kb_is_configured(void);
uint8_t kb_send_key(uint8_t modifier, uint8_t code);
```

- `kb_poll()` must be called regularly from `main()`; it drives the
  USB state machine without interrupts.
- `kb_send_key()` arms the next EP1 IN buffer with an 8-byte HID boot
  keyboard report and returns 1 if the report was queued, or 0 if the
  device is not yet configured or the IN buffer is still in flight.
- `modifier` is the boot keyboard modifier byte (bit 0 = left ctrl,
  bit 1 = left shift, etc.).
- `code` is the HID usage ID of the key (e.g. `0x04` for `a`). Use
  `0x00` to release all keys.

The `main()` in the source file shows the expected pattern: drive
`kb_poll()` every iteration, then alternate between a key-down and a
key-up report once per tick.

## License

Apache License 2.0. See [../LICENSE](../LICENSE).
