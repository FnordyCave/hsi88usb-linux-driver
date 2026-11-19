# Linux HSI-88-USB driver

This kernel module binds **only interface 1** of USB VID:PID `1bbf:0064`, the HSI-88-USB function. It does not bind interface 0 (`DiCoStation-USB`).

It exposes `/dev/hsi88usbN` and accepts the same documented HSI command stream used by the Windows API:

- binary mode (default): `s <left> <middle> <right> \r`, `m\r`, `v\r`, `t\r`
- terminal mode: after `t\r`, numerical values are sent/received as two ASCII hex digits
- feedback changes arrive asynchronously as `i <count> <module> <HB> <LB> ... \r`

## Build

Install your distribution's kernel headers/build tools, then:

```sh
make
sudo insmod hsi88usb.ko
```

Check:

```sh
dmesg | tail
ls -l /dev/hsi88usb*
```

For automatic permissions, copy `99-hsi88usb.rules` to `/etc/udev/rules.d/`, adjust its group if necessary, run `sudo udevadm control --reload-rules`, then reconnect the device.

Unload with:

```sh
sudo rmmod hsi88usb
```

## Example binary command

With one module on each bus line, the application-level registration bytes are:

```text
73 01 01 01 0d
```

(`73` is ASCII `s`). The driver translates this to the captured USB OUT packet `05 81 01 01 01`.

## Notes

- `poll(2)`/`select(2)` work for asynchronous feedback.
- The module maintains parser state across writes, so applications may write a command in pieces.
