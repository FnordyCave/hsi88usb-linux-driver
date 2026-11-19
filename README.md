## Unofficial Linux driver for the HSI-88-USB
See driver.md for details. Uses same commands as the official Windows driver (See [specifications (German)](https://www.softdigital.de/fileadmin/sites/softdigital.de/downloads/hsi88usb/Schnittstellenbeschreibung_HSI-88-USB.pdf)).
Binds to /dev/hsi88usb*, starting with 1 (/dev/hsi88usb1).

### Methodology
I captured the USB traffic between the official Windows driver and the HSI88USB using Wireshark. I then provided these captures to ChatGPT, which analysed the traffic and wrote a Linux driver

## Inoffizieller Linux Treiber für HSI-88-USB
Siehe driver.md (englisch) für mehr Details. Nutzt selben Befehlssatz wie offizieller Windows-Treiber (Siehe [Schnittstellenbeschreibung](https://www.softdigital.de/fileadmin/sites/softdigital.de/downloads/hsi88usb/Schnittstellenbeschreibung_HSI-88-USB.pdf)).
Die Geräte stehen unter /dev/hsi88usb* zur Verfügung, beginnend mit 1 (/dev/hsi88usb1).
