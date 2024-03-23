# slcan-udev-rules

udev rules and systemd service to automatically configure and bring up/down usb slcan interfaces.

## Installation

```bash
sudo cp 90-usb-slcan.rules /etc/udev/rules.d/
sudo cp slcan_add.sh /usr/local/bin/
sudo cp slcan@.service /etc/systemd/system/
```
