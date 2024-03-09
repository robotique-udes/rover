# Can Setup

## Udev Rules

To automatically setup the USB to CAN adapters the rover is using each time they are connected to your PC, follow these steps:

1. Open a text editor in the udev folder

   ```Bash
   sudo nano /etc/udev/rules.d/99-can-adapter.rules
   ```

2. Add the following lines to the file

   ```udev
   SUBSYSTEM=="usb", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="606f", RUN+="/bin/bash -c 'sudo ip link set can0 up txqueuelen 1000 restart-ms 1 type can bitrate 500000'"
   ```

3. Save and exit

   ```keyboard
   -> ctrl+x
   -> y
   -> enter
   ```

4. Restart the udev server

   ```Bash
   sudo udevadm control --reload-rules
   ```

## Debugging

Wireshark is a great debugging tool, it allows the user to see every msg on the canbus network and record them

To install:

```bash
sudo apt update
sudo apt install wireshark
```

To launch:

```Bash
sudo wireshark
```
