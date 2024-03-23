# Can Setup

- [Can Setup](#can-setup)
  - [Udev Rules](#udev-rules)
  - [Debugging](#debugging)
    - [Wireshark](#wireshark)
    - [can-utils](#can-utils)

## Udev Rules

To automatically setup the USB to CAN adapters the rover is using each time they are connected to your PC, follow these steps*:

*A simple Udev rules won't init our canable pro 1.0 adapter so we have to create a service.

1. Navigates to the /can_configuration folder inside of the rover_can/script package.

   ```Bash
   cd ~/ros2_ws/src/rover/rover_can/scripts/can_configuration
   ```

2. Run theses commands to copy the necessary file into your computer

   ```udev
   sudo cp 90-usb-slcan.rules /etc/udev/rules.d/
   sudo cp slcan_add.sh /usr/local/bin/
   sudo cp slcan@.service /etc/systemd/system/
   ```

3. Restart the udev server

   ```Bash
   sudo udevadm control --reload-rules
   ```

4. Unconnect and reconnect your device to your computer.

5. If everything is setup correctly, you should see a "canRovus" network adapter when your USB-to-CAN adapter is plugged in. You can run the following command to confirm

   ```Bash
   ifconfig canRovus
   ```

   Expected output when adapter is connected:

   ```Bash
   canRovus: flags=193<UP,RUNNING,NOARP>  mtu 16
        unspec 00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00  txqueuelen 1000  (UNSPEC)
        RX packets ...
   ```

   Expected output with wrong configuration

   ```bash
   canRovus: error fetching interface information: Device not found
   ```

## Debugging

### Wireshark

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

### can-utils

can-utils include basic tools needed for debugging and setuping CAN devices.

To install:

```bash
sudo apt install can-utils
```

candump is a terminal tool that you can use to see live packet on the canbus network.

Usage:

```bash
candump canRovus #candump <canNetworkName>
```

cansend is a terminal tool to send message over the can network for testing purposes

Usage:

```bash
cansend canRovus 001#2345678910ABCDEF #cansend <canNetworkName> <canId>#<canData (max 8 bytes)> 
```
