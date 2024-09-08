# How to use rover canbus (canRovus)

If you haven't configure your linux computer for canbus yet, checkout [this guide](can_setup.md).

## Using the can bus

Our can network works with a "master/slave" model.

### The Master

The master is a single ROS Node running on the rover's pc (called the main computer unit or MCU). This node makes a bridge between ROS topics and services and our can network. To start it, you need a USB to CAN interface for your computer like the [canable pro 1.0](https://canable.io/) or any other adapters with the *slcan* firmware ([see can documentation](https://canable.io/getting-started.html)). (If you bought a device no one used before, you'll need to add it to [the automatic connection script](../../rover_can/scripts/can_configuration/90-usb-slcan.rules))

Launching the master node:

```bash
ros2 launch rover_can can.launch.py
```

### The slaves

Slaves node are all other Can devices on the network, they can be motors, sensors, accessories... They are always alive when connected to power and will send packet according to their respective functionalities. You don't need to launch them, if they have power they are writing and reading.

### Error feedback

The master will ask each registered slave node for their error state when starting. This can be monitored on the ROS topic */rover/can/device_status*. Nodes can also report state on this topic if programed to do so. In the future, from the GUI, you'll be able to ask each devices their state and debug the ones reporting errors or warning.

When a node falls into an error state (either warning or error), it needs to log into its serial port the source of the error.

### Can messages protocol

The rover is using the can transport protocol with our own communication layer on top.

#### Summary

The rover uses CAN2.0A. This standard use 11 bits long IDs. They range from 0 to 2047 or from 0x00 to 0x7FF in hex. The lower the id the higher it's priority. Each id can represent either a msg type or device/module on the rover. When sending a message on the can bus, you'll set an ID for the message and 8 bytes of data. The [RoverCanProtocol Syntax](#rovercanprotocol-syntax) section explain how those 8 bytes are used.

#### RoverCanProtocol Syntax

| bytes index | 0x00  | 0x01           | 0x02    | 0x03 | 0x04 | 0x05 | 0x06 | 0x07 |
|-------------|-------|----------------|---------|------|------|------|------|------|
| field       | msgID | Msg Content Id | data... | ...  | ...  | ...  | ...  | ...  |

#### Ids

All IDs are defined inside the [can_id.md](can_id.md) file.

#### How to read a can msg

With the help of the candump command I captured this packet:

```bash
 canRovus  101   [6]  11 02 00 00 8A 42
```

- canRovus: This is the name of the network
- 0x101: This is the id of the device (hexadecimal), according to the [can_id.md](can_id.md) file this device is the "FrontLeft motor".
- [6]: The number of byte in the message
- 0x11: This is the message type ID, according to [can_id.md](can_id.md) this is a "PROPULSION_MOTOR"
- 02: This is the message content ID, according to [can_id.md](can_id.md) this is a TARGET_SPEED command
- 0x00008A42: After the message content ID this is the actual data, in the case TARGET_SPEED is a float and so, casting 0x00'00'8A'42 in float will equal 69.0.

In summary this message tells the FrontLeft motor to set it's target speed at 69.0

## Software development on the canbus

### How to create new devices

#### How to add new slaves

TODO

#### How to modify the Master to work with new slaves and link them with ROS topics

TODO

### How to create new rover_can_lib message types

TODO
