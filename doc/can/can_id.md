# Rover can IDs

- [Rover can IDs](#rover-can-ids)
  - [Summary](#summary)
    - [RoverCanProtocol Syntaxt](#rovercanprotocol-syntaxt)
  - [ID ranges](#id-ranges)
  - [Device and message ID list](#device-and-message-id-list)
    - [Guidelines](#guidelines)
    - [List of devices ID](#list-of-devices-id)
    - [List of msgs ID](#list-of-msgs-id)
  - [Message Content Id List](#message-content-id-list)
    - [ErrorState](#errorstate)
    - [Heartbeat](#heartbeat)
    - [GPS](#gps)
    - [PropulsionMotor](#propulsionmotor)

## Summary

The rover uses CAN2.0A. This standard use 11 bits long IDs. They range from 0 to 2047 or from 0x00 to 0x7FF in hex. The lower the id the higher it's priority. Each id can represent either a msg type or device/module on the rover. When sending a message on the can bus, you'll set an ID for the message and 8 bytes of data. The [RoverCanProtocol Syntaxt](#rovercanprotocol-syntaxt) section explain how those 8 bytes are used.

### RoverCanProtocol Syntaxt

| bytes index | 0x00  | 0x01           | 0x02    | 0x03 | 0x04 | 0x05 | 0x06 | 0x07 |
|-------------|-------|----------------|---------|------|------|------|------|------|
| data        | msgID | Msg Content Id | data... | ...  | ...  | ...  | ...  | ...  |

## ID ranges

| RANGE | Type                              |
|-------|-----------------------------------|
| 0x0** | Reserved for Master               |
| 0x1** | Propulsion                        |
| 0x2** | Arm                               |
| 0x3** | Science                           |
| 0x4** | Accessory (lights, speakers, etc) |
| 0x5** | Free Space                        |

## Device and message ID list

### Guidelines

- Add your can device ID to the list as soon as possible
- Don't use range delimiter as ID
- Leave space for ids where you think a module could fit there in the future
  - Example: A MiddleLeft and a MiddleRight motor could be added in the future so empty space is left between FrontRight(0x102) and RearLeft(0x105)
- If you change any already defined ID, you'll have to reupload into each microcontroller. (Basically don't)

### List of devices ID

Devices are organized into a specific order in relation to arbitration.

| ID        | Devices                           |
|-----------|-----------------------------------|
|           |                                   |
| **0x000** | RESERVED FOR MASTER               |
| 0x020     | - Master Computer unit            |
| 0x021     | - BMS                             |
|           |                                   |
| **0x100** | PROPULSION                        |
| 0x101     | - FrontLeft motor                 |
| 0x102     | - FrontRight motor                |
| 0x103     |                                   |
| 0x104     |                                   |
| 0x105     | - RearLeft motor                  |
| 0x106     | - RearRight motor                 |
|           |                                   |
| **0x200** | ARM                               |
| 0x210     | - J0 Controller                   |
| 0x211     | - J1 Controller                   |
| 0x212     | - J2 Controller                   |
| 0x213     | - J3 Controller                   |
| 0x214     | - J4 Controller                   |
| 0x215     | - J5 Controller                   |
| 0x216     | - J6 Controller                   |
|           |                                   |
| **0x300** | Science                           |
|           |                                   |
| **0x400** | Accessory (lights, speakers, etc) |
| 0x401     | - GPS                             |
| 0x402     | - Lights                          |
| 0x402     | - Infrared lights                 |
| 0x410     | - Speakers                        |
|           |                                   |
| **0x500** | Free Space                        |
|           |                                   |

### List of msgs ID

Msgs Ids aren't organised as their order don't matter in the arbitration, just add new messages types at the end of the list.

| ID   | Msgs             |
|------|------------------|
| 0x00 | NOT_USED         |
| 0x01 | ERROR_STATE      |
| 0x02 | HEARTBEAT (msg)  |
| 0x10 | GPS (msg)        |
| 0x11 | PROPULSION_MOTOR |

## Message Content Id List

In the following section are defined all the message content internal IDs, remember, if you change any already defined ID, you'll have to reupload into each microcontroller.

### ErrorState

| ID   | Device or Message | TYPE |
|------|-------------------|------|
| 0x00 | NOT_USED          | n/a  |
| 0x01 | ERROR             | bool |
| 0x02 | WARNING           | bool |

### Heartbeat

| ID   | Device or Message | TYPE |
|------|-------------------|------|
| 0x00 | NOT_USED          |      |
| 0x01 | DONT_USE          |      |
| TODO | TODO              |      |

### GPS

| ID   | Device or Message | TYPE |
|------|-------------------|------|
| 0x00 | NOT_USED          |      |
| TODO | TODO              |      |

### PropulsionMotor

| ID   | Device or Message | TYPE    |
|------|-------------------|---------|
| 0x00 | NOT_USED          |         |
| 0x01 | enable            | bool    |
| 0x02 | targetSpeed       | float32 |
| 0x03 | currentSpeed      | float32 |
| 0x04 | kp                | float32 |
| 0x05 | ki                | float32 |
| 0x06 | kd                | float32 |
| 0x07 | closeLoop         | bool    |
