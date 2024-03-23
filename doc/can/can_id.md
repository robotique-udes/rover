# Rover can IDs

- [Rover can IDs](#rover-can-ids)
  - [ID ranges](#id-ranges)
  - [Device and message ID list](#device-and-message-id-list)
    - [Guidelines](#guidelines)
    - [List of devices ID](#list-of-devices-id)
    - [List of msgs ID](#list-of-msgs-id)
  - [Message Content Id List](#message-content-id-list)
    - [ErrorState](#errorstate)
    - [Heartbeat](#heartbeat)
    - [GPS](#gps)
    - [PROPULSION\_MOTOR\_CMD](#propulsion_motor_cmd)
    - [PROPULSION\_MOTOR\_STATUS](#propulsion_motor_status)

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

| ID   | Msgs                    |
|------|-------------------------|
| 0x00 | NOT_USED                |
| 0x01 | ERROR_STATE             |
| 0x02 | HEARTBEAT (msg)         |
| 0x10 | GPS (msg)               |
| 0x11 | PROPULSION_MOTOR_CMD    |
| 0x12 | PROPULSION_MOTOR_STATUS |

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
| 0x01 | DONT_USE          | bool |
| TODO | TODO              |      |

### GPS

| ID   | Device or Message | TYPE |
|------|-------------------|------|
| 0x00 | NOT_USED          |      |
| TODO | TODO              |      |

### PROPULSION_MOTOR_CMD

| ID   | Device or Message | TYPE    |
|------|-------------------|---------|
| 0x00 | NOT_USED          |         |
| 0x01 | ENABLE            | bool    |
| 0x02 | TARGET_SPEED      | float32 |
| 0x03 | CLOSE_LOOP        | bool    |

### PROPULSION_MOTOR_STATUS

| ID   | Device or Message | TYPE    |
|------|-------------------|---------|
| 0x00 | NOT_USED          |         |
| 0x01 | CURRENT_SPEED     | float32 |
