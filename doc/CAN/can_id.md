# Rover can IDs

- [Rover can IDs](#rover-can-ids)
  - [Summary](#summary)
  - [ID ranges](#id-ranges)
  - [ID list](#id-list)
    - [Guidelines](#guidelines)
    - [list](#list)

## Summary

The rover uses CAN2.0A. This standard use 11 bits long IDs. They range from 0 to 2047 or from 0x00 to 0x7FF in hex. The lower the id the higher it's priority. Each id can represent either a msg type or device/module on the rover.

## ID ranges

| RANGE | Type                              |
|-------|-----------------------------------|
| 0x0** | Reserved for Master               |
| 0x1** | Propulsion                        |
| 0x2** | Arm                               |
| 0x3** | Science                           |
| 0x4** | Accessory (lights, speakers, etc) |
| 0x5** | Free Space                        |

## ID list

### Guidelines

- Add your can device ID to the list as soon as possible
- Don't use range delimiter as ID
- Leave space for ids where you think a module could fit there in the future
  - Example: A MiddleLeft motor could be added in the future an empty space is left between FrontRight and RearLeft  

### list

| ID        | Device or Message                 |
|-----------|-----------------------------------|
|           |                                   |
| **0x000** | RESERVED FOR MASTER               |
| 0x001     | - Heartbeat (msg)                 |
| 0x020     | - Master Computer unit            |
| 0x021     | - GPS (msg)                       |
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
| 0x401     | - Main Lights                     |
| 0x410     | - Speakers                        |
|           |                                   |
| **0x500** | Free Space                        |
|           |                                   |
