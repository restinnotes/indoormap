# SCS BLE Protocol Reference

This document describes the Bluetooth LE protocol used by the SCS sensors, extracted from the `scs-ur3-mujoco` Python codebase.

## UUIDs
*   **Service UUID**: `6E400001-B5A3-F393-E0A9-E50E24DCCA9E` (Nordic UART)
*   **Write Characteristic**: `6E400002-B5A3-F393-E0A9-E50E24DCCA9E`
*   **Read (Notify) Characteristic**: `6E400003-B5A3-F393-E0A9-E50E24DCCA9E`

## Commands (Write)

### Start Streaming (Quaternion)
To start streaming 50Hz Quaternion data:
**Command Bytes**: `0x19 0x0C 0x00 0x00 0x00 0x00 0x00 0x32 0xF0 0x00 0x00`

*   `0x19 0x0C`: OpCode for Control
*   `0x00` (x5): Logging/Feedback flags (Disabled)
*   `0x32`: Frequency (50 Hz)
*   `0xF0`: Activity ID (GRV / Quaternion)
*   `0x00`: Mock Data (0=Real, 1=Mock)
*   `0x00`: Padding

### Start Streaming (Raw Data)
To start streaming 50Hz Raw Accelerometer/Gyro:
**Command Bytes**: `0x19 0x0C 0x00 0x00 0x00 0x00 0x00 0x32 0x00 0x00 0x00`

*   Change Activity ID `0xF0` to `0x00` (Raw Data).

## Data Parsing (Read/Notify)

All data comes in Little Endian format.

### Quaternion Packet (Type 131 / 0x83)
Total Length: 15 bytes

| Offset | Field | Type | Scale Factor | Note |
| :--- | :--- | :--- | :--- | :--- |
| 0 | Type | `uint8` | - | Value is 131 (0x83) |
| 1 | Reserved | `uint8` | - | - |
| 2 | Index | `uint8` | - | Sensor Index |
| 3 | Timestamp | `uint16` | 1 ms | Time in ms |
| 5 | Qx | `int16` | 1/16384.0 | Quaternion X |
| 7 | Qy | `int16` | 1/16384.0 | Quaternion Y |
| 9 | Qz | `int16` | 1/16384.0 | Quaternion Z |
| 11 | Qw | `int16` | 1/16384.0 | Quaternion W |
| 13 | Accuracy | `uint16` | 1/16384.0 | Radian accuracy |

### Raw Data Packet (Type 125 / 0x7D)
Total Length: varies (12 + 12 per sensor)

| Offset | Field | Type | Scale Factor |
| :--- | :--- | :--- | :--- |
| 0 | Type | `uint8` | - |
| 2 | Timestamp | `uint32` | 1 ms |
| 6 | Ax | `int16` | 16384 * 2 / Range |
| 8 | Ay | `int16` | ... |
| 10 | Az | `int16` | ... |
| 12 | Gx | `int16` | 16384 * 2 / 4096 |
| 14 | Gy | `int16` | ... |
| 16 | Gz | `int16` | ... |

*Note: Raw data scaling depends on the dynamic range set in FW. Default for Gyro appears to be 4096.*
