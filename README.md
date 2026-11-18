# WheelLeg — Full-Stack Telemetry for a Wheel-Legged Robot

**WheelLeg** is a wheel-legged robot with two drive wheels and two 5-bar parallel legs, powered by **6 DMM (达妙) motors** (4× DM4310 joint motors + 2× DM6215 hub motors). This repository implements the complete telemetry pipeline:

**STM32 firmware sensor capture → CAN3 multi-frame protocol → RDK X5 ROS 2 nodes → Web real-time dashboard**

## Architecture

The legacy UART link was replaced with a unified **CAN3 (FDCAN3, classic CAN @ 1 Mbps)** as the main communication channel between the STM32 and the RDK host. The motor buses (FDCAN1/2) are physically isolated, so there is zero bus conflict.

```
┌──────────────┐   FDCAN1/2 (1Mbps)   ┌─────────────────┐
│  STM32H723   │◄────────────────────►│ 6× DMM motors   │
│  DM-MC02     │                      │ DM4310/DM6215   │
└──────┬───────┘                      └─────────────────┘
       │ CAN3 (classic CAN 1Mbps, ID 0x700 status / 0x701 cmd)
┌──────▼───────┐   ROS 2 Humble       ┌─────────────────┐
│  RDK X5      │◄────────────────────►│ Web dashboard   │
│  mcu_telemetry_node                 │ :8080 WS        │
└──────────────┘                      └─────────────────┘
```

## Repository layout

| Path | Contents |
|------|----------|
| [`WheelLeg-FW/`](WheelLeg-FW/) | STM32 firmware (Keil MDK, STM32H723): motor control, CAN3 multi-frame TX, BMI088 IMU + Mahony, battery protection, WS2812 LED, buzzer, CAN bus-off self-healing |
| [`wheel_leg_ws/`](wheel_leg_ws/) | ROS 2 Humble workspace: `rl_policy` (CAN bridge + telemetry + web nodes), `wheel_leg_bringup`, `wheel_leg_control` (state machine), `wheel_leg_msgs`, `wheel_leg_vision` |
| [`docs/版本记录.md`](docs/版本记录.md) | Version history: 18 firmware + 11 host revisions, key milestones, rollback guide |

## Hardware

| Component | Model | Notes |
|-----------|-------|-------|
| MCU | DMM DM-MC02 (STM32H723) | 480 MHz, 3× FDCAN |
| Joint motors | 4× DM4310 | 5-bar legs, CAN ID 6/8 (fb 3/4) |
| Hub motors | 2× DM6215 | drive wheels, CAN ID 1 (fb 0) |
| IMU | BMI088 | SPI2: PB13/PC1/PC2, CS: PC0/PC3 |
| Battery | 6S LiPo | 22.2 V nominal / 25.2 V full |
| Voltage sense | ADC1_CH4 (PC04) | 11:1 divider |
| LED | WS2812 RGB | SPI6 (PA5/PA7), CAN link status |
| Host | D-Robotics RDK X5 | Ubuntu 22.04 + ROS 2 Humble |

### CAN topology

```
FDCAN1 (PD0/PD1) → left lower/upper joint (ID6/8) + left hub (ID1)
FDCAN2 (PB5/PB6) → right lower/upper joint (ID6/8) + right hub (ID1)
FDCAN3 (PD12/PD13) → RDK can0 (classic CAN 1Mbps, ID 0x700 status / 0x701 cmd)
```

## CAN3 multi-frame protocol

Each telemetry packet is split into 8-byte classic CAN frames: `[Type(1B)][Flags(1B)][Data0..5]`, CAN ID `0x700`. `Flags: bit0=last, bits1-7=seq`.

| Type | Value | Total | Frames | Content | Rate |
|------|-------|-------|--------|---------|------|
| TYPE_IMU | 0x82 | 40 B | 7 | roll, pitch, yaw, gyro[3], accel[3] | 25 Hz |
| TYPE_MOTOR | 0x83 | 48 B | 8 | 4 joints × (pos, vel, tor) | 50 Hz |
| TYPE_MOTOR_TEMP | 0x84 | 32 B | 6 | 4 joints Tmos+Tcoil | 1 Hz |
| TYPE_POWER | 0x85 | 12 B | 2 | bat_voltage, imu_temp, protect | 2 Hz |
| TYPE_WHEEL | 0x86 | 24 B | 4 | 2 hubs × (pos, vel, tor) | 20 Hz |
| TYPE_WHEEL_TEMP | 0x87 | 16 B | 3 | 2 hubs Tmos+Tcoil | 1 Hz |

## ROS 2 topics (mcu_telemetry_node)

**Publishes:** `/mcu/imu` (sensor_msgs/Imu), `/mcu/joint_states` (JointState), `/mcu/motor_temp`, `/mcu/power`, `/mcu/wheel_states` (Float32MultiArray), `/odom` (placeholder).
**Subscribes:** `/cmd_vel`, `/mcu/torque_cmd`, `/mcu/pos_cmd`, `/mcu/enable`.
**Transport:** `can` (default) / `uart`, `can_interface=can0`.

The Web dashboard (`telemetry_web_node.py`, port 8080) is pure Python stdlib — no extra dependencies — and streams IMU attitude, battery voltage, joint and wheel states, and the four joint motors’ MOS and coil temperatures over WebSocket every 100 ms. The dashboard marks missing and stale samples and reconnects automatically.

## Key design decisions

1. **CAN multi-frame protocol** — 8-byte classic CAN carries >6 B payloads (`[Type][Flags][6B data]`), avoiding CAN FD Message RAM conflicts.
2. **FIFO congestion avoidance** — staggered TX periods per frame type (motor 20 ms / imu 40 ms / wheel 50 ms / temp 1000 ms / power 500 ms+250 ms phase offset), TX FIFO expanded to 16 slots.
3. **LED link detection** — uses `HAL_FDCAN_GetProtocolStatus` BusOff flag + `GetErrorCounters` TxErrorCnt, not the `AddMessage` return value.
4. **Bus-off self-healing** — auto Stop→Start FDCAN3 restart on bus drop (3 s period).
5. **IMU port** — BMI088 via SPI2 + Mahony, roll/pitch mapped consistently with the legacy build.
6. **Web concurrency** — ws_clients thread lock + defensive removal to avoid multi-thread ValueError.

## Deploy & verify

```bash
# On the RDK, with the repository workspace checked out under ~/wheel_leg_ws
source /opt/ros/humble/setup.bash
cd ~/wheel_leg_ws
colcon build --packages-up-to rl_policy
source install/setup.bash
sudo ip link set can0 up type can bitrate 1000000
ros2 launch rl_policy mcu_telemetry.launch.py transport:=can can_interface:=can0

# In a second RDK shell
source /opt/ros/humble/setup.bash
source ~/wheel_leg_ws/install/setup.bash
ros2 run rl_policy telemetry_web_node.py
# Browser: http://<RDK_IP>:8080
```

Firmware is built and flashed with Keil MDK (`WheelLeg-FW/MDK-ARM/CtrBoard-H7_FDCAN.uvprojx`, 0 Error / 1 harmless warning, ~30 KB code).

## License

MIT
