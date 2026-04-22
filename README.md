# IMU383 ROS2 Node

This package provides a ROS 2 (Jazzy) driver for the IMU383 sensor. It discovers the sensor on a serial port (e.g. `/dev/ttyUSB0`), configures it, reads the stream containing Scaled Sensor Data (S1) packet, and publishes it on a ROS 2 topic.

## Prerequisites

- ROS 2 Jazzy (also works on newer/older distributions with minor changes)
- Supported OS: Ubuntu 24.04 (Noble) or compatible Linux systems
- C++17 compiler

### Dependencies

Ensure the ROS 2 dependencies and `libserial-dev` are installed:
```bash
sudo apt-get update
sudo apt-get install libserial-dev ros-jazzy-rclcpp ros-jazzy-std-msgs
```

## Build

Assuming you have a standard colcon workspace (e.g. `~/ros2_ws`), navigate to the root of your workspace:

```bash
cd ~/ros2_ws
colcon build --packages-select imu383
source install/setup.bash
```

## Usage

1. **Connect the IMU383**: Plug the device into a USB port. Ensure the OS recognizes it as `/dev/ttyUSBx` or `/dev/ttyACMx`.
2. **Permissions**: The user running the node must have read/write access to the serial port.
   ```bash
   sudo usermod -aG dialout $USER
   ```
   *(You'll need to log out and back in for this to take effect)*

3. **Run the Node**: 
   ```bash
   ros2 run imu383 imu383_node
   ```

## Nodes

### `imu383_node`

This represents the primary driver for the sensor.

#### Published Topics
- `/imu383/data` (`imu383/msg/IMU383Data`)
  Contains acceleration, angular rate, and temperature readings. The values are automatically scaled to standard units (g, deg/sec, °C).

#### Services (Proposed)
- *Reserved for future implementation.*

#### Parameters
- *Reserved for future implementation.*

## Troubleshooting

- **Node hangs on "Searching for device..."**: Ensure the serial port uses supported prefixes (`/dev/ttyUSB*` / `/dev/ttyACM*`) and that permissions are properly set.
- **CRC verification failed**: This can occasionally happen due to noise on the serial line or when connecting mid-stream. The parser will automatically drop corrupted packets and recover on the next valid sync byte.
