# microros_ws

micro-ROS workspace for communication with microcontrollers.

## Structure

This workspace is managed inside a Docker container. The source files are built during the Docker image build process.

```
microros_ws/
├── Dockerfile          # Container definition
├── src/                # Source code (gitignored, auto-cloned)
│   └── micro_ros_setup/
├── build/              # Build artifacts (gitignored)
├── install/            # Installation files (gitignored)
└── log/                # Build logs (gitignored)
```

## 使い方

See [docs/microros-setup.md](../docs/microros-setup.md) for detailed instructions.

For STM32 Nucleo F446RE specific setup, see [docs/microros-nucleo-f446re.md](../docs/microros-nucleo-f446re.md).

### Quick Start (STM32 Nucleo F446RE)

```bash
# Build container
make build

# Start container
make up

# Access container
make shell-microros

# Run agent (Nucleo F446RE uses /dev/ttyACM0)
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```

### Firmware Build (already built in container!)

The firmware is **pre-built** in the Docker image for Nucleo F446RE with ping_pong app.

Just flash it to your board:

```bash
# Access container
make shell-microros

# Connect Nucleo board via USB, then flash
ros2 run micro_ros_setup flash_firmware.sh
```

### Rebuild Firmware (if needed)

```bash
# In container
make shell-microros

# Rebuild
cd /root/microros_ws
ros2 run micro_ros_setup build_firmware.sh

# Flash
ros2 run micro_ros_setup flash_firmware.sh
```

### Build for Different App

```bash
# In container
make shell-microros

# Reconfigure with different app
ros2 run micro_ros_setup configure_firmware.sh <app_name> --transport serial

# Build
ros2 run micro_ros_setup build_firmware.sh

# Flash
ros2 run micro_ros_setup flash_firmware.sh
```

## Note

The `src/`, `build/`, `install/`, and `log/` directories are gitignored because they are generated during the Docker build process.
