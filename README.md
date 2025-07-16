# ROS 2 System Controller

A comprehensive ROS 2 (Humble) C++ codebase implementing a modular system controller architecture with policy management, command arbitration, vehicle/mission adapters, and telemetry collection.

## Architecture Overview

This system implements a distributed architecture with the following components:

- **System Manager Node**: Manages multiple policy types and publishes policy commands
- **Command Input Arbiter Node**: Arbitrates between policy, teleop, and autonomy commands
- **Vehicle Adapter Manager Node**: Manages vehicle types and interfaces with vehicle hardware
- **Telemetry Collector Node**: Collects and redistributes telemetry data
- **Mission Service Node**: Manages mission types and publishes mission status
- **Mission Adapter Manager Node**: Manages payload types and mission hardware

### Communication Flow

```
System Manager → Policy Commands → Command Arbiter → Vehicle Adapter → Vehicle HW
                                      ↓
Telemetry Collector ← Vehicle Status ←
       ↓
Mission Service ← Mission Commands
       ↓
Mission Adapter → Mission HW
```

## Prerequisites

### For Local Development (without Docker)
- ROS 2 Humble
- C++ compiler (GCC 9+)
- CMake 3.8+
- colcon build tools

### For Docker Development (Recommended for Mac)
- Docker Desktop
- Docker Compose
- XQuartz (for GUI applications - optional)

## Docker Setup (Recommended for Mac)

### 1. Build and Run with Docker Compose

```bash
# Clone and navigate to the repository
cd system_controller

# Build the Docker image
docker-compose build

# Start the container
docker-compose up -d ros2-system-controller

# Access the container
docker-compose exec ros2-system-controller bash
```

### 2. Alternative: Direct Docker Commands

```bash
# Build the Docker image
docker build -t ros2-system-controller .

# Run the container with source code mounted
docker run -it --rm \
  -v $(pwd)/src:/ros2_ws/src \
  ros2-system-controller
```

### 3. Multi-Architecture Support

The Docker setup supports both Apple Silicon (M1/M2) and Intel Macs:

- **Apple Silicon (M1/M2)**: Uses ARM64 base images
- **Intel Macs**: Uses AMD64 base images

**Note**: Some ROS 2 images may have limited ARM64 support. If you encounter issues on Apple Silicon, try:

```bash
# Force AMD64 architecture (may be slower on M1/M2)
docker build --platform linux/amd64 -t ros2-system-controller .
```

## Building the Code

### Inside Docker Container

```bash
# Navigate to workspace
cd /ros2_ws

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the package
colcon build --packages-select system_controller

# Source the setup
source install/setup.bash
```

### Local Build (if you have ROS 2 Humble installed)

```bash
# From the repository root
cd src
colcon build --packages-select system_controller
source install/setup.bash
```

## Running the System

### 1. Launch All Nodes

```bash
# Using launch file (recommended)
ros2 launch system_controller system_controller.launch.py

# Or launch demo with fewer nodes
ros2 launch system_controller demo.launch.py
```

### 2. Run Individual Nodes

```bash
# In separate terminals (or tmux/screen sessions)

# System Manager
ros2 run system_controller system_manager_node

# Command Arbiter
ros2 run system_controller command_input_arbiter_node

# Vehicle Adapter
ros2 run system_controller vehicle_adapter_manager_node

# Telemetry Collector
ros2 run system_controller telemetry_collector_node

# Mission Service
ros2 run system_controller mission_service_node

# Mission Adapter
ros2 run system_controller mission_adapter_manager_node
```

### 3. Testing Message Flow

Open additional terminals to test the system:

```bash
# Send teleop commands (high priority)
ros2 topic pub /Teleop/Command std_msgs/String "data: 'emergency_stop'"

# Send autonomy commands (medium priority)
ros2 topic pub /Autonomy/Command std_msgs/String "data: 'autonomous_navigation'"

# Send mission commands
ros2 topic pub /MissionSel/Command std_msgs/String "data: 'start'"

# Monitor topics
ros2 topic echo /PolicyCommand
ros2 topic echo /ArbitratedCommand
ros2 topic echo /VehicleStatus
ros2 topic echo /MissionStatus
```

### 4. Service Testing

```bash
# Test status services
ros2 service call /vehicle_adapter_status system_controller/srv/GetStatus "{component_id: 'vehicle'}"
ros2 service call /mission_service_status system_controller/srv/GetStatus "{component_id: 'mission'}"
```

## Monitoring and Debugging

### View Active Topics and Nodes

```bash
# List all topics
ros2 topic list

# List all nodes
ros2 node list

# Show topic info
ros2 topic info /PolicyCommand

# Monitor message flow
ros2 topic hz /VehicleStatus
```

### RQT Tools (requires X11 forwarding)

```bash
# Start RQT
rqt

# Or specific tools
rqt_graph  # Node/topic graph
rqt_topic  # Topic monitor
rqt_service_caller  # Service testing
```

## GUI Applications with X11 Forwarding (Optional)

For running RViz2, RQT, and other GUI tools on Mac:

### 1. Install XQuartz

```bash
# Install XQuartz using Homebrew
brew install --cask xquartz

# Start XQuartz and enable network connections
open -a XQuartz
# In XQuartz preferences: Security → "Allow connections from network clients"
```

### 2. Set Display Variable

```bash
# Get your IP address
export DISPLAY=$(ipconfig getifaddr en0):0

# Allow X11 forwarding
xhost +localhost
```

### 3. Run GUI Applications

```bash
# Start with GUI profile
docker-compose --profile gui up rviz2

# Or in running container
docker-compose exec ros2-system-controller rviz2
```

## Package Structure

```
src/system_controller/
├── package.xml                 # Package metadata
├── CMakeLists.txt             # Build configuration
├── msg/                       # Custom message definitions
│   ├── SystemCommand.msg
│   ├── VehicleStatus.msg
│   └── MissionStatus.msg
├── srv/                       # Service definitions
│   └── GetStatus.srv
├── include/                   # Header files
│   ├── policy_base.hpp
│   ├── vehicle_type_base.hpp
│   ├── mission_type_base.hpp
│   └── payload_type_base.hpp
├── src/                       # Implementation files
│   ├── *.cpp                  # Node implementations
│   └── *_base.cpp            # Class implementations
└── launch/                    # Launch files
    ├── system_controller.launch.py
    └── demo.launch.py
```

## Class Hierarchies

### Policy Classes
- `PolicyBase` (abstract)
  - `Policy1`: Forward movement with variable speed
  - `Policy2`: Turn-based navigation pattern
  - `Policy3`: Aerial maneuvers and waypoint navigation

### Vehicle Type Classes
- `VehicleTypeBase` (abstract)
  - `VehicleType1`: Ground vehicle (wheels, steering)
  - `VehicleType2`: Aerial vehicle (rotors, hover)

### Mission Type Classes
- `MissionTypeBase` (abstract)
  - `MissionType1`: Surveillance mission
  - `MissionType2`: Delivery mission

### Payload Type Classes
- `PayloadTypeBase` (abstract)
  - `PayloadType1`: Camera payload
  - `PayloadType2`: Sensor package

## Key Features

- **Modular Architecture**: Each component is a separate node
- **Plugin-like System**: Easy to add new policy/vehicle/mission/payload types
- **Command Arbitration**: Priority-based command selection
- **Comprehensive Logging**: Message flow visibility
- **Docker Support**: Easy development on Mac
- **Service Interface**: Status monitoring and control
- **Custom Messages**: Structured communication

## Troubleshooting

### Common Issues

1. **Container won't start**: Check Docker Desktop is running
2. **Build failures**: Ensure all dependencies are installed with `rosdep`
3. **No GUI**: Verify XQuartz is running and DISPLAY is set
4. **Permission errors**: Check Docker volume mount permissions

### Debug Commands

```bash
# Check ROS environment
printenv | grep ROS

# Verify message definitions
ros2 interface show system_controller/msg/SystemCommand

# Test node connectivity
ros2 node info /system_manager

# Monitor system resources
htop  # Inside container
```

### Apple Silicon Specific

If you encounter issues on M1/M2 Macs:

```bash
# Try AMD64 emulation
docker build --platform linux/amd64 -t ros2-system-controller .

# Check architecture
docker run --rm ros2-system-controller uname -m
```

## Development

### Adding New Components

1. **New Policy**: Extend `PolicyBase` in `include/policy_base.hpp`
2. **New Vehicle Type**: Extend `VehicleTypeBase` in `include/vehicle_type_base.hpp`
3. **New Mission Type**: Extend `MissionTypeBase` in `include/mission_type_base.hpp`
4. **New Payload Type**: Extend `PayloadTypeBase` in `include/payload_type_base.hpp`

### Build and Test Cycle

```bash
# Inside container
colcon build --packages-select system_controller
source install/setup.bash
ros2 launch system_controller demo.launch.py
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make changes and test thoroughly
4. Submit a pull request

## License

Apache 2.0 License 