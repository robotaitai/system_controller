# ROS 2 System Controller

A comprehensive ROS 2 (Humble) C++ codebase implementing a modular agricultural automation system with policy management, command arbitration, vehicle/implement adapters, mission business logic, and advanced safety features.

## Architecture Overview

This system implements a sophisticated distributed architecture designed for agricultural automation with the following components:

- **System Manager Node**: Manages multiple policy types and publishes policy commands
- **Command Input Arbiter Node**: Arbitrates between policy, teleop, and autonomy commands
- **Vehicle Adapter Manager Node**: Manages vehicle types and interfaces with vehicle hardware
- **Implement Adapters**: Specialized nodes for agricultural equipment (SprayerAdapter, MowerAdapter, SeederAdapter)
- **Mission Nodes**: Business logic controllers for field operations (SprayerMission, MowerMission, SeederMission)
- **Policy Nodes**: Advanced policy management (TeleopOnlyPolicy, AdasPolicy)
- **Telemetry Collector Node**: Collects and redistributes telemetry data

### Communication Flow

```
System Manager → Policy Commands → Command Arbiter → Vehicle Commands → Vehicle Adapters → Vehicle HW
                     ↓                                      ↓
                Policy State                           Implement Commands
                     ↓                                      ↓
             Policy Enforcement                    Specific Adapters → Implement HW
                     ↓                                      ↓
            Mission Controllers ← Mission Status ← Business Logic Control
                     ↓
            Telemetry Collector ← Status Data ← Safety Monitoring
```

### Specific Command Messages

The system uses type-safe command messages for different implement types:

- **VehicleCommand**: Steering, velocity, acceleration, safety controls
- **SprayerCommand**: Chemical application, flow rates, boom control, environmental monitoring
- **MowerCommand**: Cutting height, blade speed, patterns, terrain safety
- **SeederCommand**: Seed rates, planting depth, soil conditions, fertilizer application
- **ImplementCommand**: Base message for generic implement control
- **PolicyState**: Operating modes, safety flags, constraints

## Prerequisites

### For Local Development (without Docker)
- ROS 2 Humble
- C++ compiler (GCC 9+)
- CMake 3.8+
- colcon build tools
- geometry_msgs package

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

# Core System Nodes
ros2 run system_controller system_manager_node
ros2 run system_controller command_input_arbiter_node
ros2 run system_controller vehicle_adapter_manager_node
ros2 run system_controller telemetry_collector_node
ros2 run system_controller mission_service_node
ros2 run system_controller mission_adapter_manager_node

# Specific Implement Adapters
ros2 run system_controller sprayer_adapter_node
ros2 run system_controller mower_adapter_node
ros2 run system_controller seeder_adapter_node

# Mission Controllers
ros2 run system_controller sprayer_mission_node
ros2 run system_controller mower_mission_node
ros2 run system_controller seeder_mission_node

# Policy Controllers
ros2 run system_controller teleop_only_policy_node
ros2 run system_controller adas_policy_node
```

### 3. Testing Message Flow

Open additional terminals to test the system:

```bash
# Send teleop commands (high priority)
ros2 topic pub /Teleop/Command std_msgs/String "data: 'emergency_stop'"

# Send autonomy commands (medium priority)
ros2 topic pub /Autonomy/Command std_msgs/String "data: 'autonomous_navigation'"

# Send vehicle commands
ros2 topic pub /VehicleCommand system_controller/msg/VehicleCommand "{
  steering_angle: 0.5,
  velocity: 2.0,
  acceleration: 0.1,
  emergency_stop: false
}"

# Send implement-specific commands
ros2 topic pub /SprayerCommand system_controller/msg/SprayerCommand "{
  base: {activate: true, raise: false},
  chemical_type: 'herbicide',
  application_rate: 10.5,
  spray_pressure: 3.0
}"

ros2 topic pub /MowerCommand system_controller/msg/MowerCommand "{
  base: {activate: true, raise: false},
  cutting_height: 0.05,
  blade_speed: 3000.0,
  cutting_pattern: 'stripe'
}"

ros2 topic pub /SeederCommand system_controller/msg/SeederCommand "{
  base: {activate: true, raise: false},
  seed_rate: 50000.0,
  planting_depth: 0.025,
  seed_type: 'corn'
}"

# Monitor system topics
ros2 topic echo /PolicyState
ros2 topic echo /VehicleCommand
ros2 topic echo /SprayerCommand
ros2 topic echo /VehicleStatus
ros2 topic echo /MissionStatus
```

### 4. Service Testing

```bash
# Test status services
ros2 service call /vehicle_adapter_status system_controller/srv/GetStatus "{component_id: 'vehicle'}"
ros2 service call /mission_service_status system_controller/srv/GetStatus "{component_id: 'mission'}"

# Test adapter configuration
ros2 service call /configure_adapter system_controller/srv/ConfigureAdapter "{
  adapter_id: 'sprayer_001',
  parameter_name: 'max_pressure',
  parameter_value: '5.0'
}"
```

### 5. Testing Agricultural Scenarios

```bash
# Test spraying mission with weather monitoring
ros2 topic pub /SprayerCommand system_controller/msg/SprayerCommand "{
  base: {activate: true},
  chemical_type: 'herbicide',
  application_rate: 15.0,
  environmental_monitoring: true
}"

# Test mowing with terrain adaptation
ros2 topic pub /MowerCommand system_controller/msg/MowerCommand "{
  base: {activate: true},
  cutting_height: 0.08,
  terrain_following: true,
  cutting_pattern: 'spiral'
}"

# Test precision seeding
ros2 topic pub /SeederCommand system_controller/msg/SeederCommand "{
  base: {activate: true},
  seed_rate: 65000.0,
  planting_depth: 0.03,
  variable_rate_seeding: true
}"
```

## Monitoring and Debugging

### View Active Topics and Nodes

```bash
# List all topics
ros2 topic list

# List all nodes
ros2 node list

# Show message structure
ros2 interface show system_controller/msg/SprayerCommand
ros2 interface show system_controller/msg/MowerCommand
ros2 interface show system_controller/msg/SeederCommand

# Monitor message flow
ros2 topic hz /VehicleCommand
ros2 topic hz /SprayerCommand
ros2 topic hz /PolicyState
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

## Package Structure

```
src/system_controller/
├── package.xml                     # Package metadata with all dependencies
├── CMakeLists.txt                 # Complete build configuration
├── msg/                           # Enhanced message definitions
│   ├── SystemCommand.msg
│   ├── VehicleCommand.msg         # Vehicle control (steering, velocity, safety)
│   ├── VehicleStatus.msg
│   ├── ImplementCommand.msg       # Base implement control
│   ├── SprayerCommand.msg         # Sprayer-specific commands
│   ├── MowerCommand.msg           # Mower-specific commands
│   ├── SeederCommand.msg          # Seeder-specific commands
│   ├── PolicyState.msg            # Policy state and operating modes
│   └── MissionStatus.msg
├── srv/                           # Service definitions
│   ├── GetStatus.srv
│   └── ConfigureAdapter.srv       # Runtime adapter configuration
├── include/                       # Header files
│   ├── adapters/                  # Vehicle and implement adapters
│   │   ├── vehicle_adapter_base.hpp
│   │   ├── polaris_adapter.hpp
│   │   ├── new_holland_adapter.hpp
│   │   ├── implement_adapter_base.hpp
│   │   ├── sprayer_adapter.hpp
│   │   ├── mower_adapter.hpp
│   │   └── seeder_adapter.hpp
│   ├── missions/                  # Mission business logic
│   │   ├── mission_base.hpp
│   │   ├── sprayer_mission.hpp
│   │   ├── mower_mission.hpp
│   │   └── seeder_mission.hpp
│   ├── policies/                  # Policy management
│   │   ├── policy_base_expanded.hpp
│   │   ├── teleop_only_policy.hpp
│   │   └── adas_policy.hpp
│   └── utils/                     # Utility classes
│       ├── vehicle_adapter_factory.hpp
│       └── mission_factory.hpp
├── src/                           # Implementation files
│   ├── adapters/                  # Adapter implementations
│   │   ├── polaris_adapter.cpp
│   │   ├── new_holland_adapter.cpp
│   │   ├── sprayer_adapter.cpp
│   │   ├── mower_adapter.cpp
│   │   └── seeder_adapter.cpp
│   ├── missions/                  # Mission implementations
│   │   ├── sprayer_mission.cpp
│   │   ├── mower_mission.cpp
│   │   └── seeder_mission.cpp
│   ├── policies/                  # Policy implementations
│   │   ├── teleop_only_policy.cpp
│   │   └── adas_policy.cpp
│   ├── *.cpp                      # Core node implementations
│   └── *_base.cpp                 # Base class implementations
├── config/                        # Configuration files
│   ├── vehicle_adapters.yaml     # Vehicle adapter configurations
│   ├── missions.yaml             # Mission configurations
│   └── policies.yaml             # Policy configurations
└── launch/                        # Launch files
    ├── system_controller.launch.py
    └── demo.launch.py
```

## System Architecture

### Vehicle Adapters
- **PolarisAdapter**: CAN bus communication, Ackermann steering, heartbeat monitoring
- **NewHollandAdapter**: Modbus RTU protocol, serial communication
- **Plugin Architecture**: Factory pattern for runtime adapter loading

### Implement Adapters  
- **SprayerAdapter**: Chemical application control, boom management, environmental monitoring
- **MowerAdapter**: Cutting height control, blade management, terrain following
- **SeederAdapter**: Variable rate seeding, soil monitoring, precision planting

### Mission Controllers
- **SprayerMission**: Weather monitoring, GPS accuracy, tree detection, turn sequences
- **MowerMission**: Pattern execution (stripe/spiral/random), terrain adaptation
- **SeederMission**: Variable rate application, soil condition monitoring, emergence tracking

### Policy Management
- **TeleopOnlyPolicy**: Manual control enforcement, operator presence, dead-man switch
- **AdasPolicy**: Collision avoidance, lane keeping, adaptive cruise, tree detection
- **Operating Modes**: TELEOP_ONLY, AUTONOMOUS, SEMI_AUTONOMOUS, ADAS_ASSISTED, SAFETY_OVERRIDE

## Key Features

- **Type-Safe Commands**: Specific message types for each implement type
- **Modular Architecture**: Each component is a separate, pluggable node
- **Advanced Safety Systems**: Multiple intervention layers and emergency stop
- **Business Logic Integration**: Agricultural-specific mission controllers
- **Plugin-Based Design**: Easy to add new vehicles, implements, missions, policies
- **Command Arbitration**: Priority-based command selection with safety filtering
- **Comprehensive Configuration**: YAML-based system configuration
- **Environmental Monitoring**: Weather, GPS, soil condition integration
- **Real-time Adaptation**: Dynamic parameter adjustment and error recovery
- **Docker Support**: Easy development and deployment on any platform

## Agricultural Use Cases

### Precision Spraying
- GPS-guided herbicide/pesticide application
- Weather condition monitoring (wind speed, temperature)
- Tree detection with automatic spray pause
- Buffer zone management around sensitive areas
- Variable rate application based on field mapping

### Autonomous Mowing
- Multiple cutting patterns (stripe, spiral, random)
- Terrain-following blade height adjustment
- Operator presence monitoring for safety
- Obstacle detection and avoidance
- Turn sequence optimization

### Precision Seeding
- Variable rate seeding based on soil conditions
- GPS-guided row spacing and depth control
- Soil moisture and temperature monitoring
- Fertilizer application integration
- Emergence tracking and reporting

## Safety Features

- **Emergency Stop**: Immediate system shutdown from any node
- **Tree Detection**: Automatic implement pause near obstacles
- **GPS Monitoring**: Mission pause on accuracy loss
- **Weather Integration**: Operation suspension in adverse conditions
- **Operator Presence**: Dead-man switch and proximity monitoring
- **Geofencing**: Restricted area enforcement
- **Command Filtering**: Policy-based safety intervention
- **Heartbeat Monitoring**: Communication failure detection

## Troubleshooting

### Common Issues

1. **Container won't start**: Check Docker Desktop is running
2. **Build failures**: Ensure all dependencies are installed with `rosdep`
3. **Missing messages**: Check that geometry_msgs is installed
4. **Adapter not found**: Verify adapter registration in factory
5. **Communication errors**: Check port permissions and hardware connections

### Debug Commands

```bash
# Check ROS environment
printenv | grep ROS

# Verify message definitions
ros2 interface show system_controller/msg/SprayerCommand
ros2 interface show system_controller/msg/MowerCommand
ros2 interface show system_controller/msg/SeederCommand

# Test node connectivity
ros2 node info /sprayer_adapter_node
ros2 node info /sprayer_mission_node

# Monitor system resources
htop  # Inside container

# Check topic communication
ros2 topic hz /SprayerCommand
ros2 topic echo /PolicyState --once
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

1. **New Vehicle Adapter**: Extend `VehicleAdapterBase` and register with factory
2. **New Implement Adapter**: Extend `ImplementAdapterBase` for your equipment type
3. **New Mission Type**: Extend `MissionBase` with your business logic
4. **New Policy**: Extend `PolicyBaseExpanded` with your control strategy
5. **New Messages**: Add implement-specific command messages as needed

### Build and Test Cycle

```bash
# Inside container
colcon build --packages-select system_controller
source install/setup.bash

# Test specific components
ros2 run system_controller sprayer_adapter_node
ros2 topic pub /SprayerCommand system_controller/msg/SprayerCommand "{...}"

# Full system test
ros2 launch system_controller demo.launch.py
```

### Configuration Development

Update YAML files for your specific equipment:

```yaml
# config/vehicle_adapters.yaml - Add your vehicle
your_vehicle:
  adapter_type: "your_vehicle_type"
  communication_port: "/dev/your_port"
  custom_params:
    max_speed: 10.0

# config/missions.yaml - Add your mission
your_mission:
  mission_type: "your_mission_type"
  required_implements: ["your_implement"]
  safety_constraints:
    max_wind_speed: 15.0
```

## Documentation

- **ADAPTER_POLICY_GUIDE.md**: Comprehensive guide for adding new adapters, missions, and policies
- **Source Code Comments**: Detailed inline documentation
- **Configuration Examples**: YAML configuration templates
- **Integration Examples**: Real-world usage scenarios

## Contributing

1. Fork the repository
2. Create a feature branch
3. Add comprehensive tests for your changes
4. Update documentation
5. Submit a pull request with detailed description

## License

Apache 2.0 License 