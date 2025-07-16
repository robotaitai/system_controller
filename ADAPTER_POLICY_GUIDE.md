# ROS 2 System Controller - Adapter and Policy Development Guide

This guide explains how to extend the ROS 2 System Controller with new vehicle adapters, implement adapters, missions, and policies.

## Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [Adding Vehicle Adapters](#adding-vehicle-adapters)
3. [Adding Implement Adapters](#adding-implement-adapters)
4. [Creating New Missions](#creating-new-missions)
5. [Implementing New Policies](#implementing-new-policies)
6. [Configuration Management](#configuration-management)
7. [Testing and Validation](#testing-and-validation)
8. [Examples](#examples)

## Architecture Overview

The system uses a plugin-based architecture with standardized interfaces:

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Policy Layer  │    │  Mission Layer  │    │  Adapter Layer  │
│                 │    │                 │    │                 │
│ • TeleopPolicy  │    │ • SprayMission  │    │ • PolarisAdapter│
│ • AdasPolicy    │    │ • MowMission    │    │ • NewHollandAdp │
│ • CustomPolicy  │    │ • CustomMission │    │ • CustomAdapter │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                        │                        │
         ▼                        ▼                        ▼
┌─────────────────────────────────────────────────────────────────┐
│                    System Manager                              │
│           Command Arbitration & Message Flow                   │
└─────────────────────────────────────────────────────────────────┘
         │                        │                        │
         ▼                        ▼                        ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│Vehicle Commands │    │Implement Cmds   │    │  Status/Safety  │
│• Steering       │    │• Spray Control  │    │• Health Monitor │
│• Velocity       │    │• Height Control │    │• Error Handling │
│• Safety Stop    │    │• Flow Rate      │    │• Diagnostics    │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

## Adding Vehicle Adapters

Vehicle adapters translate high-level `VehicleCommand` messages into vehicle-specific protocols (CAN, Serial, Ethernet, etc.).

### Step 1: Create the Adapter Header

Create `src/system_controller/include/adapters/your_vehicle_adapter.hpp`:

```cpp
#ifndef YOUR_VEHICLE_ADAPTER_HPP
#define YOUR_VEHICLE_ADAPTER_HPP

#include "vehicle_adapter_base.hpp"

namespace system_controller {

class YourVehicleAdapter : public VehicleAdapterBase {
public:
    YourVehicleAdapter();
    ~YourVehicleAdapter() override = default;

    // Implement all pure virtual functions from VehicleAdapterBase
    bool initialize(const VehicleAdapterConfig& config) override;
    bool connect() override;
    bool disconnect() override;
    bool isConnected() const override;
    
    bool processVehicleCommand(const VehicleCommand& command) override;
    VehicleStatusInfo getStatus() const override;
    
    // ... other required methods
    
private:
    // Your vehicle-specific members
    YourVehicleConfig your_config_;
    int communication_handle_;
    // ... etc
};

} // namespace system_controller

#endif
```

### Step 2: Implement the Adapter

Create `src/system_controller/src/adapters/your_vehicle_adapter.cpp`:

```cpp
#include "../../include/adapters/your_vehicle_adapter.hpp"

namespace system_controller {

YourVehicleAdapter::YourVehicleAdapter() 
    : VehicleAdapterBase("your_vehicle_adapter") {
    // Initialize your adapter
}

bool YourVehicleAdapter::initialize(const VehicleAdapterConfig& config) {
    config_ = config;
    
    // Parse your vehicle-specific parameters
    auto it = config.custom_params.find("your_param");
    if (it != config.custom_params.end()) {
        your_config_.your_param = it->second;
    }
    
    return true;
}

bool YourVehicleAdapter::processVehicleCommand(const VehicleCommand& command) {
    if (!validateCommand(command)) {
        setError("Invalid command");
        return false;
    }
    
    // Translate to your vehicle protocol
    // Example: Convert steering angle to your format
    auto your_steering = convertSteeringAngle(command.steering_angle);
    auto your_velocity = convertVelocity(command.velocity);
    
    // Send to vehicle hardware
    return sendToVehicle(your_steering, your_velocity);
}

// ... implement other methods

} // namespace system_controller
```

### Step 3: Register the Adapter

Add registration in your implementation file:

```cpp
// Register with factory (add to your adapter's .cpp file)
#include "vehicle_adapter_factory.hpp"
REGISTER_VEHICLE_ADAPTER(YourVehicleAdapter, "your_vehicle_type");
```

### Step 4: Add Configuration

Update `src/system_controller/config/vehicle_adapters.yaml`:

```yaml
vehicle_adapters:
  your_vehicle_instance:
    adapter_type: "your_vehicle_type"
    adapter_id: "your_vehicle_001"
    communication_port: "/dev/your_port"
    # ... your parameters
    custom_params:
      your_param: "your_value"
```

## Adding Implement Adapters

Implement adapters control mission equipment (sprayers, mowers, seeders, etc.).

### Step 1: Create the Adapter

```cpp
// src/system_controller/include/adapters/your_implement_adapter.hpp
class YourImplementAdapter : public ImplementAdapterBase {
public:
    YourImplementAdapter();
    
    // Implement ImplementAdapterBase interface
    bool processImplementCommand(const ImplementCommand& command) override;
    
    // Implement-specific methods
    bool activateImplement() override;
    bool setHeight(double height) override;
    // ... etc

private:
    // Your implement-specific logic
};
```

### Step 2: Implement Business Logic

```cpp
bool YourImplementAdapter::processImplementCommand(const ImplementCommand& command) {
    // Handle implement-specific commands
    if (command.implement_type == "your_implement") {
        if (command.activate) {
            return activateImplement();
        }
        
        if (command.raise) {
            return raiseImplement();
        }
        
        // Handle your specific parameters
        if (command.your_specific_field > 0) {
            return setYourSpecificControl(command.your_specific_field);
        }
    }
    
    return false;
}
```

## Creating New Missions

Missions contain the business logic for field operations.

### Step 1: Create Mission Header

```cpp
// src/system_controller/include/missions/your_mission.hpp
class YourMission : public MissionBase {
public:
    YourMission();
    
    // Implement MissionBase interface
    bool start() override;
    bool startNewRow() override;
    bool endCurrentRow() override;
    std::vector<std::string> getRequiredImplements() const override;
    
protected:
    // Business logic hooks
    void onRowStart() override;
    void onRowEnd() override;
    void onTurnStart() override;
    void onTurnEnd() override;

private:
    // Mission-specific logic
    bool checkYourSpecificConditions();
    bool performYourSpecificAction();
    
    YourMissionConfig your_config_;
};
```

### Step 2: Implement Mission Logic

```cpp
bool YourMission::startNewRow() {
    // Your specific row start logic
    if (!checkYourSpecificConditions()) {
        setError("Conditions not met for row start");
        return false;
    }
    
    // Activate required implements
    if (!activateImplement()) {
        return false;
    }
    
    // Your mission-specific setup
    return performYourSpecificAction();
}

void YourMission::onRowEnd() {
    // Business logic: what to do at end of row
    // Example: Lift implement, stop application, etc.
    
    auto implement = getImplementAdapter("your_implement");
    if (implement) {
        implement->raiseImplement();
        implement->deactivate();
    }
}
```

### Step 3: Register Mission

```cpp
REGISTER_MISSION(YourMission, "your_mission_type");
```

## Implementing New Policies

Policies control operating modes and command filtering.

### Step 1: Create Policy Class

```cpp
// src/system_controller/include/policies/your_policy.hpp
class YourPolicy : public PolicyBaseExpanded {
public:
    YourPolicy();
    
    // Mode management
    bool requestModeChange(PolicyStateInfo::OperatingMode mode) override;
    bool canTransitionTo(PolicyStateInfo::OperatingMode target) const override;
    
    // Command filtering
    CommandFilterResult filterVehicleCommand(const VehicleCommand& command) override;
    CommandFilterResult filterImplementCommand(const ImplementCommand& command) override;
    
    // Mission policies
    bool isMissionAllowed(const std::string& mission_type) const override;
    
private:
    // Your policy-specific logic
    bool checkYourSpecificConditions();
    YourPolicyConfig your_config_;
};
```

### Step 2: Implement Policy Logic

```cpp
CommandFilterResult YourPolicy::filterVehicleCommand(const VehicleCommand& command) {
    CommandFilterResult result;
    
    // Your policy logic
    if (!checkYourSpecificConditions()) {
        result.allowed = false;
        result.reason = "Your policy violation";
        return result;
    }
    
    // Modify command if needed
    if (needsYourModification(command)) {
        VehicleCommand modified = command;
        modified.velocity = std::min(command.velocity, your_config_.max_velocity);
        result.modified_vehicle_command = &modified;
    }
    
    result.allowed = true;
    return result;
}

bool YourPolicy::isMissionAllowed(const std::string& mission_type) const {
    // Your mission policy logic
    if (mission_type == "dangerous_mission" && !safety_conditions_met_) {
        return false;
    }
    return true;
}
```

### Step 3: Register Policy

```cpp
REGISTER_POLICY(YourPolicy, "your_policy_type");
```

## Configuration Management

### Vehicle Adapter Configuration

```yaml
# src/system_controller/config/vehicle_adapters.yaml
vehicle_adapters:
  your_vehicle:
    adapter_type: "your_type"
    communication_port: "/dev/your_port"
    custom_params:
      your_param: "value"
```

### Mission Configuration

```yaml
# src/system_controller/config/missions.yaml
missions:
  your_mission:
    mission_type: "your_type"
    required_implements: ["implement1", "implement2"]
    default_config:
      your_param: 123.45
```

### Policy Configuration

```yaml
# src/system_controller/config/policies.yaml
policies:
  your_policy:
    policy_type: "your_policy_type"
    config:
      your_setting: true
      your_threshold: 10.0
```

## Testing and Validation

### Unit Tests

Create unit tests for your components:

```cpp
// test/test_your_adapter.cpp
TEST(YourAdapterTest, BasicFunctionality) {
    YourVehicleAdapter adapter;
    VehicleAdapterConfig config;
    
    ASSERT_TRUE(adapter.initialize(config));
    ASSERT_TRUE(adapter.connect());
    
    VehicleCommand command{};
    command.steering_angle = 0.5;
    command.velocity = 2.0;
    
    ASSERT_TRUE(adapter.processVehicleCommand(command));
}
```

### Integration Tests

Test your components with the full system:

```cpp
TEST(SystemIntegrationTest, YourAdapterIntegration) {
    // Test your adapter with the full system
    SystemManager manager;
    manager.addVehicleAdapter("your_adapter");
    
    // Send commands and verify behavior
}
```

## Examples

### Example: Custom GPS-Guided Sprayer

```cpp
class GPSGuidedSprayerMission : public MissionBase {
private:
    bool checkGPSAccuracy() {
        return gps_accuracy_ < 0.05; // 5cm accuracy required
    }
    
    void onRowStart() override {
        if (!checkGPSAccuracy()) {
            pauseMission("GPS accuracy insufficient");
            return;
        }
        
        // Lower boom, start spraying
        auto sprayer = getImplementAdapter("boom_sprayer");
        sprayer->lowerImplement();
        sprayer->activate();
    }
    
    void onTurnStart() override {
        // Lift boom for turn
        auto sprayer = getImplementAdapter("boom_sprayer");
        sprayer->raiseImplement();
        sprayer->deactivate();
    }
};
```

### Example: Safety-First Policy

```cpp
class SafetyFirstPolicy : public PolicyBaseExpanded {
public:
    CommandFilterResult filterVehicleCommand(const VehicleCommand& command) override {
        CommandFilterResult result;
        
        // Always enforce speed limits
        if (command.velocity > MAX_SAFE_SPEED) {
            VehicleCommand modified = command;
            modified.velocity = MAX_SAFE_SPEED;
            result.modified_vehicle_command = &modified;
            result.reason = "Speed limited for safety";
        }
        
        // Stop if obstacle detected
        if (obstacle_detected_) {
            result.allowed = false;
            result.reason = "Obstacle detected";
            return result;
        }
        
        result.allowed = true;
        return result;
    }
};
```

## Build System Integration

Update `CMakeLists.txt` to include your new components:

```cmake
# Add your adapter sources
add_executable(your_vehicle_adapter_node 
    src/your_vehicle_adapter_node.cpp
    src/adapters/your_vehicle_adapter.cpp)

# Link with base libraries
ament_target_dependencies(your_vehicle_adapter_node rclcpp std_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(your_vehicle_adapter_node "${cpp_typesupport_target}")
```

## Best Practices

1. **Error Handling**: Always validate inputs and provide meaningful error messages
2. **Safety First**: Implement proper safety checks and emergency stop functionality
3. **Documentation**: Document your adapter's communication protocol and configuration
4. **Testing**: Write comprehensive unit and integration tests
5. **Configuration**: Make your adapter configurable through YAML files
6. **Logging**: Use ROS 2 logging for debugging and monitoring
7. **Thread Safety**: Ensure thread-safe operations for concurrent access

## Troubleshooting

### Common Issues

1. **Adapter Not Found**: Ensure your adapter is registered with the factory
2. **Communication Errors**: Check port permissions and communication settings
3. **Build Errors**: Verify all dependencies are included in CMakeLists.txt
4. **Runtime Errors**: Check ROS 2 logs for detailed error messages

### Debug Tools

```bash
# View ROS 2 logs
ros2 log view

# Monitor topics
ros2 topic echo /VehicleCommand
ros2 topic echo /ImplementCommand

# Check node status
ros2 node list
ros2 node info /your_adapter_node
```

---

For more information, see the ROS 2 documentation and the system controller source code. 