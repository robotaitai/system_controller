# ROS 2 System Controller - Adapter and Policy Development Guide

This guide explains how to extend the ROS 2 System Controller with new vehicle adapters, implement adapters, missions, and policies for agricultural automation systems.

## Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [Message Type System](#message-type-system)
3. [Adding Vehicle Adapters](#adding-vehicle-adapters)
4. [Adding Implement Adapters](#adding-implement-adapters)
5. [Creating New Missions](#creating-new-missions)
6. [Implementing New Policies](#implementing-new-policies)
7. [Configuration Management](#configuration-management)
8. [Testing and Validation](#testing-and-validation)
9. [Real-World Examples](#real-world-examples)
10. [Best Practices](#best-practices)

## Architecture Overview

The system uses a sophisticated plugin-based architecture with type-safe interfaces designed for agricultural automation:

```
┌─────────────────────────────────────────────────────────────────┐
│                     POLICY LAYER                               │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  │
│  │ TeleopOnly      │  │ ADAS Policy     │  │ Custom Policy   │  │
│  │ Policy          │  │                 │  │                 │  │
│  │ • Manual Control│  │ • Collision     │  │ • Your Rules    │  │
│  │ • Dead Man      │  │   Avoidance     │  │ • Custom Safety │  │
│  │ • Operator      │  │ • Tree Detection│  │ • Field Logic   │  │
│  │   Presence      │  │ • GPS Monitor   │  │                 │  │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                   COMMAND ARBITRATION                          │
│           Priority Management & Safety Filtering               │
└─────────────────────────────────────────────────────────────────┘
              ▼                                  ▼
┌───────────────────────────┐    ┌───────────────────────────────┐
│      VEHICLE LAYER        │    │      IMPLEMENT LAYER          │
│  ┌─────────────────────┐  │    │  ┌─────────────────────────┐  │
│  │ Vehicle Commands    │  │    │  │ Sprayer Commands        │  │
│  │ • Steering Angle    │  │    │  │ • Chemical Type         │  │
│  │ • Velocity          │  │    │  │ • Application Rate      │  │
│  │ • Acceleration      │  │    │  │ • Spray Pressure        │  │
│  │ • Emergency Stop    │  │    │  │ • Boom Control          │  │
│  └─────────────────────┘  │    │  │ • Weather Monitor       │  │
│           ▼               │    │  └─────────────────────────┘  │
│  ┌─────────────────────┐  │    │  ┌─────────────────────────┐  │
│  │ Polaris Adapter     │  │    │  │ Mower Commands          │  │
│  │ • CAN Bus           │  │    │  │ • Cutting Height        │  │
│  │ • Ackermann         │  │    │  │ • Blade Speed           │  │
│  │ • Heartbeat         │  │    │  │ • Cutting Pattern       │  │
│  └─────────────────────┘  │    │  │ • Terrain Safety        │  │
│  ┌─────────────────────┐  │    │  └─────────────────────────┘  │
│  │ New Holland         │  │    │  ┌─────────────────────────┐  │
│  │ • Modbus RTU        │  │    │  │ Seeder Commands         │  │
│  │ • Serial Comm       │  │    │  │ • Seed Rate             │  │
│  │ • Protocol Stack    │  │    │  │ • Planting Depth        │  │
│  └─────────────────────┘  │    │  │ • Soil Monitoring       │  │
└───────────────────────────┘    │  │ • Fertilizer Control    │  │
                                 │  └─────────────────────────┘  │
                                 └───────────────────────────────┘
                                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                   MISSION LAYER                                │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  │
│  │ Sprayer Mission │  │ Mower Mission   │  │ Seeder Mission  │  │
│  │ • Weather Check │  │ • Pattern Exec  │  │ • Variable Rate │  │
│  │ • GPS Monitor   │  │ • Terrain Adapt │  │ • Soil Monitor  │  │
│  │ • Tree Detection│  │ • Blade Control │  │ • Emergence     │  │
│  │ • Turn Sequence │  │ • Safety Stop   │  │   Tracking      │  │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

## Message Type System

The system uses type-safe command messages for different equipment types:

### Vehicle Commands
```cpp
// VehicleCommand.msg
double steering_angle      # Normalized [-1.0, 1.0]
double velocity           # m/s
double acceleration       # m/s²
bool emergency_stop       # Immediate stop flag
geometry_msgs/Twist cmd_vel
```

### Implement-Specific Commands

#### Sprayer Commands
```cpp
// SprayerCommand.msg
ImplementCommand base            # Base implement control
string chemical_type            # "herbicide", "pesticide", "fertilizer"
double application_rate         # L/ha or kg/ha
double spray_pressure          # Bar
bool[] boom_sections           # Individual section control
bool environmental_monitoring  # Weather condition checking
double target_droplet_size     # Microns
bool drift_reduction          # Drift reduction mode
```

#### Mower Commands
```cpp
// MowerCommand.msg
ImplementCommand base          # Base implement control
double cutting_height         # Meters above ground
double blade_speed           # RPM
string cutting_pattern       # "stripe", "spiral", "random"
bool terrain_following       # Automatic height adjustment
bool collection_enabled      # Grass collection system
bool mulching_mode          # Mulching vs collection
double overlap_percentage    # Pattern overlap 0-100%
```

#### Seeder Commands
```cpp
// SeederCommand.msg
ImplementCommand base              # Base implement control
double seed_rate                  # Seeds per hectare
double planting_depth            # Meters
string seed_type                 # Crop variety
bool variable_rate_seeding       # VRT enabled
double fertilizer_rate           # kg/ha (if equipped)
bool soil_condition_monitoring   # Real-time soil analysis
double row_spacing              # Meters between rows
bool emergence_tracking         # Post-plant monitoring
```

## Adding Vehicle Adapters

Vehicle adapters translate high-level `VehicleCommand` messages into vehicle-specific protocols.

### Step 1: Create the Adapter Header

Create `src/system_controller/include/adapters/your_vehicle_adapter.hpp`:

```cpp
#ifndef YOUR_VEHICLE_ADAPTER_HPP
#define YOUR_VEHICLE_ADAPTER_HPP

#include "vehicle_adapter_base.hpp"
#include "your_vehicle_protocol.hpp" // Your specific protocol

namespace system_controller {

class YourVehicleAdapter : public VehicleAdapterBase {
public:
    YourVehicleAdapter();
    ~YourVehicleAdapter() override = default;

    // Core adapter interface
    bool initialize(const VehicleAdapterConfig& config) override;
    bool connect() override;
    bool disconnect() override;
    bool isConnected() const override;
    
    bool processVehicleCommand(const VehicleCommand& command) override;
    VehicleStatusInfo getStatus() const override;
    
    // Vehicle-specific methods
    bool sendSteeringCommand(double angle);
    bool sendVelocityCommand(double velocity);
    bool performEmergencyStop();
    
    // Diagnostic and monitoring
    std::map<std::string, double> getDiagnostics() const override;
    bool performSelfTest() override;

private:
    // Configuration and state
    YourVehicleConfig config_;
    YourProtocolHandler protocol_handler_;
    
    // Communication
    int communication_fd_;
    std::string communication_port_;
    
    // State tracking
    VehicleState current_state_;
    std::chrono::steady_clock::time_point last_heartbeat_;
    
    // Helper methods
    bool validateCommand(const VehicleCommand& command);
    bool convertToVehicleUnits(const VehicleCommand& command, YourVehicleMessage& msg);
    bool handleHeartbeat();
    void updateStatus();
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
    : VehicleAdapterBase("your_vehicle_adapter"), communication_fd_(-1) {
    RCLCPP_INFO(this->get_logger(), "YourVehicleAdapter initialized");
}

bool YourVehicleAdapter::initialize(const VehicleAdapterConfig& config) {
    config_ = config;
    
    // Parse vehicle-specific parameters
    auto port_it = config.custom_params.find("communication_port");
    if (port_it != config.custom_params.end()) {
        communication_port_ = port_it->second;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Missing communication_port parameter");
        return false;
    }
    
    // Initialize protocol handler
    if (!protocol_handler_.initialize(config_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize protocol handler");
        return false;
    }
    
    return true;
}

bool YourVehicleAdapter::connect() {
    // Open communication channel (CAN, Serial, Ethernet, etc.)
    communication_fd_ = protocol_handler_.open(communication_port_);
    if (communication_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to vehicle");
        return false;
    }
    
    // Perform handshake
    if (!protocol_handler_.handshake()) {
        RCLCPP_ERROR(this->get_logger(), "Vehicle handshake failed");
        disconnect();
        return false;
    }
    
    // Start heartbeat monitoring
    last_heartbeat_ = std::chrono::steady_clock::now();
    
    RCLCPP_INFO(this->get_logger(), "Connected to vehicle successfully");
    return true;
}

bool YourVehicleAdapter::processVehicleCommand(const VehicleCommand& command) {
    if (!isConnected()) {
        setError("Vehicle not connected");
        return false;
    }
    
    if (!validateCommand(command)) {
        setError("Invalid vehicle command");
        return false;
    }
    
    // Handle emergency stop with highest priority
    if (command.emergency_stop) {
        return performEmergencyStop();
    }
    
    // Convert command to vehicle-specific format
    YourVehicleMessage vehicle_msg;
    if (!convertToVehicleUnits(command, vehicle_msg)) {
        setError("Command conversion failed");
        return false;
    }
    
    // Send to vehicle hardware
    if (!protocol_handler_.sendMessage(vehicle_msg)) {
        setError("Failed to send command to vehicle");
        return false;
    }
    
    // Update internal state
    current_state_.last_command = command;
    current_state_.last_command_time = std::chrono::steady_clock::now();
    
    return true;
}

bool YourVehicleAdapter::performEmergencyStop() {
    YourVehicleMessage stop_msg;
    stop_msg.message_type = YOUR_EMERGENCY_STOP;
    stop_msg.priority = HIGHEST_PRIORITY;
    
    bool success = protocol_handler_.sendEmergencyMessage(stop_msg);
    if (success) {
        current_state_.emergency_stopped = true;
        RCLCPP_WARN(this->get_logger(), "Emergency stop executed");
    }
    
    return success;
}

VehicleStatusInfo YourVehicleAdapter::getStatus() const {
    VehicleStatusInfo status;
    status.adapter_id = getAdapterId();
    status.is_connected = isConnected();
    status.is_healthy = isHealthy();
    status.last_update = std::chrono::steady_clock::now();
    
    // Vehicle-specific status
    status.vehicle_specific_data["current_speed"] = current_state_.velocity;
    status.vehicle_specific_data["steering_angle"] = current_state_.steering_angle;
    status.vehicle_specific_data["battery_voltage"] = current_state_.battery_voltage;
    status.vehicle_specific_data["engine_rpm"] = current_state_.engine_rpm;
    
    return status;
}

// Register the adapter with the factory
REGISTER_VEHICLE_ADAPTER(YourVehicleAdapter, "your_vehicle_type");

} // namespace system_controller
```

### Step 3: Add to Build System

Update `CMakeLists.txt`:

```cmake
# Add your vehicle adapter
add_executable(your_vehicle_adapter_node
    src/your_vehicle_adapter_node.cpp
    src/adapters/your_vehicle_adapter.cpp)

ament_target_dependencies(your_vehicle_adapter_node
    rclcpp
    std_msgs
    geometry_msgs
    your_vehicle_protocol_lib)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(your_vehicle_adapter_node "${cpp_typesupport_target}")

install(TARGETS your_vehicle_adapter_node DESTINATION lib/${PROJECT_NAME})
```

## Adding Implement Adapters

Implement adapters control agricultural equipment with type-safe command messages.

### Step 1: Choose Your Implement Type

Determine which specific implement you're adding:
- **Sprayer**: Chemical application, boom control, environmental monitoring
- **Mower**: Cutting systems, pattern execution, terrain following
- **Seeder**: Precision planting, variable rate, soil monitoring
- **Custom**: Create your own command message type

### Step 2: Create Specific Adapter

Example for a custom fertilizer spreader:

```cpp
// src/system_controller/include/adapters/fertilizer_adapter.hpp
#ifndef FERTILIZER_ADAPTER_HPP
#define FERTILIZER_ADAPTER_HPP

#include "implement_adapter_base.hpp"

namespace system_controller {

class FertilizerAdapter : public ImplementAdapterBase {
public:
    FertilizerAdapter();
    
    // Implement ImplementAdapterBase interface
    bool processImplementCommand(const ImplementCommand& command) override;
    
    // Fertilizer-specific methods
    bool setApplicationRate(double rate_kg_per_ha);
    bool setSpreadWidth(double width_meters);
    bool calibrateFlow();
    bool startApplication();
    bool stopApplication();
    
    // Safety and monitoring
    bool checkHopperLevel();
    bool validateApplicationRate(double rate);

private:
    // Fertilizer-specific configuration
    struct FertilizerConfig {
        double max_application_rate;
        double min_application_rate;
        double hopper_capacity;
        double spread_width;
        std::string fertilizer_type;
    } config_;
    
    // Current state
    struct FertilizerState {
        double current_rate;
        double hopper_level;
        bool application_active;
        double total_applied;
    } state_;
    
    // Communication
    FertilizerProtocolHandler protocol_;
};

} // namespace system_controller

#endif
```

### Step 3: Implement Business Logic

```cpp
// src/system_controller/src/adapters/fertilizer_adapter.cpp
bool FertilizerAdapter::processImplementCommand(const ImplementCommand& command) {
    // Validate command for fertilizer operations
    if (command.implement_type != "fertilizer") {
        return false;
    }
    
    // Handle activation/deactivation
    if (command.activate && !state_.application_active) {
        if (!checkHopperLevel()) {
            setError("Insufficient fertilizer in hopper");
            return false;
        }
        return startApplication();
    }
    
    if (!command.activate && state_.application_active) {
        return stopApplication();
    }
    
    // Handle rate adjustments
    if (command.custom_params.count("application_rate")) {
        double rate = std::stod(command.custom_params.at("application_rate"));
        if (!validateApplicationRate(rate)) {
            setError("Invalid application rate");
            return false;
        }
        return setApplicationRate(rate);
    }
    
    return true;
}

bool FertilizerAdapter::setApplicationRate(double rate_kg_per_ha) {
    if (!validateApplicationRate(rate)) {
        return false;
    }
    
    // Convert to spreader units
    FertilizerMessage msg;
    msg.command_type = SET_APPLICATION_RATE;
    msg.rate = convertToSpreadRate(rate_kg_per_ha);
    
    if (protocol_.sendCommand(msg)) {
        state_.current_rate = rate_kg_per_ha;
        return true;
    }
    
    return false;
}

bool FertilizerAdapter::checkHopperLevel() {
    // Query hopper level from hardware
    auto level_response = protocol_.queryHopperLevel();
    if (level_response.success) {
        state_.hopper_level = level_response.level_percentage;
        return state_.hopper_level > 5.0; // Minimum 5% for operation
    }
    return false;
}
```

## Creating New Missions

Missions contain the business logic for field operations with comprehensive safety and monitoring.

### Step 1: Design Mission Logic

Example for a precision fertilizer application mission:

```cpp
// src/system_controller/include/missions/fertilizer_mission.hpp
#ifndef FERTILIZER_MISSION_HPP
#define FERTILIZER_MISSION_HPP

#include "mission_base.hpp"
#include "fertilizer_adapter.hpp"
#include "gps_monitor.hpp"
#include "soil_analyzer.hpp"

namespace system_controller {

class FertilizerMission : public MissionBase {
public:
    FertilizerMission();
    
    // MissionBase interface
    bool start() override;
    bool pause() override;
    bool resume() override;
    bool stop() override;
    bool abort() override;
    
    bool startNewRow() override;
    bool endCurrentRow() override;
    
    std::vector<std::string> getRequiredImplements() const override {
        return {"fertilizer_spreader"};
    }

protected:
    // Business logic hooks
    void onRowStart() override;
    void onRowEnd() override;
    void onTurnStart() override;
    void onTurnEnd() override;
    
    // Mission-specific monitoring
    void onMissionTick() override;

private:
    // Fertilizer-specific logic
    bool checkSoilConditions();
    bool checkGPSAccuracy();
    bool adjustApplicationRate();
    bool validateFieldBoundaries();
    
    // Variable rate application
    double calculateOptimalRate(const SoilSample& sample);
    bool applyVariableRate(double rate);
    
    // Safety checks
    bool checkWeatherConditions();
    bool checkEnvironmentalRestrictions();
    
    // Configuration
    struct FertilizerMissionConfig {
        double base_application_rate;    // kg/ha
        double min_gps_accuracy;         // meters
        double max_wind_speed;           // m/s
        double soil_moisture_threshold;   // percentage
        bool variable_rate_enabled;
        std::string fertilizer_type;
    } config_;
    
    // Mission state
    struct FertilizerMissionState {
        double total_area_covered;
        double total_fertilizer_applied;
        size_t current_row;
        bool soil_conditions_ok;
        bool weather_conditions_ok;
    } state_;
    
    // Helper objects
    std::unique_ptr<GPSMonitor> gps_monitor_;
    std::unique_ptr<SoilAnalyzer> soil_analyzer_;
    std::unique_ptr<WeatherStation> weather_station_;
};

} // namespace system_controller

#endif
```

### Step 2: Implement Mission Business Logic

```cpp
// src/system_controller/src/missions/fertilizer_mission.cpp
bool FertilizerMission::startNewRow() {
    if (!checkSoilConditions()) {
        pauseMission("Soil conditions not suitable");
        return false;
    }
    
    if (!checkGPSAccuracy()) {
        pauseMission("GPS accuracy insufficient");
        return false;
    }
    
    if (!checkWeatherConditions()) {
        pauseMission("Weather conditions not suitable");
        return false;
    }
    
    // Calculate optimal application rate for this area
    auto soil_sample = soil_analyzer_->getCurrentSample();
    double optimal_rate = calculateOptimalRate(soil_sample);
    
    // Activate fertilizer spreader with calculated rate
    auto fertilizer = getImplementAdapter("fertilizer_spreader");
    if (!fertilizer) {
        setError("Fertilizer spreader not available");
        return false;
    }
    
    // Configure for row start
    ImplementCommand cmd;
    cmd.implement_type = "fertilizer";
    cmd.activate = true;
    cmd.lower = true;
    cmd.custom_params["application_rate"] = std::to_string(optimal_rate);
    
    if (!fertilizer->processImplementCommand(cmd)) {
        setError("Failed to activate fertilizer spreader");
        return false;
    }
    
    state_.current_row++;
    RCLCPP_INFO(get_logger(), "Started row %zu with rate %.2f kg/ha", 
                state_.current_row, optimal_rate);
    
    return true;
}

void FertilizerMission::onRowEnd() {
    // Stop application at row end
    auto fertilizer = getImplementAdapter("fertilizer_spreader");
    if (fertilizer) {
        ImplementCommand cmd;
        cmd.implement_type = "fertilizer";
        cmd.activate = false;
        cmd.raise = true;
        
        fertilizer->processImplementCommand(cmd);
        
        // Update mission statistics
        auto adapter_cast = std::dynamic_pointer_cast<FertilizerAdapter>(fertilizer);
        if (adapter_cast) {
            state_.total_fertilizer_applied += adapter_cast->getRowApplication();
            state_.total_area_covered += adapter_cast->getRowArea();
        }
    }
    
    RCLCPP_INFO(get_logger(), "Completed row %zu", state_.current_row);
}

double FertilizerMission::calculateOptimalRate(const SoilSample& sample) {
    if (!config_.variable_rate_enabled) {
        return config_.base_application_rate;
    }
    
    // Variable rate calculation based on soil analysis
    double rate_modifier = 1.0;
    
    // Adjust based on soil nutrient levels
    if (sample.nitrogen_level < 50.0) {  // ppm
        rate_modifier += 0.3;  // Increase by 30%
    } else if (sample.nitrogen_level > 100.0) {
        rate_modifier -= 0.2;  // Decrease by 20%
    }
    
    // Adjust based on soil moisture
    if (sample.moisture_content < 20.0) {
        rate_modifier -= 0.1;  // Reduce on dry soil
    }
    
    // Adjust based on soil pH
    double optimal_ph = 6.5;
    double ph_deviation = std::abs(sample.ph - optimal_ph);
    if (ph_deviation > 1.0) {
        rate_modifier += ph_deviation * 0.1;
    }
    
    double final_rate = config_.base_application_rate * rate_modifier;
    
    // Clamp to reasonable bounds
    final_rate = std::max(final_rate, 10.0);  // Minimum 10 kg/ha
    final_rate = std::min(final_rate, 200.0); // Maximum 200 kg/ha
    
    return final_rate;
}

bool FertilizerMission::checkSoilConditions() {
    auto sample = soil_analyzer_->getCurrentSample();
    
    // Check soil moisture
    if (sample.moisture_content < 10.0) {
        RCLCPP_WARN(get_logger(), "Soil too dry for fertilizer application");
        return false;
    }
    
    // Check soil temperature
    if (sample.temperature < 5.0 || sample.temperature > 35.0) {
        RCLCPP_WARN(get_logger(), "Soil temperature out of range");
        return false;
    }
    
    state_.soil_conditions_ok = true;
    return true;
}

bool FertilizerMission::checkWeatherConditions() {
    auto weather = weather_station_->getCurrentConditions();
    
    // Check wind speed
    if (weather.wind_speed > config_.max_wind_speed) {
        RCLCPP_WARN(get_logger(), "Wind speed too high: %.2f m/s", weather.wind_speed);
        return false;
    }
    
    // Check for precipitation
    if (weather.precipitation_rate > 0.1) {  // mm/hr
        RCLCPP_WARN(get_logger(), "Precipitation detected");
        return false;
    }
    
    state_.weather_conditions_ok = true;
    return true;
}

// Register the mission
REGISTER_MISSION(FertilizerMission, "fertilizer_application");

} // namespace system_controller
```

## Implementing New Policies

Policies control operating modes, safety filtering, and mission authorization.

### Step 1: Create Policy Class

Example for a field operation safety policy:

```cpp
// src/system_controller/include/policies/field_safety_policy.hpp
#ifndef FIELD_SAFETY_POLICY_HPP
#define FIELD_SAFETY_POLICY_HPP

#include "policy_base_expanded.hpp"
#include "gps_monitor.hpp"
#include "obstacle_detector.hpp"
#include "geofence_manager.hpp"

namespace system_controller {

class FieldSafetyPolicy : public PolicyBaseExpanded {
public:
    FieldSafetyPolicy();
    
    // Mode management
    bool requestModeChange(PolicyStateInfo::OperatingMode mode) override;
    bool canTransitionTo(PolicyStateInfo::OperatingMode target) const override;
    
    // Command filtering
    CommandFilterResult filterVehicleCommand(const VehicleCommand& command) override;
    CommandFilterResult filterImplementCommand(const ImplementCommand& command) override;
    
    // Mission policies
    bool isMissionAllowed(const std::string& mission_type) const override;
    bool isMissionParameterAllowed(const std::string& mission_type, 
                                  const std::string& parameter,
                                  const std::string& value) const override;
    
    // Safety monitoring
    void updateSafetyStatus() override;

private:
    // Safety checks
    bool checkGPSIntegrity();
    bool checkObstacleField();
    bool checkGeofenceBoundaries();
    bool checkOperatorPresence();
    bool checkVehicleHealth();
    bool checkImplementHealth();
    
    // Speed limiting based on conditions
    double calculateSafeMaxSpeed() const;
    double calculateTurnSpeed() const;
    
    // Field-specific safety
    bool checkFieldBoundaries(const VehicleCommand& command);
    bool checkSafeDistanceToObstacles(const VehicleCommand& command);
    bool validateImplementOperation(const ImplementCommand& command);
    
    // Configuration
    struct FieldSafetyConfig {
        double max_speed_near_obstacles;    // m/s
        double obstacle_detection_range;    // meters
        double boundary_buffer_distance;    // meters
        double min_gps_accuracy;           // meters
        bool require_operator_presence;
        double max_implement_height;       // meters
        double safe_turning_radius;        // meters
    } config_;
    
    // Safety state
    struct SafetyState {
        bool gps_healthy;
        bool obstacles_detected;
        bool within_boundaries;
        bool operator_present;
        bool vehicle_healthy;
        bool implements_healthy;
        double nearest_obstacle_distance;
        double current_gps_accuracy;
    } safety_state_;
    
    // Helper objects
    std::unique_ptr<GPSMonitor> gps_monitor_;
    std::unique_ptr<ObstacleDetector> obstacle_detector_;
    std::unique_ptr<GeofenceManager> geofence_manager_;
};

} // namespace system_controller

#endif
```

### Step 2: Implement Policy Logic

```cpp
// src/system_controller/src/policies/field_safety_policy.cpp
CommandFilterResult FieldSafetyPolicy::filterVehicleCommand(const VehicleCommand& command) {
    CommandFilterResult result;
    result.allowed = true;
    
    // Always allow emergency stop
    if (command.emergency_stop) {
        return result;
    }
    
    // Update safety status
    updateSafetyStatus();
    
    // Check GPS integrity
    if (!safety_state_.gps_healthy) {
        result.allowed = false;
        result.reason = "GPS signal integrity compromised";
        return result;
    }
    
    // Check boundary constraints
    if (!checkFieldBoundaries(command)) {
        result.allowed = false;
        result.reason = "Command would violate field boundaries";
        return result;
    }
    
    // Check obstacle proximity
    if (!checkSafeDistanceToObstacles(command)) {
        // Don't reject, but modify speed
        VehicleCommand modified = command;
        modified.velocity = std::min(command.velocity, config_.max_speed_near_obstacles);
        result.modified_vehicle_command = std::make_unique<VehicleCommand>(modified);
        result.reason = "Speed reduced due to obstacles";
        addSafetyEvent("Speed limited due to obstacle proximity");
    }
    
    // Enforce maximum safe speed
    double max_safe_speed = calculateSafeMaxSpeed();
    if (command.velocity > max_safe_speed) {
        VehicleCommand modified = command;
        modified.velocity = max_safe_speed;
        result.modified_vehicle_command = std::make_unique<VehicleCommand>(modified);
        result.reason = "Speed limited for safety";
    }
    
    // Check operator presence (if required)
    if (config_.require_operator_presence && !safety_state_.operator_present) {
        result.allowed = false;
        result.reason = "Operator presence required";
        return result;
    }
    
    return result;
}

CommandFilterResult FieldSafetyPolicy::filterImplementCommand(const ImplementCommand& command) {
    CommandFilterResult result;
    result.allowed = true;
    
    // Validate implement operation safety
    if (!validateImplementOperation(command)) {
        result.allowed = false;
        result.reason = "Implement operation not safe in current conditions";
        return result;
    }
    
    // Check implement health
    if (!safety_state_.implements_healthy) {
        result.allowed = false;
        result.reason = "Implement health check failed";
        return result;
    }
    
    // Prevent activation near obstacles
    if (command.activate && safety_state_.obstacles_detected) {
        if (safety_state_.nearest_obstacle_distance < 5.0) {  // 5 meter safety buffer
            result.allowed = false;
            result.reason = "Cannot activate implement near obstacles";
            return result;
        }
    }
    
    return result;
}

bool FieldSafetyPolicy::isMissionAllowed(const std::string& mission_type) const {
    // Check current operating mode
    auto current_mode = getCurrentMode();
    
    // Only allow certain missions in certain modes
    if (current_mode == PolicyStateInfo::TELEOP_ONLY) {
        // In teleop mode, only allow manual missions
        return mission_type == "manual_spraying" || 
               mission_type == "manual_mowing" ||
               mission_type == "manual_seeding";
    }
    
    if (current_mode == PolicyStateInfo::SAFETY_OVERRIDE) {
        // In safety override, no autonomous missions allowed
        return false;
    }
    
    // Check mission-specific requirements
    if (mission_type == "autonomous_spraying") {
        return safety_state_.gps_healthy && 
               safety_state_.within_boundaries &&
               !safety_state_.obstacles_detected;
    }
    
    return true;
}

double FieldSafetyPolicy::calculateSafeMaxSpeed() const {
    double base_max_speed = 15.0;  // m/s base maximum
    
    // Reduce speed based on conditions
    if (safety_state_.obstacles_detected) {
        base_max_speed = std::min(base_max_speed, config_.max_speed_near_obstacles);
    }
    
    // Reduce speed in poor GPS conditions
    if (safety_state_.current_gps_accuracy > 0.1) {  // > 10cm
        base_max_speed *= 0.7;  // Reduce by 30%
    }
    
    // Reduce speed near boundaries
    double distance_to_boundary = geofence_manager_->getDistanceToBoundary();
    if (distance_to_boundary < config_.boundary_buffer_distance) {
        double speed_factor = distance_to_boundary / config_.boundary_buffer_distance;
        base_max_speed *= std::max(speed_factor, 0.3);  // Minimum 30% speed
    }
    
    return base_max_speed;
}

void FieldSafetyPolicy::updateSafetyStatus() {
    safety_state_.gps_healthy = checkGPSIntegrity();
    safety_state_.obstacles_detected = checkObstacleField();
    safety_state_.within_boundaries = checkGeofenceBoundaries();
    safety_state_.operator_present = checkOperatorPresence();
    safety_state_.vehicle_healthy = checkVehicleHealth();
    safety_state_.implements_healthy = checkImplementHealth();
    
    if (safety_state_.obstacles_detected) {
        safety_state_.nearest_obstacle_distance = obstacle_detector_->getNearestObstacleDistance();
    }
    
    safety_state_.current_gps_accuracy = gps_monitor_->getCurrentAccuracy();
}

// Register the policy
REGISTER_POLICY(FieldSafetyPolicy, "field_safety");

} // namespace system_controller
```

## Configuration Management

### Vehicle Adapter Configuration

```yaml
# src/system_controller/config/vehicle_adapters.yaml
vehicle_adapters:
  john_deere_8r:
    adapter_type: "john_deere_can"
    adapter_id: "jd8r_001"
    communication_port: "/dev/can0"
    can_bitrate: 250000
    heartbeat_interval: 100  # ms
    scaling_factors:
      steering_scale: 32767
      velocity_scale: 100
      acceleration_scale: 100
    safety_limits:
      max_steering_rate: 30.0  # deg/s
      max_velocity: 20.0       # m/s
      max_acceleration: 2.0    # m/s²
    custom_params:
      implement_lift_can_id: "0x123"
      hydraulic_pressure_can_id: "0x124"
      
  case_ih_magnum:
    adapter_type: "case_ih_modbus"
    adapter_id: "cih_magnum_001"
    communication_port: "/dev/ttyUSB0"
    modbus_slave_id: 1
    baud_rate: 9600
    data_bits: 8
    stop_bits: 1
    parity: "none"
    scaling_factors:
      steering_scale: 1000
      velocity_scale: 10
    register_mapping:
      steering_register: 40001
      velocity_register: 40002
      status_register: 30001
```

### Mission Configuration

```yaml
# src/system_controller/config/missions.yaml
missions:
  precision_spraying:
    mission_type: "sprayer_mission"
    required_implements: ["boom_sprayer"]
    default_config:
      base_application_rate: 150.0  # L/ha
      boom_width: 12.0              # meters
      min_gps_accuracy: 0.05        # meters
      max_wind_speed: 15.0          # m/s
      chemical_type: "herbicide"
      environmental_monitoring: true
    safety_constraints:
      buffer_zones:
        - type: "waterway"
          distance: 5.0  # meters
        - type: "residential"
          distance: 100.0  # meters
      weather_limits:
        max_wind_speed: 15.0
        min_temperature: 5.0
        max_temperature: 35.0
        
  autonomous_mowing:
    mission_type: "mower_mission" 
    required_implements: ["rotary_mower"]
    default_config:
      cutting_height: 0.05          # meters
      cutting_pattern: "stripe"
      overlap_percentage: 10.0
      blade_speed: 3000             # RPM
      terrain_following: true
    safety_constraints:
      operator_presence_required: true
      max_slope: 15.0               # degrees
      min_obstacle_distance: 2.0   # meters
      
  variable_rate_seeding:
    mission_type: "seeder_mission"
    required_implements: ["precision_seeder"]
    default_config:
      base_seed_rate: 60000         # seeds/ha
      planting_depth: 0.025         # meters
      row_spacing: 0.75             # meters
      seed_type: "corn"
      variable_rate_enabled: true
    soil_requirements:
      min_moisture: 15.0            # percentage
      max_moisture: 35.0            # percentage
      min_temperature: 8.0          # celsius
      optimal_ph_range: [6.0, 7.5]
```

### Policy Configuration

```yaml
# src/system_controller/config/policies.yaml
policies:
  teleop_only:
    policy_type: "teleop_only_policy"
    config:
      require_dead_man_switch: true
      operator_timeout: 5.0         # seconds
      max_speed_without_operator: 2.0  # m/s
      emergency_stop_on_timeout: true
      
  field_safety:
    policy_type: "field_safety_policy"
    config:
      max_speed_near_obstacles: 3.0    # m/s
      obstacle_detection_range: 10.0   # meters
      boundary_buffer_distance: 5.0    # meters
      min_gps_accuracy: 0.1            # meters
      require_operator_presence: false
      max_implement_height: 2.0        # meters
      safe_turning_radius: 10.0        # meters
      
  adas_assistance:
    policy_type: "adas_policy"
    config:
      collision_avoidance_enabled: true
      lane_keeping_enabled: true
      adaptive_cruise_enabled: true
      tree_detection_enabled: true
      gps_monitoring_enabled: true
      intervention_thresholds:
        collision_distance: 5.0       # meters
        lane_deviation: 0.5           # meters
        gps_accuracy_limit: 0.2       # meters
```

## Testing and Validation

### Unit Tests

Create comprehensive unit tests for your components:

```cpp
// test/test_fertilizer_adapter.cpp
#include <gtest/gtest.h>
#include "fertilizer_adapter.hpp"

class FertilizerAdapterTest : public ::testing::Test {
protected:
    void SetUp() override {
        adapter_ = std::make_unique<FertilizerAdapter>();
        
        // Configure test adapter
        ImplementAdapterConfig config;
        config.adapter_id = "test_fertilizer";
        config.custom_params["max_rate"] = "200.0";
        config.custom_params["hopper_capacity"] = "1000.0";
        
        ASSERT_TRUE(adapter_->initialize(config));
    }
    
    std::unique_ptr<FertilizerAdapter> adapter_;
};

TEST_F(FertilizerAdapterTest, ApplicationRateValidation) {
    // Test valid rate
    EXPECT_TRUE(adapter_->setApplicationRate(50.0));
    
    // Test invalid rates
    EXPECT_FALSE(adapter_->setApplicationRate(-10.0));  // Negative
    EXPECT_FALSE(adapter_->setApplicationRate(500.0));  // Too high
}

TEST_F(FertilizerAdapterTest, HopperLevelMonitoring) {
    // Mock hopper level
    adapter_->setMockHopperLevel(80.0);
    EXPECT_TRUE(adapter_->checkHopperLevel());
    
    // Test low level
    adapter_->setMockHopperLevel(3.0);
    EXPECT_FALSE(adapter_->checkHopperLevel());
}

TEST_F(FertilizerAdapterTest, CommandProcessing) {
    ImplementCommand cmd;
    cmd.implement_type = "fertilizer";
    cmd.activate = true;
    cmd.custom_params["application_rate"] = "75.0";
    
    adapter_->setMockHopperLevel(50.0);  // Sufficient fertilizer
    EXPECT_TRUE(adapter_->processImplementCommand(cmd));
}
```

### Integration Tests

Test your components with the full system:

```cpp
// test/test_mission_integration.cpp
TEST(MissionIntegrationTest, SprayerMissionLifecycle) {
    // Setup system components
    auto vehicle_adapter = std::make_shared<MockVehicleAdapter>();
    auto sprayer_adapter = std::make_shared<MockSprayerAdapter>();
    auto mission = std::make_unique<SprayerMission>();
    
    // Configure mission
    MissionConfig config;
    config.mission_type = "sprayer_mission";
    config.required_implements = {"boom_sprayer"};
    ASSERT_TRUE(mission->initialize(config));
    
    // Mock good conditions
    sprayer_adapter->setMockWeatherConditions(WeatherConditions{
        .wind_speed = 8.0,      // m/s - acceptable
        .temperature = 22.0,    // celsius - good
        .humidity = 65.0        // percentage - good
    });
    
    // Test mission lifecycle
    EXPECT_TRUE(mission->start());
    EXPECT_TRUE(mission->startNewRow());
    
    // Simulate row completion
    mission->simulateRowProgress(100.0);  // 100% complete
    EXPECT_TRUE(mission->endCurrentRow());
    
    EXPECT_TRUE(mission->stop());
}

TEST(PolicyIntegrationTest, SafetyPolicyEnforcement) {
    auto policy = std::make_unique<FieldSafetyPolicy>();
    
    // Configure policy
    PolicyConfig config;
    config.policy_type = "field_safety";
    ASSERT_TRUE(policy->initialize(config));
    
    // Test command filtering
    VehicleCommand high_speed_cmd;
    high_speed_cmd.velocity = 25.0;  // Very high speed
    
    auto result = policy->filterVehicleCommand(high_speed_cmd);
    
    EXPECT_TRUE(result.allowed);
    ASSERT_TRUE(result.modified_vehicle_command);
    EXPECT_LT(result.modified_vehicle_command->velocity, high_speed_cmd.velocity);
}
```

### Hardware-in-the-Loop Testing

Test with real hardware using mock interfaces:

```cpp
// test/test_hardware_integration.cpp
class HardwareIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Setup hardware interfaces
        if (hasRealHardware()) {
            vehicle_adapter_ = std::make_unique<PolarisAdapter>();
            sprayer_adapter_ = std::make_unique<SprayerAdapter>();
        } else {
            vehicle_adapter_ = std::make_unique<MockVehicleAdapter>();
            sprayer_adapter_ = std::make_unique<MockSprayerAdapter>();
        }
    }
    
    bool hasRealHardware() {
        return std::getenv("TEST_WITH_HARDWARE") != nullptr;
    }
    
    std::unique_ptr<VehicleAdapterBase> vehicle_adapter_;
    std::unique_ptr<ImplementAdapterBase> sprayer_adapter_;
};

TEST_F(HardwareIntegrationTest, BasicVehicleControl) {
    if (!hasRealHardware()) {
        GTEST_SKIP() << "Skipping hardware test - no hardware available";
    }
    
    ASSERT_TRUE(vehicle_adapter_->connect());
    
    VehicleCommand cmd;
    cmd.steering_angle = 0.1;  // Small steering input
    cmd.velocity = 1.0;        // Low speed
    
    EXPECT_TRUE(vehicle_adapter_->processVehicleCommand(cmd));
    
    // Allow time for command execution
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    auto status = vehicle_adapter_->getStatus();
    EXPECT_TRUE(status.is_healthy);
}
```

## Real-World Examples

### Example 1: John Deere Integration

```cpp
// Complete John Deere adapter with ISOBUS communication
class JohnDeereAdapter : public VehicleAdapterBase {
public:
    JohnDeereAdapter() : VehicleAdapterBase("john_deere_adapter") {
        isobus_handler_ = std::make_unique<IsoBusHandler>();
    }
    
    bool initialize(const VehicleAdapterConfig& config) override {
        // Configure ISOBUS parameters
        IsoBusConfig isobus_config;
        isobus_config.can_interface = config.custom_params.at("can_interface");
        isobus_config.ecu_address = std::stoi(config.custom_params.at("ecu_address"));
        isobus_config.implement_address = std::stoi(config.custom_params.at("implement_address"));
        
        return isobus_handler_->initialize(isobus_config);
    }
    
    bool processVehicleCommand(const VehicleCommand& command) override {
        // Translate to ISOBUS PGN messages
        IsoBusMessage steering_msg;
        steering_msg.pgn = 0xFE49;  // Electronic Engine Controller 2
        steering_msg.data = convertSteeringToIsobus(command.steering_angle);
        
        IsoBusMessage velocity_msg;
        velocity_msg.pgn = 0xFEF1;  // Cruise Control/Vehicle Speed
        velocity_msg.data = convertVelocityToIsobus(command.velocity);
        
        return isobus_handler_->sendMessage(steering_msg) &&
               isobus_handler_->sendMessage(velocity_msg);
    }

private:
    std::unique_ptr<IsoBusHandler> isobus_handler_;
    
    std::vector<uint8_t> convertSteeringToIsobus(double angle) {
        // Convert -1.0 to 1.0 range to ISOBUS steering format
        int16_t isobus_angle = static_cast<int16_t>(angle * 32767);
        return {static_cast<uint8_t>(isobus_angle & 0xFF),
                static_cast<uint8_t>((isobus_angle >> 8) & 0xFF)};
    }
};
```

### Example 2: Case IH with Advanced Farming Systems

```cpp
// Case IH adapter with AFS (Advanced Farming Systems) integration
class CaseIHAdapter : public VehicleAdapterBase {
public:
    bool processVehicleCommand(const VehicleCommand& command) override {
        // Convert to AFS protocol messages
        AFSMessage afs_msg;
        
        // Steering control via AFS AccuGuide
        afs_msg.message_id = AFS_STEERING_CONTROL;
        afs_msg.steering_angle = convertSteeringToAFS(command.steering_angle);
        afs_msg.steering_rate = calculateSteeringRate(command);
        
        // Speed control via AFS AutoSpeed
        afs_msg.target_speed = convertSpeedToAFS(command.velocity);
        afs_msg.speed_control_mode = AFS_AUTO_SPEED_MODE;
        
        return afs_handler_->sendMessage(afs_msg);
    }

private:
    std::unique_ptr<AFSHandler> afs_handler_;
    
    double convertSteeringToAFS(double normalized_angle) {
        // Convert to AFS steering wheel angle (degrees)
        return normalized_angle * max_steering_angle_;
    }
    
    double calculateSteeringRate(const VehicleCommand& command) {
        // Calculate steering rate based on current vs target
        double current_angle = getCurrentSteeringAngle();
        double target_angle = convertSteeringToAFS(command.steering_angle);
        double angle_diff = target_angle - current_angle;
        
        // Limit steering rate for smooth operation
        return std::clamp(angle_diff / 0.1, -30.0, 30.0);  // deg/s
    }
};
```

### Example 3: Complete Precision Spraying System

```cpp
// Complete precision spraying implementation with all safety features
class PrecisionSprayingMission : public MissionBase {
public:
    bool startNewRow() override {
        // Comprehensive pre-row safety checks
        if (!performPreRowSafetyChecks()) {
            return false;
        }
        
        // Calculate variable rate for this row section
        auto prescription_map = field_manager_->getPrescriptionMap();
        auto current_position = gps_->getCurrentPosition();
        double optimal_rate = prescription_map.getRateAtPosition(current_position);
        
        // Configure sprayer for row start
        SprayerCommand spray_cmd;
        spray_cmd.base.activate = true;
        spray_cmd.base.lower = true;
        spray_cmd.application_rate = optimal_rate;
        spray_cmd.chemical_type = mission_config_.chemical_type;
        spray_cmd.environmental_monitoring = true;
        
        // Activate boom sections based on field mapping
        auto boom_config = calculateBoomConfiguration(current_position);
        spray_cmd.boom_sections = boom_config;
        
        auto sprayer = getSprayerAdapter();
        if (!sprayer->processSprayerCommand(spray_cmd)) {
            setError("Failed to configure sprayer for row start");
            return false;
        }
        
        // Start row monitoring
        startRowMonitoring();
        
        return true;
    }
    
    void onMissionTick() override {
        // Continuous monitoring during operation
        checkWeatherConditions();
        checkGPSAccuracy();
        checkTreeDetection();
        checkBufferZones();
        adjustApplicationRate();
        monitorSprayQuality();
    }

private:
    bool performPreRowSafetyChecks() {
        // Weather check
        auto weather = weather_station_->getCurrentConditions();
        if (weather.wind_speed > mission_config_.max_wind_speed) {
            pauseMission("Wind speed exceeds limit: " + std::to_string(weather.wind_speed));
            return false;
        }
        
        // GPS accuracy check
        if (gps_->getCurrentAccuracy() > mission_config_.min_gps_accuracy) {
            pauseMission("GPS accuracy insufficient");
            return false;
        }
        
        // Chemical level check
        auto sprayer = getSprayerAdapter();
        if (sprayer->getChemicalLevel() < 10.0) {  // 10% minimum
            pauseMission("Chemical level too low");
            return false;
        }
        
        // Tree detection check
        if (tree_detector_->detectTrees()) {
            tree_pause_active_ = true;
            pauseMission("Trees detected - pausing spray");
            return false;
        }
        
        return true;
    }
    
    void checkTreeDetection() {
        bool trees_detected = tree_detector_->detectTrees();
        
        if (trees_detected && !tree_pause_active_) {
            // Trees newly detected - pause spraying
            auto sprayer = getSprayerAdapter();
            SprayerCommand pause_cmd;
            pause_cmd.base.activate = false;  // Stop spraying
            sprayer->processSprayerCommand(pause_cmd);
            
            tree_pause_active_ = true;
            RCLCPP_WARN(get_logger(), "Trees detected - spray paused");
            
        } else if (!trees_detected && tree_pause_active_) {
            // Trees cleared - resume spraying
            auto sprayer = getSprayerAdapter();
            SprayerCommand resume_cmd;
            resume_cmd.base.activate = true;  // Resume spraying
            sprayer->processSprayerCommand(resume_cmd);
            
            tree_pause_active_ = false;
            RCLCPP_INFO(get_logger(), "Trees cleared - spray resumed");
        }
    }
    
    std::shared_ptr<SprayerAdapter> getSprayerAdapter() {
        auto implement = getImplementAdapter("boom_sprayer");
        return std::dynamic_pointer_cast<SprayerAdapter>(implement);
    }
    
    bool tree_pause_active_ = false;
    std::unique_ptr<WeatherStation> weather_station_;
    std::unique_ptr<GPSMonitor> gps_;
    std::unique_ptr<TreeDetector> tree_detector_;
    std::unique_ptr<FieldManager> field_manager_;
};
```

## Best Practices

### 1. Safety-First Design
- Always implement emergency stop functionality
- Use multiple layers of safety checks
- Fail safe - default to stopping/disabling on errors
- Log all safety events for analysis

### 2. Robust Error Handling
```cpp
bool MyAdapter::processCommand(const Command& command) {
    try {
        // Validate inputs
        if (!validateCommand(command)) {
            setError("Invalid command parameters");
            return false;
        }
        
        // Execute with timeout
        auto future = std::async(std::launch::async, [this, command]() {
            return executeCommand(command);
        });
        
        if (future.wait_for(std::chrono::milliseconds(100)) == std::future_status::timeout) {
            setError("Command execution timeout");
            return false;
        }
        
        return future.get();
        
    } catch (const std::exception& e) {
        setError("Exception in command processing: " + std::string(e.what()));
        return false;
    }
}
```

### 3. Comprehensive Logging
```cpp
void logMissionEvent(const std::string& event, const std::map<std::string, double>& data) {
    RCLCPP_INFO(get_logger(), "Mission Event: %s", event.c_str());
    
    for (const auto& [key, value] : data) {
        RCLCPP_INFO(get_logger(), "  %s: %.3f", key.c_str(), value);
    }
    
    // Also log to file for analysis
    mission_logger_->logEvent(event, data);
}
```

### 4. Configuration Validation
```cpp
bool validateMissionConfig(const MissionConfig& config) {
    if (config.application_rate <= 0.0) {
        RCLCPP_ERROR(get_logger(), "Invalid application rate: %.2f", config.application_rate);
        return false;
    }
    
    if (config.max_wind_speed < 0.0 || config.max_wind_speed > 50.0) {
        RCLCPP_ERROR(get_logger(), "Invalid wind speed limit: %.2f", config.max_wind_speed);
        return false;
    }
    
    // Validate required implements exist
    for (const auto& implement : config.required_implements) {
        if (!isImplementAvailable(implement)) {
            RCLCPP_ERROR(get_logger(), "Required implement not available: %s", implement.c_str());
            return false;
        }
    }
    
    return true;
}
```

### 5. Thread Safety
```cpp
class ThreadSafeAdapter {
private:
    mutable std::shared_mutex status_mutex_;
    AdapterStatus status_;

public:
    AdapterStatus getStatus() const {
        std::shared_lock<std::shared_mutex> lock(status_mutex_);
        return status_;
    }
    
    void updateStatus(const AdapterStatus& new_status) {
        std::unique_lock<std::shared_mutex> lock(status_mutex_);
        status_ = new_status;
    }
};
```

### 6. Testing Strategy
- Unit tests for each component
- Integration tests for component interactions
- Hardware-in-the-loop tests with real equipment
- Safety scenario testing
- Performance benchmarking
- Field validation with operators

---

This guide provides a comprehensive foundation for developing agricultural automation systems with the ROS 2 System Controller. Each component is designed for safety, reliability, and extensibility in real-world farming operations. 