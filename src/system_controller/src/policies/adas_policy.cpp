#include "policies/adas_policy.hpp"
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <algorithm>
#include <cmath>

using namespace std::chrono_literals;

namespace system_controller {

AdasPolicy::AdasPolicy() 
    : PolicyBaseExpanded("adas_policy", "Advanced Driver Assistance Policy")
{
    policy_type_ = "adas_assisted";
    operating_mode_ = OperatingMode::ADAS_ASSISTED;
}

bool AdasPolicy::initialize(const PolicyConfig& config) 
{
    if (!PolicyBaseExpanded::initialize(config)) {
        return false;
    }
    
    // Load ADAS-specific configuration
    if (config.policy_params.count("collision_avoidance_enabled")) {
        adas_config_.collision_avoidance_enabled = (config.policy_params.at("collision_avoidance_enabled") == "true");
    }
    if (config.policy_params.count("lane_keeping_enabled")) {
        adas_config_.lane_keeping_enabled = (config.policy_params.at("lane_keeping_enabled") == "true");
    }
    if (config.policy_params.count("adaptive_cruise_enabled")) {
        adas_config_.adaptive_cruise_enabled = (config.policy_params.at("adaptive_cruise_enabled") == "true");
    }
    if (config.policy_params.count("tree_detection_enabled")) {
        adas_config_.tree_detection_enabled = (config.policy_params.at("tree_detection_enabled") == "true");
    }
    if (config.policy_params.count("gps_monitoring_enabled")) {
        adas_config_.gps_monitoring_enabled = (config.policy_params.at("gps_monitoring_enabled") == "true");
    }
    if (config.policy_params.count("speed_limiting_enabled")) {
        adas_config_.speed_limiting_enabled = (config.policy_params.at("speed_limiting_enabled") == "true");
    }
    if (config.policy_params.count("collision_warning_distance")) {
        adas_config_.collision_warning_distance = std::stod(config.policy_params.at("collision_warning_distance"));
    }
    if (config.policy_params.count("emergency_brake_distance")) {
        adas_config_.emergency_brake_distance = std::stod(config.policy_params.at("emergency_brake_distance"));
    }
    if (config.policy_params.count("max_assisted_speed")) {
        adas_config_.max_assisted_speed = std::stod(config.policy_params.at("max_assisted_speed"));
    }
    if (config.policy_params.count("gps_accuracy_threshold")) {
        adas_config_.gps_accuracy_threshold = std::stod(config.policy_params.at("gps_accuracy_threshold"));
    }
    
    // Initialize sensor data
    sensor_data_.collision_distance = 100.0;  // No obstacles initially
    sensor_data_.lane_position = 0.0;         // Centered in lane
    sensor_data_.gps_accuracy = 0.05;         // 5cm accuracy
    sensor_data_.trees_detected = false;
    sensor_data_.obstacles_detected = false;
    sensor_data_.lane_detected = true;
    
    // Initialize ADAS state
    adas_state_.collision_warning_active = false;
    adas_state_.emergency_braking_active = false;
    adas_state_.lane_keeping_active = false;
    adas_state_.adaptive_cruise_active = false;
    adas_state_.tree_avoidance_active = false;
    adas_state_.gps_error_detected = false;
    adas_state_.speed_limited = false;
    adas_state_.assistance_level = 0.0;
    
    // Initialize intervention tracking
    intervention_stats_.collision_interventions = 0;
    intervention_stats_.lane_corrections = 0;
    intervention_stats_.speed_corrections = 0;
    intervention_stats_.tree_avoidances = 0;
    intervention_stats_.gps_error_stops = 0;
    
    RCLCPP_INFO(rclcpp::get_logger("adas_policy"), "AdasPolicy initialized - Intelligent assistance enabled");
    return true;
}

VehicleCommand AdasPolicy::processVehicleCommand(const VehicleCommand& input_command) 
{
    VehicleCommand assisted_command = input_command;
    
    // Update sensor data
    updateSensorData();
    
    // Apply collision avoidance
    if (adas_config_.collision_avoidance_enabled) {
        assisted_command = applyCollisionAvoidance(assisted_command);
    }
    
    // Apply lane keeping assistance
    if (adas_config_.lane_keeping_enabled && sensor_data_.lane_detected) {
        assisted_command = applyLaneKeeping(assisted_command);
    }
    
    // Apply adaptive cruise control
    if (adas_config_.adaptive_cruise_enabled) {
        assisted_command = applyAdaptiveCruise(assisted_command);
    }
    
    // Apply tree detection and avoidance
    if (adas_config_.tree_detection_enabled) {
        assisted_command = applyTreeAvoidance(assisted_command);
    }
    
    // Apply GPS monitoring
    if (adas_config_.gps_monitoring_enabled) {
        assisted_command = applyGPSMonitoring(assisted_command);
    }
    
    // Apply speed limiting
    if (adas_config_.speed_limiting_enabled) {
        assisted_command = applySpeedLimiting(assisted_command);
    }
    
    // Calculate assistance level
    calculateAssistanceLevel(input_command, assisted_command);
    
    // Log interventions
    logInterventions(input_command, assisted_command);
    
    return assisted_command;
}

ImplementCommand AdasPolicy::processImplementCommand(const ImplementCommand& input_command) 
{
    ImplementCommand assisted_command = input_command;
    
    // Apply implement safety monitoring
    assisted_command = applyImplementSafety(assisted_command);
    
    // Apply tree-based implement control
    if (adas_config_.tree_detection_enabled && sensor_data_.trees_detected) {
        assisted_command = applyTreeBasedImplementControl(assisted_command);
    }
    
    // Apply GPS-based implement control
    if (adas_config_.gps_monitoring_enabled && adas_state_.gps_error_detected) {
        assisted_command = applyGPSBasedImplementControl(assisted_command);
    }
    
    return assisted_command;
}

void AdasPolicy::update() 
{
    // Update sensor data
    updateSensorData();
    
    // Update ADAS states
    updateAdasStates();
    
    // Update assistance algorithms
    updateCollisionDetection();
    updateLaneDetection();
    updateTreeDetection();
    updateGPSMonitoring();
    
    // Update policy status
    updatePolicyStatus();
}

PolicyState AdasPolicy::getState() const 
{
    PolicyState state;
    state.operating_mode = static_cast<uint8_t>(operating_mode_);
    state.safety_level = calculateSafetyLevel();
    state.autonomous_enabled = false;  // ADAS assists but doesn't enable full autonomy
    state.emergency_stop_active = adas_state_.emergency_braking_active;
    
    // Add ADAS-specific safety flags
    state.safety_flags.clear();
    if (adas_state_.collision_warning_active) state.safety_flags.push_back("COLLISION_WARNING");
    if (adas_state_.emergency_braking_active) state.safety_flags.push_back("EMERGENCY_BRAKING");
    if (adas_state_.lane_keeping_active) state.safety_flags.push_back("LANE_KEEPING_ACTIVE");
    if (adas_state_.tree_avoidance_active) state.safety_flags.push_back("TREE_AVOIDANCE");
    if (adas_state_.gps_error_detected) state.safety_flags.push_back("GPS_ERROR");
    if (adas_state_.speed_limited) state.safety_flags.push_back("SPEED_LIMITED");
    
    return state;
}

bool AdasPolicy::handleEmergencyStop() 
{
    RCLCPP_ERROR(rclcpp::get_logger("adas_policy"), "Emergency stop activated in ADAS mode");
    
    adas_state_.emergency_braking_active = true;
    
    // ADAS can help with controlled emergency stop
    return true;
}

VehicleCommand AdasPolicy::applyCollisionAvoidance(const VehicleCommand& command) 
{
    VehicleCommand safe_command = command;
    
    if (sensor_data_.collision_distance < adas_config_.emergency_brake_distance) {
        // Emergency braking
        RCLCPP_WARN(rclcpp::get_logger("adas_policy"), 
                    "Emergency braking activated - obstacle at %.2fm", sensor_data_.collision_distance);
        
        safe_command.linear_velocity = 0.0;
        safe_command.emergency_stop = true;
        adas_state_.emergency_braking_active = true;
        intervention_stats_.collision_interventions++;
        
    } else if (sensor_data_.collision_distance < adas_config_.collision_warning_distance) {
        // Collision warning and speed reduction
        double reduction_factor = (sensor_data_.collision_distance - adas_config_.emergency_brake_distance) / 
                                 (adas_config_.collision_warning_distance - adas_config_.emergency_brake_distance);
        
        safe_command.linear_velocity *= reduction_factor;
        adas_state_.collision_warning_active = true;
        
        RCLCPP_DEBUG(rclcpp::get_logger("adas_policy"), 
                     "Collision warning - reducing speed by %.0f%%", (1.0 - reduction_factor) * 100);
        
    } else {
        adas_state_.collision_warning_active = false;
        adas_state_.emergency_braking_active = false;
    }
    
    return safe_command;
}

VehicleCommand AdasPolicy::applyLaneKeeping(const VehicleCommand& command) 
{
    VehicleCommand corrected_command = command;
    
    // Apply lane keeping correction if vehicle is drifting
    const double LANE_CORRECTION_THRESHOLD = 0.5;  // meters
    const double MAX_CORRECTION_ANGLE = 0.1;       // radians (~5.7 degrees)
    
    if (std::abs(sensor_data_.lane_position) > LANE_CORRECTION_THRESHOLD) {
        double correction = -sensor_data_.lane_position * 0.1;  // Proportional control
        correction = std::max(-MAX_CORRECTION_ANGLE, std::min(MAX_CORRECTION_ANGLE, correction));
        
        corrected_command.steering_angle += correction;
        adas_state_.lane_keeping_active = true;
        intervention_stats_.lane_corrections++;
        
        RCLCPP_DEBUG(rclcpp::get_logger("adas_policy"), 
                     "Lane keeping correction: %.2f° for position offset %.2fm",
                     correction * 180.0 / M_PI, sensor_data_.lane_position);
    } else {
        adas_state_.lane_keeping_active = false;
    }
    
    return corrected_command;
}

VehicleCommand AdasPolicy::applyAdaptiveCruise(const VehicleCommand& command) 
{
    VehicleCommand cruise_command = command;
    
    // Maintain safe following distance
    const double SAFE_FOLLOWING_DISTANCE = 20.0;  // meters
    const double TARGET_SPEED = 5.0;              // m/s
    
    if (sensor_data_.collision_distance < SAFE_FOLLOWING_DISTANCE && 
        sensor_data_.collision_distance > adas_config_.collision_warning_distance) {
        
        // Adjust speed to maintain safe distance
        double speed_factor = sensor_data_.collision_distance / SAFE_FOLLOWING_DISTANCE;
        double target_speed = TARGET_SPEED * speed_factor;
        
        if (command.linear_velocity > target_speed) {
            cruise_command.linear_velocity = target_speed;
            adas_state_.adaptive_cruise_active = true;
            
            RCLCPP_DEBUG(rclcpp::get_logger("adas_policy"), 
                         "Adaptive cruise: reducing speed to %.2f m/s for %.2fm distance",
                         target_speed, sensor_data_.collision_distance);
        }
    } else {
        adas_state_.adaptive_cruise_active = false;
    }
    
    return cruise_command;
}

VehicleCommand AdasPolicy::applyTreeAvoidance(const VehicleCommand& command) 
{
    VehicleCommand tree_safe_command = command;
    
    if (sensor_data_.trees_detected) {
        // Reduce speed when trees are detected
        const double TREE_SAFETY_SPEED = 2.0;  // m/s
        
        if (command.linear_velocity > TREE_SAFETY_SPEED) {
            tree_safe_command.linear_velocity = TREE_SAFETY_SPEED;
            adas_state_.tree_avoidance_active = true;
            intervention_stats_.tree_avoidances++;
            
            RCLCPP_INFO(rclcpp::get_logger("adas_policy"), 
                        "Trees detected - reducing speed to %.2f m/s for safety",
                        TREE_SAFETY_SPEED);
        }
    } else {
        adas_state_.tree_avoidance_active = false;
    }
    
    return tree_safe_command;
}

VehicleCommand AdasPolicy::applyGPSMonitoring(const VehicleCommand& command) 
{
    VehicleCommand gps_safe_command = command;
    
    if (sensor_data_.gps_accuracy > adas_config_.gps_accuracy_threshold) {
        // GPS accuracy insufficient - stop vehicle
        RCLCPP_WARN(rclcpp::get_logger("adas_policy"), 
                    "GPS accuracy insufficient: %.2fm (threshold: %.2fm) - stopping vehicle",
                    sensor_data_.gps_accuracy, adas_config_.gps_accuracy_threshold);
        
        gps_safe_command.linear_velocity = 0.0;
        gps_safe_command.angular_velocity = 0.0;
        adas_state_.gps_error_detected = true;
        intervention_stats_.gps_error_stops++;
    } else {
        adas_state_.gps_error_detected = false;
    }
    
    return gps_safe_command;
}

VehicleCommand AdasPolicy::applySpeedLimiting(const VehicleCommand& command) 
{
    VehicleCommand limited_command = command;
    
    if (command.linear_velocity > adas_config_.max_assisted_speed) {
        limited_command.linear_velocity = adas_config_.max_assisted_speed;
        adas_state_.speed_limited = true;
        intervention_stats_.speed_corrections++;
        
        RCLCPP_DEBUG(rclcpp::get_logger("adas_policy"), 
                     "Speed limited from %.2f to %.2f m/s",
                     command.linear_velocity, adas_config_.max_assisted_speed);
    } else {
        adas_state_.speed_limited = false;
    }
    
    return limited_command;
}

ImplementCommand AdasPolicy::applyImplementSafety(const ImplementCommand& command) 
{
    ImplementCommand safe_command = command;
    
    // Basic implement safety - emergency stop if vehicle stopped
    if (adas_state_.emergency_braking_active) {
        safe_command.activate = false;
        safe_command.emergency_stop = true;
        
        RCLCPP_INFO(rclcpp::get_logger("adas_policy"), 
                    "Vehicle emergency braking - stopping implement");
    }
    
    return safe_command;
}

ImplementCommand AdasPolicy::applyTreeBasedImplementControl(const ImplementCommand& command) 
{
    ImplementCommand tree_safe_command = command;
    
    if (sensor_data_.trees_detected) {
        // Pause implement operation when trees detected
        tree_safe_command.activate = false;
        
        RCLCPP_INFO(rclcpp::get_logger("adas_policy"), 
                    "Trees detected - pausing implement operation for safety");
    }
    
    return tree_safe_command;
}

ImplementCommand AdasPolicy::applyGPSBasedImplementControl(const ImplementCommand& command) 
{
    ImplementCommand gps_safe_command = command;
    
    if (adas_state_.gps_error_detected) {
        // Stop implement operation when GPS error detected
        gps_safe_command.activate = false;
        
        RCLCPP_WARN(rclcpp::get_logger("adas_policy"), 
                    "GPS error detected - stopping implement operation");
    }
    
    return gps_safe_command;
}

void AdasPolicy::updateSensorData() 
{
    // Simulate sensor data updates
    sensor_data_.collision_distance = simulateCollisionSensor();
    sensor_data_.lane_position = simulateLaneSensor();
    sensor_data_.gps_accuracy = simulateGPSAccuracy();
    sensor_data_.trees_detected = simulateTreeDetection();
    sensor_data_.obstacles_detected = sensor_data_.collision_distance < 50.0;
    sensor_data_.lane_detected = simulateLaneDetection();
}

void AdasPolicy::updateAdasStates() 
{
    // States are updated in individual apply methods
    // This method can be used for cross-system state updates
}

void AdasPolicy::updateCollisionDetection() 
{
    // Simulate collision detection algorithm
    RCLCPP_DEBUG(rclcpp::get_logger("adas_policy"), 
                 "Collision detection: %.2fm distance", sensor_data_.collision_distance);
}

void AdasPolicy::updateLaneDetection() 
{
    // Simulate lane detection algorithm
    if (sensor_data_.lane_detected) {
        RCLCPP_DEBUG(rclcpp::get_logger("adas_policy"), 
                     "Lane position: %.2fm from center", sensor_data_.lane_position);
    }
}

void AdasPolicy::updateTreeDetection() 
{
    // Simulate tree detection algorithm
    if (sensor_data_.trees_detected) {
        RCLCPP_DEBUG(rclcpp::get_logger("adas_policy"), "Trees detected in vicinity");
    }
}

void AdasPolicy::updateGPSMonitoring() 
{
    // Simulate GPS monitoring
    RCLCPP_DEBUG(rclcpp::get_logger("adas_policy"), 
                 "GPS accuracy: %.2fm", sensor_data_.gps_accuracy);
}

void AdasPolicy::calculateAssistanceLevel(const VehicleCommand& input, const VehicleCommand& output) 
{
    // Calculate how much the ADAS system is assisting
    double speed_assistance = std::abs(input.linear_velocity - output.linear_velocity) / 
                             std::max(0.1, std::abs(input.linear_velocity));
    double steering_assistance = std::abs(input.steering_angle - output.steering_angle) / 
                                std::max(0.1, std::abs(input.steering_angle + 0.1));
    
    adas_state_.assistance_level = std::max(speed_assistance, steering_assistance);
}

void AdasPolicy::logInterventions(const VehicleCommand& input, const VehicleCommand& output) 
{
    static int log_counter = 0;
    if (++log_counter % 1000 == 0) {  // Log every 1000 updates
        RCLCPP_INFO(rclcpp::get_logger("adas_policy"), 
                    "ADAS interventions - Collision: %d, Lane: %d, Speed: %d, Tree: %d, GPS: %d",
                    intervention_stats_.collision_interventions,
                    intervention_stats_.lane_corrections,
                    intervention_stats_.speed_corrections,
                    intervention_stats_.tree_avoidances,
                    intervention_stats_.gps_error_stops);
    }
}

uint8_t AdasPolicy::calculateSafetyLevel() const 
{
    // Calculate safety level based on active warnings and interventions
    uint8_t safety_level = 100;
    
    if (adas_state_.collision_warning_active) safety_level -= 30;
    if (adas_state_.emergency_braking_active) safety_level -= 50;
    if (adas_state_.gps_error_detected) safety_level -= 20;
    if (adas_state_.tree_avoidance_active) safety_level -= 10;
    
    return std::max(static_cast<uint8_t>(0), safety_level);
}

void AdasPolicy::updatePolicyStatus() 
{
    // Update overall policy health based on sensor status and interventions
    bool sensors_healthy = sensor_data_.gps_accuracy < adas_config_.gps_accuracy_threshold &&
                          sensor_data_.collision_distance > adas_config_.emergency_brake_distance;
    
    if (!sensors_healthy && policy_healthy_) {
        RCLCPP_WARN(rclcpp::get_logger("adas_policy"), "ADAS sensors unhealthy");
    } else if (sensors_healthy && !policy_healthy_) {
        RCLCPP_INFO(rclcpp::get_logger("adas_policy"), "ADAS sensors healthy");
    }
    
    policy_healthy_ = sensors_healthy;
}

// Simulation helper methods
double AdasPolicy::simulateCollisionSensor() const 
{
    // Simulate varying collision distances
    static double distance = 100.0;
    static int direction = -1;
    
    distance += direction * 0.5;
    if (distance < 5.0 || distance > 100.0) {
        direction *= -1;
    }
    
    return distance;
}

double AdasPolicy::simulateLaneSensor() const 
{
    // Simulate lane position with gentle oscillation
    static double time = 0.0;
    time += 0.01;
    return 0.3 * std::sin(time * 0.1);  // Oscillate ±30cm
}

double AdasPolicy::simulateGPSAccuracy() const 
{
    // Simulate GPS accuracy variations
    static double accuracy = 0.05;
    accuracy += (rand() % 21 - 10) * 0.001;  // ±1cm variation
    return std::max(0.01, std::min(0.5, accuracy));
}

bool AdasPolicy::simulateTreeDetection() const 
{
    // Simulate occasional tree detection
    return (rand() % 10000) < 50;  // 0.5% chance
}

bool AdasPolicy::simulateLaneDetection() const 
{
    // Simulate lane detection availability
    return (rand() % 100) > 5;  // 95% lane detection success rate
}

} // namespace system_controller 