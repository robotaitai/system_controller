#include "policies/teleop_only_policy.hpp"
#include <rclcpp/rclcpp.hpp>
#include <chrono>

using namespace std::chrono_literals;

namespace system_controller {

TeleopOnlyPolicy::TeleopOnlyPolicy() 
    : PolicyBaseExpanded("teleop_only_policy", "Manual Control Only Policy")
{
    policy_type_ = "teleop_only";
    operating_mode_ = OperatingMode::TELEOP_ONLY;
}

bool TeleopOnlyPolicy::initialize(const PolicyConfig& config) 
{
    if (!PolicyBaseExpanded::initialize(config)) {
        return false;
    }
    
    // Load teleop-specific configuration
    if (config.policy_params.count("operator_presence_timeout")) {
        teleop_config_.operator_presence_timeout = std::stod(config.policy_params.at("operator_presence_timeout"));
    }
    if (config.policy_params.count("dead_man_switch_required")) {
        teleop_config_.dead_man_switch_required = (config.policy_params.at("dead_man_switch_required") == "true");
    }
    if (config.policy_params.count("max_speed_limit")) {
        teleop_config_.max_speed_limit = std::stod(config.policy_params.at("max_speed_limit"));
    }
    if (config.policy_params.count("enable_geo_fencing")) {
        teleop_config_.enable_geo_fencing = (config.policy_params.at("enable_geo_fencing") == "true");
    }
    
    // Initialize safety state
    safety_state_.operator_present = false;
    safety_state_.dead_man_switch_active = false;
    safety_state_.last_operator_input = std::chrono::steady_clock::now();
    safety_state_.emergency_stop_active = false;
    safety_state_.geo_fence_violation = false;
    safety_state_.speed_limit_exceeded = false;
    
    // Initialize input monitoring
    input_state_.last_command_time = std::chrono::steady_clock::now();
    input_state_.command_rate = 0.0;
    input_state_.steering_input = 0.0;
    input_state_.throttle_input = 0.0;
    input_state_.brake_input = 0.0;
    input_state_.implement_input = false;
    
    RCLCPP_INFO(rclcpp::get_logger("teleop_only_policy"), "TeleopOnlyPolicy initialized - Manual control enforced");
    return true;
}

VehicleCommand TeleopOnlyPolicy::processVehicleCommand(const VehicleCommand& input_command) 
{
    VehicleCommand filtered_command = input_command;
    
    // Update input monitoring
    updateInputMonitoring(input_command);
    
    // Check operator safety requirements
    if (!checkOperatorSafety()) {
        RCLCPP_WARN(rclcpp::get_logger("teleop_only_policy"), "Operator safety check failed - stopping vehicle");
        return createEmergencyStopCommand();
    }
    
    // Check dead man switch if required
    if (teleop_config_.dead_man_switch_required && !safety_state_.dead_man_switch_active) {
        RCLCPP_WARN(rclcpp::get_logger("teleop_only_policy"), "Dead man switch not active - stopping vehicle");
        return createEmergencyStopCommand();
    }
    
    // Enforce speed limits
    if (input_command.linear_velocity > teleop_config_.max_speed_limit) {
        RCLCPP_WARN(rclcpp::get_logger("teleop_only_policy"), 
                    "Speed limit exceeded: %.2f m/s (max: %.2f m/s) - limiting speed",
                    input_command.linear_velocity, teleop_config_.max_speed_limit);
        filtered_command.linear_velocity = teleop_config_.max_speed_limit;
        safety_state_.speed_limit_exceeded = true;
    } else {
        safety_state_.speed_limit_exceeded = false;
    }
    
    // Check geo-fencing if enabled
    if (teleop_config_.enable_geo_fencing && checkGeofenceViolation()) {
        RCLCPP_ERROR(rclcpp::get_logger("teleop_only_policy"), "Geo-fence violation detected - stopping vehicle");
        return createEmergencyStopCommand();
    }
    
    // Enforce manual control - reject any autonomous commands
    if (hasAutonomousFlags(input_command)) {
        RCLCPP_WARN(rclcpp::get_logger("teleop_only_policy"), "Autonomous control attempted - rejecting command");
        return createSafeStopCommand();
    }
    
    // Validate command safety
    if (!validateCommandSafety(filtered_command)) {
        RCLCPP_WARN(rclcpp::get_logger("teleop_only_policy"), "Command safety validation failed");
        return createSafeStopCommand();
    }
    
    // Log command acceptance
    RCLCPP_DEBUG(rclcpp::get_logger("teleop_only_policy"), 
                 "Teleop command accepted - speed: %.2f m/s, steering: %.2f°",
                 filtered_command.linear_velocity, filtered_command.steering_angle * 180.0 / M_PI);
    
    return filtered_command;
}

ImplementCommand TeleopOnlyPolicy::processImplementCommand(const ImplementCommand& input_command) 
{
    ImplementCommand filtered_command = input_command;
    
    // Check operator safety for implement control
    if (!checkOperatorSafety()) {
        RCLCPP_WARN(rclcpp::get_logger("teleop_only_policy"), "Operator safety check failed - stopping implement");
        return createImplementEmergencyStop();
    }
    
    // Check dead man switch for implement operations
    if (teleop_config_.dead_man_switch_required && !safety_state_.dead_man_switch_active) {
        RCLCPP_WARN(rclcpp::get_logger("teleop_only_policy"), "Dead man switch not active - stopping implement");
        return createImplementEmergencyStop();
    }
    
    // Enforce manual implement control only
    if (hasAutonomousImplementFlags(input_command)) {
        RCLCPP_WARN(rclcpp::get_logger("teleop_only_policy"), "Autonomous implement control attempted - rejecting");
        return createImplementSafeStop();
    }
    
    // Validate implement command safety
    if (!validateImplementSafety(filtered_command)) {
        RCLCPP_WARN(rclcpp::get_logger("teleop_only_policy"), "Implement command safety validation failed");
        return createImplementSafeStop();
    }
    
    RCLCPP_DEBUG(rclcpp::get_logger("teleop_only_policy"), 
                 "Teleop implement command accepted - type: %s, activate: %s",
                 filtered_command.implement_type.c_str(),
                 filtered_command.activate ? "true" : "false");
    
    return filtered_command;
}

void TeleopOnlyPolicy::update() 
{
    // Update safety monitoring
    updateSafetyMonitoring();
    
    // Check for operator presence timeout
    checkOperatorPresenceTimeout();
    
    // Check dead man switch status
    checkDeadManSwitch();
    
    // Update geofencing
    if (teleop_config_.enable_geo_fencing) {
        updateGeofencing();
    }
    
    // Update status
    updatePolicyStatus();
}

PolicyState TeleopOnlyPolicy::getState() const 
{
    PolicyState state;
    state.operating_mode = static_cast<uint8_t>(operating_mode_);
    state.safety_level = safety_state_.operator_present && 
                        (!teleop_config_.dead_man_switch_required || safety_state_.dead_man_switch_active) ? 
                        100 : 0;
    state.autonomous_enabled = false;  // Never allow autonomous in teleop-only
    state.emergency_stop_active = safety_state_.emergency_stop_active;
    
    // Safety flags
    state.safety_flags.clear();
    if (!safety_state_.operator_present) state.safety_flags.push_back("NO_OPERATOR");
    if (teleop_config_.dead_man_switch_required && !safety_state_.dead_man_switch_active) {
        state.safety_flags.push_back("DEAD_MAN_SWITCH_INACTIVE");
    }
    if (safety_state_.speed_limit_exceeded) state.safety_flags.push_back("SPEED_LIMIT_EXCEEDED");
    if (safety_state_.geo_fence_violation) state.safety_flags.push_back("GEOFENCE_VIOLATION");
    
    return state;
}

bool TeleopOnlyPolicy::handleEmergencyStop() 
{
    RCLCPP_ERROR(rclcpp::get_logger("teleop_only_policy"), "Emergency stop activated in teleop-only mode");
    
    safety_state_.emergency_stop_active = true;
    
    // In teleop-only mode, emergency stop requires operator intervention to clear
    return true;
}

bool TeleopOnlyPolicy::checkOperatorSafety() 
{
    // Check operator presence
    if (!safety_state_.operator_present) {
        return false;
    }
    
    // Check for recent operator input
    auto now = std::chrono::steady_clock::now();
    auto time_since_input = std::chrono::duration_cast<std::chrono::seconds>(
        now - safety_state_.last_operator_input).count();
    
    if (time_since_input > teleop_config_.operator_presence_timeout) {
        RCLCPP_WARN(rclcpp::get_logger("teleop_only_policy"), 
                    "No operator input for %ld seconds (timeout: %.1f)",
                    time_since_input, teleop_config_.operator_presence_timeout);
        return false;
    }
    
    return true;
}

void TeleopOnlyPolicy::updateInputMonitoring(const VehicleCommand& command) 
{
    auto now = std::chrono::steady_clock::now();
    
    // Update input timestamp if command has meaningful input
    if (std::abs(command.linear_velocity) > 0.01 || 
        std::abs(command.angular_velocity) > 0.01 ||
        std::abs(command.steering_angle) > 0.01) {
        
        safety_state_.last_operator_input = now;
        input_state_.last_command_time = now;
        
        // Update input values
        input_state_.steering_input = command.steering_angle;
        input_state_.throttle_input = command.linear_velocity;
        // brake_input would come from separate brake command
        
        // Calculate command rate
        static auto last_update = now;
        auto dt = std::chrono::duration<double>(now - last_update).count();
        if (dt > 0.0) {
            input_state_.command_rate = 1.0 / dt;  // Hz
        }
        last_update = now;
    }
}

void TeleopOnlyPolicy::updateSafetyMonitoring() 
{
    // Simulate operator presence detection
    // In real implementation, this would read from sensors
    safety_state_.operator_present = simulateOperatorPresence();
    
    // Simulate dead man switch
    if (teleop_config_.dead_man_switch_required) {
        safety_state_.dead_man_switch_active = simulateDeadManSwitch();
    }
}

void TeleopOnlyPolicy::checkOperatorPresenceTimeout() 
{
    auto now = std::chrono::steady_clock::now();
    auto time_since_input = std::chrono::duration_cast<std::chrono::seconds>(
        now - safety_state_.last_operator_input).count();
    
    if (time_since_input > teleop_config_.operator_presence_timeout) {
        RCLCPP_WARN(rclcpp::get_logger("teleop_only_policy"), 
                    "Operator presence timeout: %ld seconds", time_since_input);
        safety_state_.operator_present = false;
    }
}

void TeleopOnlyPolicy::checkDeadManSwitch() 
{
    if (!teleop_config_.dead_man_switch_required) {
        return;
    }
    
    if (!safety_state_.dead_man_switch_active) {
        RCLCPP_DEBUG(rclcpp::get_logger("teleop_only_policy"), "Dead man switch not active");
    }
}

void TeleopOnlyPolicy::updateGeofencing() 
{
    // Simulate geofence checking
    // In real implementation, this would check GPS coordinates against defined boundaries
    safety_state_.geo_fence_violation = simulateGeofenceCheck();
    
    if (safety_state_.geo_fence_violation) {
        RCLCPP_WARN(rclcpp::get_logger("teleop_only_policy"), "Geofence violation detected");
    }
}

bool TeleopOnlyPolicy::checkGeofenceViolation() const 
{
    return safety_state_.geo_fence_violation;
}

bool TeleopOnlyPolicy::hasAutonomousFlags(const VehicleCommand& command) const 
{
    // Check for autonomous control flags
    // In real implementation, this would check specific flags in the command
    return false;  // Simulated - no autonomous flags detected
}

bool TeleopOnlyPolicy::hasAutonomousImplementFlags(const ImplementCommand& command) const 
{
    // Check for autonomous implement control flags
    return false;  // Simulated - no autonomous flags detected
}

bool TeleopOnlyPolicy::validateCommandSafety(const VehicleCommand& command) const 
{
    // Validate steering angle limits
    const double MAX_STEERING_ANGLE = M_PI / 4;  // 45 degrees
    if (std::abs(command.steering_angle) > MAX_STEERING_ANGLE) {
        RCLCPP_WARN(rclcpp::get_logger("teleop_only_policy"), 
                    "Steering angle too large: %.2f° (max: %.2f°)",
                    command.steering_angle * 180.0 / M_PI,
                    MAX_STEERING_ANGLE * 180.0 / M_PI);
        return false;
    }
    
    // Validate velocity limits
    if (command.linear_velocity < -5.0 || command.linear_velocity > teleop_config_.max_speed_limit) {
        return false;
    }
    
    return true;
}

bool TeleopOnlyPolicy::validateImplementSafety(const ImplementCommand& command) const 
{
    // Basic implement safety checks
    if (command.emergency_stop) {
        return true;  // Emergency stop is always safe
    }
    
    // Check implement type is recognized
    if (command.implement_type.empty()) {
        return false;
    }
    
    return true;
}

VehicleCommand TeleopOnlyPolicy::createEmergencyStopCommand() const 
{
    VehicleCommand stop_command;
    stop_command.linear_velocity = 0.0;
    stop_command.angular_velocity = 0.0;
    stop_command.steering_angle = 0.0;
    stop_command.emergency_stop = true;
    return stop_command;
}

VehicleCommand TeleopOnlyPolicy::createSafeStopCommand() const 
{
    VehicleCommand stop_command;
    stop_command.linear_velocity = 0.0;
    stop_command.angular_velocity = 0.0;
    stop_command.steering_angle = 0.0;
    stop_command.emergency_stop = false;  // Gradual stop, not emergency
    return stop_command;
}

ImplementCommand TeleopOnlyPolicy::createImplementEmergencyStop() const 
{
    ImplementCommand stop_command;
    stop_command.activate = false;
    stop_command.emergency_stop = true;
    stop_command.safety_mode = 2;  // Maintenance/safe mode
    return stop_command;
}

ImplementCommand TeleopOnlyPolicy::createImplementSafeStop() const 
{
    ImplementCommand stop_command;
    stop_command.activate = false;
    stop_command.emergency_stop = false;
    stop_command.safety_mode = 1;  // Safe mode
    return stop_command;
}

void TeleopOnlyPolicy::updatePolicyStatus() 
{
    // Update overall policy health
    bool healthy = safety_state_.operator_present && 
                  (!teleop_config_.dead_man_switch_required || safety_state_.dead_man_switch_active) &&
                  !safety_state_.geo_fence_violation &&
                  !safety_state_.emergency_stop_active;
    
    if (!healthy && policy_healthy_) {
        RCLCPP_WARN(rclcpp::get_logger("teleop_only_policy"), "Policy status changed to unhealthy");
    } else if (healthy && !policy_healthy_) {
        RCLCPP_INFO(rclcpp::get_logger("teleop_only_policy"), "Policy status changed to healthy");
    }
    
    policy_healthy_ = healthy;
}

// Simulation helper methods
bool TeleopOnlyPolicy::simulateOperatorPresence() const 
{
    // Simulate operator presence - in real implementation, this would read from sensors
    static bool presence = true;
    static int toggle_counter = 0;
    
    // Occasionally simulate operator leaving/returning for testing
    if (++toggle_counter > 10000) {  // Every ~10 seconds at 1kHz
        presence = !presence;
        toggle_counter = 0;
    }
    
    return presence;
}

bool TeleopOnlyPolicy::simulateDeadManSwitch() const 
{
    // Simulate dead man switch - in real implementation, this would read from hardware
    return safety_state_.operator_present;  // Active when operator is present
}

bool TeleopOnlyPolicy::simulateGeofenceCheck() const 
{
    // Simulate geofence check - in real implementation, this would use GPS
    return false;  // No violations in simulation
}

} // namespace system_controller 