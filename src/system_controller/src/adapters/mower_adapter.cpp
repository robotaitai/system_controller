#include "adapters/mower_adapter.hpp"
#include "system_controller/msg/mission_status.hpp"
#include <chrono>
#include <thread>
#include <cmath>

using namespace std::chrono_literals;

namespace system_controller
{

MowerAdapter::MowerAdapter(const rclcpp::NodeOptions& options)
    : ImplementAdapterBase("mower_adapter", options)
{
    RCLCPP_INFO(this->get_logger(), "MowerAdapter constructor called");
}

bool MowerAdapter::initialize()
{
    RCLCPP_INFO(this->get_logger(), "Initializing MowerAdapter");
    
    // Load configuration
    load_mower_config();
    
    // Create subscriptions
    mower_command_sub_ = this->create_subscription<system_controller::msg::MowerCommand>(
        "mower_command", 10,
        std::bind(&MowerAdapter::mower_command_callback, this, std::placeholders::_1));
    
    // Create publishers
    status_pub_ = this->create_publisher<system_controller::msg::MissionStatus>(
        "mower_status", 10);
    
    // Create timers
    status_timer_ = this->create_wall_timer(
        1000ms, std::bind(&MowerAdapter::status_timer_callback, this));
    
    monitoring_timer_ = this->create_wall_timer(
        500ms, std::bind(&MowerAdapter::monitoring_timer_callback, this));
    
    safety_timer_ = this->create_wall_timer(
        100ms, std::bind(&MowerAdapter::safety_timer_callback, this));
    
    // Initialize hardware interface
    hardware_.can_bus_connected = validate_hardware_connection();
    
    // Perform self-test
    if (!perform_self_test()) {
        RCLCPP_ERROR(this->get_logger(), "MowerAdapter self-test failed");
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "MowerAdapter initialized successfully");
    return true;
}

bool MowerAdapter::shutdown()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down MowerAdapter");
    
    // Emergency stop and disengage blades
    emergency_stop_mower();
    
    // Close hardware connections
    hardware_.can_bus_connected = false;
    
    return true;
}

bool MowerAdapter::process_command(const system_controller::msg::ImplementCommand& command)
{
    RCLCPP_INFO(this->get_logger(), "Processing generic implement command for mower");
    
    // Handle basic implement commands
    if (command.emergency_stop) {
        return emergency_stop_mower();
    }
    
    // Convert to mower-specific command (basic mapping)
    system_controller::msg::MowerCommand mower_cmd;
    mower_cmd.header = command.header;
    mower_cmd.implement_id = command.implement_id;
    mower_cmd.activate = command.activate;
    mower_cmd.raise = command.raise;
    mower_cmd.height_position = command.height_position;
    mower_cmd.emergency_stop = command.emergency_stop;
    mower_cmd.safety_mode = command.safety_mode;
    mower_cmd.active_zones = command.active_zones;
    mower_cmd.command_id = command.command_id;
    mower_cmd.timeout = command.timeout;
    mower_cmd.priority = command.priority;
    
    // Set reasonable defaults for mower-specific fields
    if (command.height_position > 0) {
        mower_cmd.cutting_height = command.height_position;
    }
    
    return process_mower_command(mower_cmd);
}

bool MowerAdapter::process_mower_command(const system_controller::msg::MowerCommand& command)
{
    RCLCPP_INFO(this->get_logger(), "Processing mower-specific command");
    
    if (!validate_mower_command(command)) {
        RCLCPP_ERROR(this->get_logger(), "Invalid mower command received");
        return false;
    }
    
    bool success = true;
    
    // Process basic controls
    if (command.activate != mower_state_.blades_engaged) {
        // Engage/disengage blades
        mower_state_.blades_engaged = command.activate;
        if (command.activate) {
            RCLCPP_INFO(this->get_logger(), "Engaging mower blades");
        } else {
            RCLCPP_INFO(this->get_logger(), "Disengaging mower blades");
        }
    }
    
    // Process cutting parameters
    if (command.cutting_height > 0 && command.cutting_height != mower_state_.cutting_height) {
        success &= set_cutting_height(command.cutting_height);
    }
    
    if (command.blade_speed > 0 && command.blade_speed != mower_state_.blade_speed) {
        success &= set_blade_speed(command.blade_speed);
    }
    
    if (command.side_discharge != mower_state_.side_discharge) {
        success &= control_side_discharge(command.side_discharge);
    }
    
    if (command.cutting_pattern != mower_state_.cutting_pattern) {
        success &= set_cutting_pattern(command.cutting_pattern);
    }
    
    if (command.overlap_percentage != mower_state_.overlap_percentage) {
        success &= set_overlap_percentage(command.overlap_percentage);
    }
    
    // Process advanced features
    if (command.mulching_mode != mower_state_.mulching_mode) {
        success &= enable_mulching_mode(command.mulching_mode);
    }
    
    if (command.grass_collection != mower_state_.grass_collection) {
        success &= enable_grass_collection(command.grass_collection);
    }
    
    if (command.ground_speed > 0 && command.ground_speed != mower_state_.ground_speed) {
        success &= set_ground_speed(command.ground_speed);
    }
    
    if (command.auto_height_control != mower_state_.auto_height_control) {
        success &= enable_auto_height_control(command.auto_height_control);
    }
    
    if (command.blade_engagement != mower_state_.blade_engagement) {
        success &= set_blade_engagement(command.blade_engagement);
    }
    
    // Process terrain adaptation
    if (command.slope_compensation != mower_state_.slope_compensation) {
        success &= enable_slope_compensation(command.slope_compensation);
    }
    
    if (command.max_slope_angle != mower_state_.max_slope_angle) {
        success &= set_max_slope_angle(command.max_slope_angle);
    }
    
    if (!success) {
        RCLCPP_ERROR(this->get_logger(), "Failed to process some mower commands");
    }
    
    return success;
}

bool MowerAdapter::set_cutting_height(double height)
{
    if (height < config_.min_cutting_height || height > config_.max_cutting_height) {
        RCLCPP_ERROR(this->get_logger(), "Invalid cutting height: %.3f m", height);
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Setting cutting height to %.1f cm", height * 100);
    
    // Simulate hydraulic height adjustment
    std::vector<uint8_t> data = {
        0x10,  // Command: Set cutting height
        static_cast<uint8_t>((height * 1000) & 0xFF),  // Convert to mm
        static_cast<uint8_t>(((static_cast<uint16_t>(height * 1000)) >> 8) & 0xFF)
    };
    
    if (send_can_message(hardware_.can_id_base + 0x10, data)) {
        mower_state_.cutting_height = height;
        return true;
    }
    
    return false;
}

bool MowerAdapter::set_blade_speed(double speed)
{
    if (speed < 0 || speed > config_.max_blade_speed) {
        RCLCPP_ERROR(this->get_logger(), "Invalid blade speed: %.1f RPM", speed);
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Setting blade speed to %.1f RPM", speed);
    
    // Simulate PTO control
    std::vector<uint8_t> data = {
        0x11,  // Command: Set blade speed
        static_cast<uint8_t>(speed & 0xFF),
        static_cast<uint8_t>((static_cast<uint16_t>(speed) >> 8) & 0xFF)
    };
    
    if (send_can_message(hardware_.can_id_base + 0x11, data)) {
        mower_state_.blade_speed = speed;
        return true;
    }
    
    return false;
}

bool MowerAdapter::control_side_discharge(bool enable)
{
    RCLCPP_INFO(this->get_logger(), "Setting side discharge to %s", enable ? "ENABLED" : "DISABLED");
    
    std::vector<uint8_t> data = {
        0x12,  // Command: Side discharge control
        static_cast<uint8_t>(enable ? 0x01 : 0x00)
    };
    
    if (send_can_message(hardware_.can_id_base + 0x12, data)) {
        mower_state_.side_discharge = enable;
        return true;
    }
    
    return false;
}

bool MowerAdapter::set_cutting_pattern(uint8_t pattern)
{
    if (pattern >= config_.cutting_patterns.size()) {
        RCLCPP_ERROR(this->get_logger(), "Invalid cutting pattern: %d", pattern);
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Setting cutting pattern to %s", 
                config_.cutting_patterns[pattern].c_str());
    
    mower_state_.cutting_pattern = pattern;
    return true;
}

bool MowerAdapter::enable_mulching_mode(bool enable)
{
    RCLCPP_INFO(this->get_logger(), "Setting mulching mode to %s", enable ? "ENABLED" : "DISABLED");
    
    std::vector<uint8_t> data = {
        0x13,  // Command: Mulching mode
        static_cast<uint8_t>(enable ? 0x01 : 0x00)
    };
    
    if (send_can_message(hardware_.can_id_base + 0x13, data)) {
        mower_state_.mulching_mode = enable;
        // Mulching and grass collection are mutually exclusive
        if (enable) {
            mower_state_.grass_collection = false;
        }
        return true;
    }
    
    return false;
}

bool MowerAdapter::enable_grass_collection(bool enable)
{
    RCLCPP_INFO(this->get_logger(), "Setting grass collection to %s", enable ? "ENABLED" : "DISABLED");
    
    std::vector<uint8_t> data = {
        0x14,  // Command: Grass collection
        static_cast<uint8_t>(enable ? 0x01 : 0x00)
    };
    
    if (send_can_message(hardware_.can_id_base + 0x14, data)) {
        mower_state_.grass_collection = enable;
        // Mulching and grass collection are mutually exclusive
        if (enable) {
            mower_state_.mulching_mode = false;
        }
        return true;
    }
    
    return false;
}

bool MowerAdapter::emergency_stop_mower()
{
    RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP: Disengaging blades and stopping mower");
    
    // Disengage blades
    mower_state_.blades_engaged = false;
    set_blade_speed(0.0);
    
    // Raise deck to maximum height
    set_cutting_height(config_.max_cutting_height);
    
    // Emergency stop CAN message
    std::vector<uint8_t> data = {0xFF, 0xFF, 0xFF};  // Emergency stop command
    send_can_message(hardware_.can_id_base + 0x00, data);
    
    return true;
}

bool MowerAdapter::check_operator_presence()
{
    // Simulate operator presence sensor
    // In real implementation, this would read from a sensor
    bool presence_detected = true;  // Simulate presence
    
    if (!presence_detected && mower_state_.blades_engaged) {
        RCLCPP_WARN(this->get_logger(), "Operator presence lost - initiating safety stop");
        emergency_stop_mower();
        return false;
    }
    
    mower_state_.operator_presence = presence_detected;
    return presence_detected;
}

bool MowerAdapter::perform_blade_check()
{
    // Simulate blade condition monitoring
    mower_state_.blade_sharpness = std::max(0.0, mower_state_.blade_sharpness - 0.1);  // Simulate wear
    
    if (mower_state_.blade_sharpness < 50.0) {
        RCLCPP_WARN(this->get_logger(), "Blade sharpness low: %.1f%% - service required", 
                    mower_state_.blade_sharpness);
    }
    
    return mower_state_.blade_sharpness > 20.0;  // Minimum acceptable sharpness
}

void MowerAdapter::load_mower_config()
{
    // Load configuration from parameters
    this->declare_parameter("min_cutting_height", 0.02);
    this->declare_parameter("max_cutting_height", 0.15);
    this->declare_parameter("max_blade_speed", 3600.0);
    this->declare_parameter("deck_width", 1.8);
    this->declare_parameter("num_blades", 3);
    
    config_.min_cutting_height = this->get_parameter("min_cutting_height").as_double();
    config_.max_cutting_height = this->get_parameter("max_cutting_height").as_double();
    config_.max_blade_speed = this->get_parameter("max_blade_speed").as_double();
    config_.deck_width = this->get_parameter("deck_width").as_double();
    config_.num_blades = this->get_parameter("num_blades").as_int();
    
    RCLCPP_INFO(this->get_logger(), "Loaded mower config: %.1fm deck, %d blades", 
                config_.deck_width, config_.num_blades);
}

void MowerAdapter::mower_command_callback(const system_controller::msg::MowerCommand::SharedPtr msg)
{
    process_mower_command(*msg);
}

void MowerAdapter::status_timer_callback()
{
    auto status_msg = system_controller::msg::MissionStatus();
    status_msg.header.stamp = this->now();
    status_msg.status = mower_state_.blades_engaged ? "MOWING" : "READY";
    
    status_pub_->publish(status_msg);
}

void MowerAdapter::monitoring_timer_callback()
{
    // Check blade condition
    perform_blade_check();
    
    // Check safety limits
    check_safety_limits();
    
    // Update cutting parameters
    update_cutting_parameters();
}

void MowerAdapter::safety_timer_callback()
{
    // Check operator presence
    check_operator_presence();
    
    // Monitor for safety violations
    if (mower_state_.blades_engaged && !check_safety_limits()) {
        RCLCPP_WARN(this->get_logger(), "Safety limits exceeded - stopping mower");
        emergency_stop_mower();
    }
}

bool MowerAdapter::send_can_message(uint32_t id, const std::vector<uint8_t>& data)
{
    if (!hardware_.can_bus_connected) {
        RCLCPP_DEBUG(this->get_logger(), "CAN bus not connected - simulating message send");
        return true;  // Simulate successful send in test mode
    }
    
    RCLCPP_DEBUG(this->get_logger(), "Sending CAN message ID:0x%X with %zu bytes", id, data.size());
    return true;  // Simulate successful hardware communication
}

bool MowerAdapter::validate_mower_command(const system_controller::msg::MowerCommand& command)
{
    // Validate cutting height
    if (command.cutting_height > 0 && 
        (command.cutting_height < config_.min_cutting_height || 
         command.cutting_height > config_.max_cutting_height)) {
        return false;
    }
    
    // Validate blade speed
    if (command.blade_speed < 0 || command.blade_speed > config_.max_blade_speed) {
        return false;
    }
    
    // Validate ground speed
    if (command.ground_speed < 0 || command.ground_speed > config_.max_ground_speed) {
        return false;
    }
    
    return true;
}

bool MowerAdapter::check_safety_limits()
{
    return mower_state_.operator_presence && mower_state_.blade_sharpness > 20.0;
}

bool MowerAdapter::perform_self_test()
{
    RCLCPP_INFO(this->get_logger(), "Performing mower self-test");
    
    // Test height adjustment
    double original_height = mower_state_.cutting_height;
    if (!set_cutting_height(0.1)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (!set_cutting_height(original_height)) return false;
    
    // Test blade engagement (without actually spinning)
    if (!set_blade_speed(100.0)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (!set_blade_speed(0.0)) return false;
    
    RCLCPP_INFO(this->get_logger(), "Mower self-test completed successfully");
    return true;
}

bool MowerAdapter::validate_hardware_connection()
{
    // Simulate hardware validation
    RCLCPP_INFO(this->get_logger(), "Validating mower hardware connection");
    return true;  // Simulate successful connection
}

double MowerAdapter::calculate_cutting_efficiency(double speed, double height)
{
    // Simple efficiency calculation based on speed and height
    double efficiency = std::max(0.0, 100.0 - (speed * 5.0) - (height * 100.0));
    return std::min(100.0, efficiency);
}

bool MowerAdapter::update_cutting_parameters()
{
    // Update cutting efficiency
    double efficiency = calculate_cutting_efficiency(mower_state_.ground_speed, mower_state_.cutting_height);
    RCLCPP_DEBUG(this->get_logger(), "Current cutting efficiency: %.1f%%", efficiency);
    return true;
}

// Additional method implementations for completeness
bool MowerAdapter::set_overlap_percentage(double percentage) { mower_state_.overlap_percentage = percentage; return true; }
bool MowerAdapter::set_ground_speed(double speed) { mower_state_.ground_speed = speed; return true; }
bool MowerAdapter::enable_auto_height_control(bool enable) { mower_state_.auto_height_control = enable; return true; }
bool MowerAdapter::set_blade_engagement(double engagement) { mower_state_.blade_engagement = engagement; return true; }
bool MowerAdapter::enable_slope_compensation(bool enable) { mower_state_.slope_compensation = enable; return true; }
bool MowerAdapter::set_max_slope_angle(double angle) { mower_state_.max_slope_angle = angle; return true; }
bool MowerAdapter::enable_anti_scalp_wheels(bool enable) { mower_state_.anti_scalp_wheels = enable; return true; }
bool MowerAdapter::enable_deck_float(bool enable) { mower_state_.deck_float = enable; return true; }

} // namespace system_controller 