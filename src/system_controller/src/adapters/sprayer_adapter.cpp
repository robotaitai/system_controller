#include "adapters/sprayer_adapter.hpp"
#include "system_controller/msg/mission_status.hpp"
#include <chrono>
#include <iomanip>
#include <sstream>

using namespace std::chrono_literals;

namespace system_controller
{

SprayerAdapter::SprayerAdapter(const rclcpp::NodeOptions& options)
    : ImplementAdapterBase("sprayer_adapter", options)
{
    RCLCPP_INFO(this->get_logger(), "SprayerAdapter constructor called");
    
    // Initialize nozzle array (8 sections by default)
    sprayer_state_.nozzle_enable.resize(8, false);
}

bool SprayerAdapter::initialize()
{
    RCLCPP_INFO(this->get_logger(), "Initializing SprayerAdapter");
    
    // Load configuration
    load_sprayer_config();
    
    // Create subscriptions
    sprayer_command_sub_ = this->create_subscription<system_controller::msg::SprayerCommand>(
        "sprayer_command", 10,
        std::bind(&SprayerAdapter::sprayer_command_callback, this, std::placeholders::_1));
    
    // Create publishers
    status_pub_ = this->create_publisher<system_controller::msg::MissionStatus>(
        "sprayer_status", 10);
    
    // Create timers
    status_timer_ = this->create_wall_timer(
        1000ms, std::bind(&SprayerAdapter::status_timer_callback, this));
    
    monitoring_timer_ = this->create_wall_timer(
        500ms, std::bind(&SprayerAdapter::monitoring_timer_callback, this));
    
    // Initialize hardware interface
    hardware_.can_bus_connected = validate_hardware_connection();
    
    // Perform self-test
    if (!perform_self_test()) {
        RCLCPP_ERROR(this->get_logger(), "SprayerAdapter self-test failed");
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "SprayerAdapter initialized successfully");
    return true;
}

bool SprayerAdapter::shutdown()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down SprayerAdapter");
    
    // Emergency stop and close valves
    emergency_stop_sprayer();
    
    // Close hardware connections
    hardware_.can_bus_connected = false;
    
    return true;
}

bool SprayerAdapter::process_command(const system_controller::msg::ImplementCommand& command)
{
    RCLCPP_INFO(this->get_logger(), "Processing generic implement command for sprayer");
    
    // Handle basic implement commands
    if (command.emergency_stop) {
        return emergency_stop_sprayer();
    }
    
    // Convert to sprayer-specific command (basic mapping)
    system_controller::msg::SprayerCommand sprayer_cmd;
    sprayer_cmd.header = command.header;
    sprayer_cmd.implement_id = command.implement_id;
    sprayer_cmd.activate = command.activate;
    sprayer_cmd.raise = command.raise;
    sprayer_cmd.height_position = command.height_position;
    sprayer_cmd.emergency_stop = command.emergency_stop;
    sprayer_cmd.safety_mode = command.safety_mode;
    sprayer_cmd.active_zones = command.active_zones;
    sprayer_cmd.command_id = command.command_id;
    sprayer_cmd.timeout = command.timeout;
    sprayer_cmd.priority = command.priority;
    
    return process_sprayer_command(sprayer_cmd);
}

bool SprayerAdapter::process_sprayer_command(const system_controller::msg::SprayerCommand& command)
{
    RCLCPP_INFO(this->get_logger(), "Processing sprayer-specific command");
    
    if (!validate_sprayer_command(command)) {
        RCLCPP_ERROR(this->get_logger(), "Invalid sprayer command received");
        return false;
    }
    
    bool success = true;
    
    // Process basic controls
    if (command.activate != sprayer_state_.valve_open) {
        success &= control_valve(command.activate);
    }
    
    // Process spray parameters
    if (command.spray_rate > 0 && command.spray_rate != sprayer_state_.spray_rate) {
        success &= set_spray_rate(command.spray_rate);
    }
    
    if (command.spray_pressure > 0 && command.spray_pressure != sprayer_state_.spray_pressure) {
        success &= set_spray_pressure(command.spray_pressure);
    }
    
    if (command.application_rate > 0 && command.application_rate != sprayer_state_.application_rate) {
        success &= set_application_rate(command.application_rate);
    }
    
    // Process boom controls
    if (!command.nozzle_enable.empty() && command.nozzle_enable != sprayer_state_.nozzle_enable) {
        success &= control_boom_sections(command.nozzle_enable);
    }
    
    if (command.spray_pattern != sprayer_state_.spray_pattern) {
        success &= set_spray_pattern(command.spray_pattern);
    }
    
    // Process advanced features
    if (command.auto_section_control != sprayer_state_.section_control_enabled) {
        success &= enable_section_control(command.auto_section_control);
    }
    
    if (command.overlap_reduction != sprayer_state_.overlap_reduction) {
        success &= set_overlap_reduction(command.overlap_reduction);
    }
    
    if (command.drift_reduction != sprayer_state_.drift_reduction_enabled) {
        success &= enable_drift_reduction(command.drift_reduction);
    }
    
    if (command.wind_compensation != sprayer_state_.wind_compensation) {
        success &= set_wind_compensation(command.wind_compensation);
    }
    
    if (!success) {
        RCLCPP_ERROR(this->get_logger(), "Failed to process some sprayer commands");
    }
    
    return success;
}

bool SprayerAdapter::set_spray_rate(double rate)
{
    if (rate < 0 || rate > config_.max_spray_rate) {
        RCLCPP_ERROR(this->get_logger(), "Invalid spray rate: %.2f L/min", rate);
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Setting spray rate to %.2f L/min", rate);
    
    // Simulate CAN bus communication
    std::vector<uint8_t> data = {
        0x01,  // Command: Set spray rate
        static_cast<uint8_t>(rate & 0xFF),
        static_cast<uint8_t>((static_cast<uint16_t>(rate) >> 8) & 0xFF)
    };
    
    if (send_can_message(hardware_.can_id_base + 0x10, data)) {
        sprayer_state_.spray_rate = rate;
        return true;
    }
    
    return false;
}

bool SprayerAdapter::set_spray_pressure(double pressure)
{
    if (pressure < 0 || pressure > config_.max_pressure) {
        RCLCPP_ERROR(this->get_logger(), "Invalid spray pressure: %.2f bar", pressure);
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Setting spray pressure to %.2f bar", pressure);
    
    // Simulate CAN bus communication with pressure control
    std::vector<uint8_t> data = {
        0x02,  // Command: Set pressure
        static_cast<uint8_t>(pressure * 10 & 0xFF),  // Convert to 0.1 bar resolution
        static_cast<uint8_t>((static_cast<uint16_t>(pressure * 10) >> 8) & 0xFF)
    };
    
    if (send_can_message(hardware_.can_id_base + 0x11, data)) {
        sprayer_state_.spray_pressure = pressure;
        return true;
    }
    
    return false;
}

bool SprayerAdapter::control_valve(bool open)
{
    RCLCPP_INFO(this->get_logger(), "Setting valve to %s", open ? "OPEN" : "CLOSED");
    
    // Check safety conditions before opening valve
    if (open && !check_safety_limits()) {
        RCLCPP_ERROR(this->get_logger(), "Cannot open valve: safety limits exceeded");
        return false;
    }
    
    std::vector<uint8_t> data = {
        0x03,  // Command: Valve control
        static_cast<uint8_t>(open ? 0x01 : 0x00)
    };
    
    if (send_can_message(hardware_.can_id_base + 0x12, data)) {
        sprayer_state_.valve_open = open;
        return true;
    }
    
    return false;
}

bool SprayerAdapter::set_application_rate(double rate)
{
    if (rate < config_.min_application_rate || rate > config_.max_application_rate) {
        RCLCPP_ERROR(this->get_logger(), "Invalid application rate: %.2f L/ha", rate);
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Setting application rate to %.2f L/ha", rate);
    sprayer_state_.application_rate = rate;
    
    // Update flow rates based on ground speed and application rate
    return update_nozzle_flow_rates();
}

bool SprayerAdapter::control_boom_sections(const std::vector<bool>& sections)
{
    if (sections.size() != config_.num_sections) {
        RCLCPP_ERROR(this->get_logger(), "Invalid number of boom sections: %zu (expected %d)", 
                     sections.size(), config_.num_sections);
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Controlling boom sections");
    
    std::vector<uint8_t> data = {0x04};  // Command: Boom section control
    
    // Pack section states into bytes
    for (size_t i = 0; i < sections.size(); ++i) {
        if (i % 8 == 0) data.push_back(0);
        if (sections[i]) {
            data.back() |= (1 << (i % 8));
        }
    }
    
    if (send_can_message(hardware_.can_id_base + 0x13, data)) {
        sprayer_state_.nozzle_enable = sections;
        return true;
    }
    
    return false;
}

bool SprayerAdapter::emergency_stop_sprayer()
{
    RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP: Closing all valves and stopping sprayer");
    
    // Close main valve
    control_valve(false);
    
    // Close all boom sections
    std::vector<bool> all_closed(config_.num_sections, false);
    control_boom_sections(all_closed);
    
    // Set pressure to zero
    set_spray_pressure(0.0);
    
    // Emergency stop CAN message
    std::vector<uint8_t> data = {0xFF, 0xFF, 0xFF};  // Emergency stop command
    send_can_message(hardware_.can_id_base + 0x00, data);
    
    return true;
}

bool SprayerAdapter::check_environmental_conditions()
{
    // Simulate environmental monitoring
    double current_temp = 25.0;  // Simulated temperature
    double current_wind = 3.0;   // Simulated wind speed
    
    if (current_temp > sprayer_state_.temperature_limit) {
        RCLCPP_WARN(this->get_logger(), "Temperature too high for spraying: %.1f°C", current_temp);
        return false;
    }
    
    if (current_wind > sprayer_state_.wind_speed_limit) {
        RCLCPP_WARN(this->get_logger(), "Wind speed too high for spraying: %.1f m/s", current_wind);
        return false;
    }
    
    return true;
}

void SprayerAdapter::load_sprayer_config()
{
    // Load configuration from parameters or config files
    this->declare_parameter("max_pressure", 20.0);
    this->declare_parameter("max_spray_rate", 500.0);
    this->declare_parameter("boom_width", 12.0);
    this->declare_parameter("num_sections", 8);
    
    config_.max_pressure = this->get_parameter("max_pressure").as_double();
    config_.max_spray_rate = this->get_parameter("max_spray_rate").as_double();
    config_.boom_width = this->get_parameter("boom_width").as_double();
    config_.num_sections = this->get_parameter("num_sections").as_int();
    
    RCLCPP_INFO(this->get_logger(), "Loaded sprayer config: %.1fm boom, %d sections", 
                config_.boom_width, config_.num_sections);
}

void SprayerAdapter::sprayer_command_callback(const system_controller::msg::SprayerCommand::SharedPtr msg)
{
    process_sprayer_command(*msg);
}

void SprayerAdapter::status_timer_callback()
{
    auto status_msg = system_controller::msg::MissionStatus();
    status_msg.header.stamp = this->now();
    status_msg.status = sprayer_state_.valve_open ? "SPRAYING" : "READY";
    
    status_pub_->publish(status_msg);
}

void SprayerAdapter::monitoring_timer_callback()
{
    // Check tank level
    check_tank_level();
    
    // Check environmental conditions
    if (sprayer_state_.weather_monitoring) {
        check_environmental_conditions();
    }
    
    // Check safety limits
    check_safety_limits();
}

bool SprayerAdapter::send_can_message(uint32_t id, const std::vector<uint8_t>& data)
{
    if (!hardware_.can_bus_connected) {
        RCLCPP_DEBUG(this->get_logger(), "CAN bus not connected - simulating message send");
        return true;  // Simulate successful send in test mode
    }
    
    RCLCPP_DEBUG(this->get_logger(), "Sending CAN message ID:0x%X with %zu bytes", id, data.size());
    return true;  // Simulate successful hardware communication
}

bool SprayerAdapter::validate_sprayer_command(const system_controller::msg::SprayerCommand& command)
{
    // Validate spray rate
    if (command.spray_rate < 0 || command.spray_rate > config_.max_spray_rate) {
        return false;
    }
    
    // Validate pressure
    if (command.spray_pressure < 0 || command.spray_pressure > config_.max_pressure) {
        return false;
    }
    
    // Validate application rate
    if (command.application_rate > 0 && 
        (command.application_rate < config_.min_application_rate || 
         command.application_rate > config_.max_application_rate)) {
        return false;
    }
    
    return true;
}

bool SprayerAdapter::check_safety_limits()
{
    return sprayer_state_.tank_level > 5.0 && check_environmental_conditions();
}

bool SprayerAdapter::perform_self_test()
{
    RCLCPP_INFO(this->get_logger(), "Performing sprayer self-test");
    
    // Test valve operation
    if (!control_valve(true)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (!control_valve(false)) return false;
    
    // Test pressure system
    if (!set_spray_pressure(5.0)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (!set_spray_pressure(0.0)) return false;
    
    RCLCPP_INFO(this->get_logger(), "Sprayer self-test completed successfully");
    return true;
}

bool SprayerAdapter::validate_hardware_connection()
{
    // Simulate hardware validation
    RCLCPP_INFO(this->get_logger(), "Validating sprayer hardware connection");
    return true;  // Simulate successful connection
}

bool SprayerAdapter::check_tank_level()
{
    // Simulate tank level monitoring
    sprayer_state_.tank_level = std::max(0.0, sprayer_state_.tank_level - 0.1);  // Simulate consumption
    
    if (sprayer_state_.tank_level < 10.0) {
        RCLCPP_WARN(this->get_logger(), "Low tank level: %.1f%%", sprayer_state_.tank_level);
    }
    
    return sprayer_state_.tank_level > 0.0;
}

bool SprayerAdapter::update_nozzle_flow_rates()
{
    // Calculate flow rates based on application rate and boom configuration
    RCLCPP_DEBUG(this->get_logger(), "Updating nozzle flow rates for application rate %.2f L/ha", 
                 sprayer_state_.application_rate);
    return true;
}

// Additional method implementations for completeness
bool SprayerAdapter::set_spray_pattern(uint8_t pattern) { sprayer_state_.spray_pattern = pattern; return true; }
bool SprayerAdapter::enable_section_control(bool enable) { sprayer_state_.section_control_enabled = enable; return true; }
bool SprayerAdapter::set_overlap_reduction(double percentage) { sprayer_state_.overlap_reduction = percentage; return true; }
bool SprayerAdapter::enable_drift_reduction(bool enable) { sprayer_state_.drift_reduction_enabled = enable; return true; }
bool SprayerAdapter::set_wind_compensation(double factor) { sprayer_state_.wind_compensation = factor; return true; }

} // namespace system_controller 