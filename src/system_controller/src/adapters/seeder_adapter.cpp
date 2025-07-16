#include "adapters/seeder_adapter.hpp"
#include "system_controller/msg/mission_status.hpp"
#include <chrono>
#include <thread>
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

namespace system_controller
{

SeederAdapter::SeederAdapter(const rclcpp::NodeOptions& options)
    : ImplementAdapterBase("seeder_adapter", options)
{
    RCLCPP_INFO(this->get_logger(), "SeederAdapter constructor called");
}

bool SeederAdapter::initialize()
{
    RCLCPP_INFO(this->get_logger(), "Initializing SeederAdapter");
    
    // Load configuration
    load_seeder_config();
    
    // Create subscriptions
    seeder_command_sub_ = this->create_subscription<system_controller::msg::SeederCommand>(
        "seeder_command", 10,
        std::bind(&SeederAdapter::seeder_command_callback, this, std::placeholders::_1));
    
    // Create publishers
    status_pub_ = this->create_publisher<system_controller::msg::MissionStatus>(
        "seeder_status", 10);
    
    // Create timers
    status_timer_ = this->create_wall_timer(
        1000ms, std::bind(&SeederAdapter::status_timer_callback, this));
    
    monitoring_timer_ = this->create_wall_timer(
        500ms, std::bind(&SeederAdapter::monitoring_timer_callback, this));
    
    calibration_timer_ = this->create_wall_timer(
        5000ms, std::bind(&SeederAdapter::calibration_timer_callback, this));
    
    // Initialize hardware interface
    hardware_.can_bus_connected = validate_hardware_connection();
    
    // Perform self-test
    if (!perform_self_test()) {
        RCLCPP_ERROR(this->get_logger(), "SeederAdapter self-test failed");
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "SeederAdapter initialized successfully");
    return true;
}

bool SeederAdapter::shutdown()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down SeederAdapter");
    
    // Emergency stop and disable metering
    emergency_stop_seeder();
    
    // Close hardware connections
    hardware_.can_bus_connected = false;
    
    return true;
}

bool SeederAdapter::process_command(const system_controller::msg::ImplementCommand& command)
{
    RCLCPP_INFO(this->get_logger(), "Processing generic implement command for seeder");
    
    // Handle basic implement commands
    if (command.emergency_stop) {
        return emergency_stop_seeder();
    }
    
    // Convert to seeder-specific command (basic mapping)
    system_controller::msg::SeederCommand seeder_cmd;
    seeder_cmd.header = command.header;
    seeder_cmd.implement_id = command.implement_id;
    seeder_cmd.activate = command.activate;
    seeder_cmd.raise = command.raise;
    seeder_cmd.height_position = command.height_position;
    seeder_cmd.emergency_stop = command.emergency_stop;
    seeder_cmd.safety_mode = command.safety_mode;
    seeder_cmd.active_zones = command.active_zones;
    seeder_cmd.command_id = command.command_id;
    seeder_cmd.timeout = command.timeout;
    seeder_cmd.priority = command.priority;
    
    // Set reasonable defaults for seeder-specific fields
    seeder_cmd.metering_enable = command.activate;
    if (command.height_position > 0) {
        seeder_cmd.planting_depth = command.height_position;
    }
    
    return process_seeder_command(seeder_cmd);
}

bool SeederAdapter::process_seeder_command(const system_controller::msg::SeederCommand& command)
{
    RCLCPP_INFO(this->get_logger(), "Processing seeder-specific command");
    
    if (!validate_seeder_command(command)) {
        RCLCPP_ERROR(this->get_logger(), "Invalid seeder command received");
        return false;
    }
    
    bool success = true;
    
    // Process basic controls
    if (command.metering_enable != seeder_state_.metering_enabled) {
        success &= enable_metering(command.metering_enable);
    }
    
    // Process seeding parameters
    if (command.seed_rate > 0 && command.seed_rate != seeder_state_.seed_rate) {
        success &= set_seed_rate(command.seed_rate);
    }
    
    if (command.planting_depth > 0 && command.planting_depth != seeder_state_.planting_depth) {
        success &= set_planting_depth(command.planting_depth);
    }
    
    if (!command.seed_type.empty() && command.seed_type != seeder_state_.seed_type) {
        success &= set_seed_type(command.seed_type);
    }
    
    if (command.row_spacing > 0 && command.row_spacing != seeder_state_.row_spacing) {
        success &= set_row_spacing(command.row_spacing);
    }
    
    // Process advanced features
    if (command.metering_system != seeder_state_.metering_system) {
        success &= set_metering_system(command.metering_system);
    }
    
    if (command.down_pressure > 0 && command.down_pressure != seeder_state_.down_pressure) {
        success &= set_down_pressure(command.down_pressure);
    }
    
    if (command.variable_rate_seeding != seeder_state_.variable_rate_seeding) {
        success &= enable_variable_rate_seeding(command.variable_rate_seeding);
    }
    
    if (command.emergence_rate > 0 && command.emergence_rate != seeder_state_.emergence_rate) {
        success &= set_emergence_rate(command.emergence_rate);
    }
    
    if (command.population_monitor != seeder_state_.population_monitor) {
        success &= enable_population_monitor(command.population_monitor);
    }
    
    // Process soil conditions
    if (command.soil_moisture > 0 && command.soil_moisture != seeder_state_.soil_moisture) {
        success &= set_soil_moisture_requirement(command.soil_moisture);
    }
    
    if (command.soil_temperature > 0 && command.soil_temperature != seeder_state_.soil_temperature) {
        success &= set_soil_temperature_requirement(command.soil_temperature);
    }
    
    // Process fertilizer integration
    if (command.fertilizer_application != seeder_state_.fertilizer_application) {
        success &= enable_fertilizer_application(command.fertilizer_application);
    }
    
    if (command.fertilizer_rate > 0 && command.fertilizer_rate != seeder_state_.fertilizer_rate) {
        success &= set_fertilizer_rate(command.fertilizer_rate);
    }
    
    if (!command.fertilizer_type.empty() && command.fertilizer_type != seeder_state_.fertilizer_type) {
        success &= set_fertilizer_type(command.fertilizer_type);
    }
    
    // Process precision features
    if (command.gps_guidance != seeder_state_.gps_guidance) {
        success &= enable_gps_guidance(command.gps_guidance);
    }
    
    if (command.section_control > 0 && command.section_control != seeder_state_.section_control_width) {
        success &= set_section_control(command.section_control);
    }
    
    if (command.overlap_reduction != seeder_state_.overlap_reduction) {
        success &= enable_overlap_reduction(command.overlap_reduction);
    }
    
    if (command.boundary_tracking != seeder_state_.boundary_tracking) {
        success &= enable_boundary_tracking(command.boundary_tracking);
    }
    
    if (!success) {
        RCLCPP_ERROR(this->get_logger(), "Failed to process some seeder commands");
    }
    
    return success;
}

bool SeederAdapter::set_seed_rate(double rate)
{
    if (rate < config_.min_seed_rate || rate > config_.max_seed_rate) {
        RCLCPP_ERROR(this->get_logger(), "Invalid seed rate: %.2f kg/ha", rate);
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Setting seed rate to %.2f kg/ha", rate);
    
    // Calculate population target
    double population = calculate_population_target(rate, seeder_state_.row_spacing);
    
    // Simulate CAN bus communication for seed rate control
    std::vector<uint8_t> data = {
        0x20,  // Command: Set seed rate
        static_cast<uint8_t>(rate & 0xFF),
        static_cast<uint8_t>((static_cast<uint16_t>(rate) >> 8) & 0xFF)
    };
    
    if (send_can_message(hardware_.can_id_base + 0x10, data)) {
        seeder_state_.seed_rate = rate;
        RCLCPP_INFO(this->get_logger(), "Target population: %.0f plants/ha", population);
        return update_metering_rates();
    }
    
    return false;
}

bool SeederAdapter::set_planting_depth(double depth)
{
    if (depth < config_.min_planting_depth || depth > config_.max_planting_depth) {
        RCLCPP_ERROR(this->get_logger(), "Invalid planting depth: %.3f m", depth);
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Setting planting depth to %.1f cm", depth * 100);
    
    // Simulate hydraulic depth control
    std::vector<uint8_t> data = {
        0x21,  // Command: Set planting depth
        static_cast<uint8_t>((depth * 1000) & 0xFF),  // Convert to mm
        static_cast<uint8_t>(((static_cast<uint16_t>(depth * 1000)) >> 8) & 0xFF)
    };
    
    if (send_can_message(hardware_.can_id_base + 0x11, data)) {
        seeder_state_.planting_depth = depth;
        return true;
    }
    
    return false;
}

bool SeederAdapter::enable_metering(bool enable)
{
    RCLCPP_INFO(this->get_logger(), "Setting seed metering to %s", enable ? "ENABLED" : "DISABLED");
    
    // Check soil conditions before enabling
    if (enable && !check_soil_conditions()) {
        RCLCPP_ERROR(this->get_logger(), "Cannot enable metering: soil conditions not suitable");
        return false;
    }
    
    std::vector<uint8_t> data = {
        0x22,  // Command: Enable/disable metering
        static_cast<uint8_t>(enable ? 0x01 : 0x00)
    };
    
    if (send_can_message(hardware_.can_id_base + 0x12, data)) {
        seeder_state_.metering_enabled = enable;
        return true;
    }
    
    return false;
}

bool SeederAdapter::set_seed_type(const std::string& type)
{
    auto it = std::find(config_.seed_types.begin(), config_.seed_types.end(), type);
    if (it == config_.seed_types.end()) {
        RCLCPP_ERROR(this->get_logger(), "Invalid seed type: %s", type.c_str());
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Setting seed type to %s", type.c_str());
    
    seeder_state_.seed_type = type;
    
    // Update metering parameters based on seed type
    return update_metering_rates();
}

bool SeederAdapter::set_row_spacing(double spacing)
{
    if (spacing < config_.min_row_spacing || spacing > config_.max_row_spacing) {
        RCLCPP_ERROR(this->get_logger(), "Invalid row spacing: %.3f m", spacing);
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Setting row spacing to %.1f cm", spacing * 100);
    
    seeder_state_.row_spacing = spacing;
    
    // Recalculate population and metering rates
    return update_metering_rates();
}

bool SeederAdapter::set_down_pressure(double pressure)
{
    RCLCPP_INFO(this->get_logger(), "Setting down pressure to %.1f N", pressure);
    
    // Simulate pneumatic down pressure control
    std::vector<uint8_t> data = {
        0x23,  // Command: Set down pressure
        static_cast<uint8_t>(pressure & 0xFF),
        static_cast<uint8_t>((static_cast<uint16_t>(pressure) >> 8) & 0xFF)
    };
    
    if (send_can_message(hardware_.can_id_base + 0x13, data)) {
        seeder_state_.down_pressure = pressure;
        return true;
    }
    
    return false;
}

bool SeederAdapter::enable_variable_rate_seeding(bool enable)
{
    RCLCPP_INFO(this->get_logger(), "Setting variable rate seeding to %s", enable ? "ENABLED" : "DISABLED");
    
    std::vector<uint8_t> data = {
        0x24,  // Command: Variable rate seeding
        static_cast<uint8_t>(enable ? 0x01 : 0x00)
    };
    
    if (send_can_message(hardware_.can_id_base + 0x14, data)) {
        seeder_state_.variable_rate_seeding = enable;
        return true;
    }
    
    return false;
}

bool SeederAdapter::check_soil_conditions()
{
    // Simulate soil monitoring
    double current_moisture = 22.0;  // Simulated soil moisture
    double current_temp = 12.0;      // Simulated soil temperature
    
    if (current_moisture < seeder_state_.soil_moisture) {
        RCLCPP_WARN(this->get_logger(), "Soil moisture too low: %.1f%% (required: %.1f%%)", 
                    current_moisture, seeder_state_.soil_moisture);
        return false;
    }
    
    if (current_temp < seeder_state_.soil_temperature) {
        RCLCPP_WARN(this->get_logger(), "Soil temperature too low: %.1f°C (required: %.1f°C)", 
                    current_temp, seeder_state_.soil_temperature);
        return false;
    }
    
    return true;
}

bool SeederAdapter::enable_fertilizer_application(bool enable)
{
    RCLCPP_INFO(this->get_logger(), "Setting fertilizer application to %s", enable ? "ENABLED" : "DISABLED");
    
    std::vector<uint8_t> data = {
        0x25,  // Command: Fertilizer application
        static_cast<uint8_t>(enable ? 0x01 : 0x00)
    };
    
    if (send_can_message(hardware_.can_id_base + 0x15, data)) {
        seeder_state_.fertilizer_application = enable;
        return true;
    }
    
    return false;
}

bool SeederAdapter::set_fertilizer_rate(double rate)
{
    RCLCPP_INFO(this->get_logger(), "Setting fertilizer rate to %.2f kg/ha", rate);
    
    std::vector<uint8_t> data = {
        0x26,  // Command: Set fertilizer rate
        static_cast<uint8_t>(rate & 0xFF),
        static_cast<uint8_t>((static_cast<uint16_t>(rate) >> 8) & 0xFF)
    };
    
    if (send_can_message(hardware_.can_id_base + 0x16, data)) {
        seeder_state_.fertilizer_rate = rate;
        return true;
    }
    
    return false;
}

bool SeederAdapter::emergency_stop_seeder()
{
    RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP: Disabling metering and raising seeder");
    
    // Disable metering
    enable_metering(false);
    
    // Raise seeder to maximum height
    set_planting_depth(config_.max_planting_depth);
    
    // Stop fertilizer application
    enable_fertilizer_application(false);
    
    // Emergency stop CAN message
    std::vector<uint8_t> data = {0xFF, 0xFF, 0xFF};  // Emergency stop command
    send_can_message(hardware_.can_id_base + 0x00, data);
    
    return true;
}

bool SeederAdapter::check_seed_level()
{
    // Simulate seed level monitoring
    seeder_state_.seed_level = std::max(0.0, seeder_state_.seed_level - 0.05);  // Simulate consumption
    
    if (seeder_state_.seed_level < 15.0) {
        RCLCPP_WARN(this->get_logger(), "Low seed level: %.1f%%", seeder_state_.seed_level);
    }
    
    if (seeder_state_.seed_level < 5.0 && seeder_state_.metering_enabled) {
        RCLCPP_ERROR(this->get_logger(), "Seed level critical - stopping seeder");
        emergency_stop_seeder();
        return false;
    }
    
    return seeder_state_.seed_level > 0.0;
}

void SeederAdapter::load_seeder_config()
{
    // Load configuration from parameters
    this->declare_parameter("min_seed_rate", 10.0);
    this->declare_parameter("max_seed_rate", 150.0);
    this->declare_parameter("min_planting_depth", 0.01);
    this->declare_parameter("max_planting_depth", 0.08);
    this->declare_parameter("num_rows", 12);
    
    config_.min_seed_rate = this->get_parameter("min_seed_rate").as_double();
    config_.max_seed_rate = this->get_parameter("max_seed_rate").as_double();
    config_.min_planting_depth = this->get_parameter("min_planting_depth").as_double();
    config_.max_planting_depth = this->get_parameter("max_planting_depth").as_double();
    config_.num_rows = this->get_parameter("num_rows").as_int();
    
    RCLCPP_INFO(this->get_logger(), "Loaded seeder config: %d rows, %.2f-%.2f kg/ha rate range", 
                config_.num_rows, config_.min_seed_rate, config_.max_seed_rate);
}

void SeederAdapter::seeder_command_callback(const system_controller::msg::SeederCommand::SharedPtr msg)
{
    process_seeder_command(*msg);
}

void SeederAdapter::status_timer_callback()
{
    auto status_msg = system_controller::msg::MissionStatus();
    status_msg.header.stamp = this->now();
    status_msg.status = seeder_state_.metering_enabled ? "SEEDING" : "READY";
    
    status_pub_->publish(status_msg);
}

void SeederAdapter::monitoring_timer_callback()
{
    // Check seed level
    check_seed_level();
    
    // Check soil conditions
    check_soil_conditions();
    
    // Monitor seed flow
    check_seed_flow();
    
    // Check safety limits
    check_safety_limits();
}

void SeederAdapter::calibration_timer_callback()
{
    // Perform periodic calibration checks
    if (seeder_state_.metering_enabled) {
        monitor_emergence();
    }
}

bool SeederAdapter::send_can_message(uint32_t id, const std::vector<uint8_t>& data)
{
    if (!hardware_.can_bus_connected) {
        RCLCPP_DEBUG(this->get_logger(), "CAN bus not connected - simulating message send");
        return true;  // Simulate successful send in test mode
    }
    
    RCLCPP_DEBUG(this->get_logger(), "Sending CAN message ID:0x%X with %zu bytes", id, data.size());
    return true;  // Simulate successful hardware communication
}

bool SeederAdapter::validate_seeder_command(const system_controller::msg::SeederCommand& command)
{
    // Validate seed rate
    if (command.seed_rate > 0 && 
        (command.seed_rate < config_.min_seed_rate || command.seed_rate > config_.max_seed_rate)) {
        return false;
    }
    
    // Validate planting depth
    if (command.planting_depth > 0 && 
        (command.planting_depth < config_.min_planting_depth || 
         command.planting_depth > config_.max_planting_depth)) {
        return false;
    }
    
    // Validate row spacing
    if (command.row_spacing > 0 && 
        (command.row_spacing < config_.min_row_spacing || 
         command.row_spacing > config_.max_row_spacing)) {
        return false;
    }
    
    return true;
}

bool SeederAdapter::check_safety_limits()
{
    return seeder_state_.seed_level > 5.0 && check_soil_conditions();
}

bool SeederAdapter::perform_self_test()
{
    RCLCPP_INFO(this->get_logger(), "Performing seeder self-test");
    
    // Test depth adjustment
    double original_depth = seeder_state_.planting_depth;
    if (!set_planting_depth(0.03)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (!set_planting_depth(original_depth)) return false;
    
    // Test metering system
    if (!enable_metering(true)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (!enable_metering(false)) return false;
    
    // Test down pressure
    if (!set_down_pressure(100.0)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (!set_down_pressure(seeder_state_.down_pressure)) return false;
    
    RCLCPP_INFO(this->get_logger(), "Seeder self-test completed successfully");
    return true;
}

bool SeederAdapter::validate_hardware_connection()
{
    // Simulate hardware validation
    RCLCPP_INFO(this->get_logger(), "Validating seeder hardware connection");
    return true;  // Simulate successful connection
}

double SeederAdapter::calculate_population_target(double rate, double spacing)
{
    // Simplified population calculation
    // This would be more sophisticated in real implementation
    double plants_per_meter = rate / (spacing * 10000);  // Convert to plants/ha
    return plants_per_meter * 10000;  // Plants per hectare
}

bool SeederAdapter::update_metering_rates()
{
    // Update metering system based on current settings
    RCLCPP_DEBUG(this->get_logger(), "Updating metering rates for %s at %.2f kg/ha", 
                 seeder_state_.seed_type.c_str(), seeder_state_.seed_rate);
    return true;
}

bool SeederAdapter::check_seed_flow()
{
    // Simulate seed flow monitoring
    if (seeder_state_.metering_enabled) {
        RCLCPP_DEBUG(this->get_logger(), "Seed flow normal");
    }
    return true;
}

bool SeederAdapter::monitor_emergence()
{
    // Simulate emergence monitoring
    RCLCPP_DEBUG(this->get_logger(), "Monitoring emergence: %.1f%% target", seeder_state_.emergence_rate);
    return true;
}

// Additional method implementations for completeness
bool SeederAdapter::set_metering_system(uint8_t system) { seeder_state_.metering_system = system; return true; }
bool SeederAdapter::set_emergence_rate(double rate) { seeder_state_.emergence_rate = rate; return true; }
bool SeederAdapter::enable_population_monitor(bool enable) { seeder_state_.population_monitor = enable; return true; }
bool SeederAdapter::set_soil_moisture_requirement(double moisture) { seeder_state_.soil_moisture = moisture; return true; }
bool SeederAdapter::set_soil_temperature_requirement(double temperature) { seeder_state_.soil_temperature = temperature; return true; }
bool SeederAdapter::check_tillage_requirements(bool required) { seeder_state_.tillage_required = required; return true; }
bool SeederAdapter::set_fertilizer_type(const std::string& type) { seeder_state_.fertilizer_type = type; return true; }
bool SeederAdapter::set_fertilizer_depth(double depth) { seeder_state_.fertilizer_depth = depth; return true; }
bool SeederAdapter::enable_gps_guidance(bool enable) { seeder_state_.gps_guidance = enable; return true; }
bool SeederAdapter::set_section_control(double width) { seeder_state_.section_control_width = width; return true; }
bool SeederAdapter::enable_overlap_reduction(bool enable) { seeder_state_.overlap_reduction = enable; return true; }
bool SeederAdapter::enable_boundary_tracking(bool enable) { seeder_state_.boundary_tracking = enable; return true; }

} // namespace system_controller 