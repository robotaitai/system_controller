#include "../../include/adapters/polaris_adapter.hpp"
#include <chrono>
#include <cstring>
#include <sstream>

namespace system_controller {

PolarisAdapter::PolarisAdapter() 
    : VehicleAdapterBase("polaris_adapter")
    , can_socket_(-1)
    , is_connected_(false)
    , communication_active_(false)
    , safety_system_healthy_(true)
    , last_valid_steering_(0.0)
    , last_valid_velocity_(0.0)
    , consecutive_errors_(0) {
    
    // Initialize default Polaris configuration
    polaris_config_ = PolarisConfig();
}

PolarisAdapter::~PolarisAdapter() {
    disconnect();
}

bool PolarisAdapter::initialize(const VehicleAdapterConfig& config) {
    config_ = config;
    
    // Parse Polaris-specific parameters
    auto it = config.custom_params.find("can_interface");
    if (it != config.custom_params.end()) {
        polaris_config_.can_interface = it->second;
    }
    
    it = config.custom_params.find("gear_ratio");
    if (it != config.custom_params.end()) {
        polaris_config_.gear_ratio = std::stod(it->second);
    }
    
    it = config.custom_params.find("wheel_base");
    if (it != config.custom_params.end()) {
        polaris_config_.wheel_base = std::stod(it->second);
    }
    
    clearError();
    return true;
}

bool PolarisAdapter::connect() {
    if (is_connected_) {
        return true;
    }
    
    // Initialize CAN communication
    if (!initializeCAN()) {
        setError("Failed to initialize CAN communication");
        return false;
    }
    
    // Start communication thread
    startCommunicationThread();
    
    is_connected_ = true;
    status_.is_connected = true;
    clearError();
    
    return true;
}

bool PolarisAdapter::disconnect() {
    if (!is_connected_) {
        return true;
    }
    
    // Stop communication thread
    stopCommunicationThread();
    
    // Close CAN connection
    closeCAN();
    
    is_connected_ = false;
    status_.is_connected = false;
    
    return true;
}

bool PolarisAdapter::isConnected() const {
    return is_connected_;
}

bool PolarisAdapter::processVehicleCommand(const VehicleCommand& command) {
    if (!is_connected_) {
        setError("Adapter not connected");
        return false;
    }
    
    if (!validateCommand(command)) {
        setError("Invalid command parameters");
        return false;
    }
    
    if (command.emergency_stop) {
        emergencyStop();
        return true;
    }
    
    // Send steering command
    double scaled_steering = applySteeringScale(command.steering_angle);
    if (!sendSteeringCommand(scaled_steering)) {
        setError("Failed to send steering command");
        return false;
    }
    
    // Send velocity command
    double scaled_velocity = applyVelocityScale(command.velocity);
    if (!sendVelocityCommand(scaled_velocity)) {
        setError("Failed to send velocity command");
        return false;
    }
    
    // Update status
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        status_.last_command_timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }
    
    last_command_time_ = std::chrono::steady_clock::now();
    clearError();
    return true;
}

VehicleStatusInfo PolarisAdapter::getStatus() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return status_;
}

bool PolarisAdapter::configure(const std::string& param_name, const std::string& param_value) {
    if (param_name == "can_interface") {
        polaris_config_.can_interface = param_value;
        return true;
    } else if (param_name == "gear_ratio") {
        polaris_config_.gear_ratio = std::stod(param_value);
        return true;
    } else if (param_name == "wheel_base") {
        polaris_config_.wheel_base = std::stod(param_value);
        return true;
    } else if (param_name == "heartbeat_interval_ms") {
        polaris_config_.heartbeat_interval_ms = std::stoi(param_value);
        return true;
    }
    
    // Handle base class parameters
    if (param_name == "max_steering_angle") {
        config_.max_steering_angle = std::stod(param_value);
        return true;
    } else if (param_name == "max_velocity") {
        config_.max_velocity = std::stod(param_value);
        return true;
    }
    
    return false;
}

std::string PolarisAdapter::getParameter(const std::string& param_name) const {
    if (param_name == "can_interface") {
        return polaris_config_.can_interface;
    } else if (param_name == "gear_ratio") {
        return std::to_string(polaris_config_.gear_ratio);
    } else if (param_name == "wheel_base") {
        return std::to_string(polaris_config_.wheel_base);
    } else if (param_name == "heartbeat_interval_ms") {
        return std::to_string(polaris_config_.heartbeat_interval_ms);
    } else if (param_name == "max_steering_angle") {
        return std::to_string(config_.max_steering_angle);
    } else if (param_name == "max_velocity") {
        return std::to_string(config_.max_velocity);
    }
    
    return "";
}

std::map<std::string, std::string> PolarisAdapter::getAllParameters() const {
    std::map<std::string, std::string> params;
    params["can_interface"] = polaris_config_.can_interface;
    params["gear_ratio"] = std::to_string(polaris_config_.gear_ratio);
    params["wheel_base"] = std::to_string(polaris_config_.wheel_base);
    params["heartbeat_interval_ms"] = std::to_string(polaris_config_.heartbeat_interval_ms);
    params["max_steering_angle"] = std::to_string(config_.max_steering_angle);
    params["max_velocity"] = std::to_string(config_.max_velocity);
    return params;
}

bool PolarisAdapter::validateCommand(const VehicleCommand& command) const {
    if (!validateSteeringAngle(command.steering_angle)) {
        return false;
    }
    
    if (!validateVelocity(command.velocity)) {
        return false;
    }
    
    // Additional Polaris-specific validation
    if (polaris_config_.use_ackermann) {
        // Check Ackermann constraints
        double inner_angle = command.steering_angle;
        if (std::abs(inner_angle) > 0.1) { // Only check for significant steering angles
            double turning_radius = polaris_config_.wheel_base / std::tan(std::abs(inner_angle));
            if (turning_radius < 2.0) { // Minimum turning radius for Polaris
                return false;
            }
        }
    }
    
    return true;
}

bool PolarisAdapter::performSafetyCheck() {
    // Check communication timeout
    auto now = std::chrono::steady_clock::now();
    auto time_since_last_command = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_command_time_).count();
    
    if (time_since_last_command > COMMUNICATION_TIMEOUT_MS) {
        setError("Communication timeout - no recent commands");
        safety_system_healthy_ = false;
        return false;
    }
    
    // Check CAN bus health (simulated)
    if (consecutive_errors_ > MAX_CONSECUTIVE_ERRORS) {
        setError("Too many consecutive communication errors");
        safety_system_healthy_ = false;
        return false;
    }
    
    safety_system_healthy_ = true;
    status_.is_healthy = true;
    return true;
}

void PolarisAdapter::emergencyStop() {
    // Send emergency stop command
    uint8_t stop_data[8] = {0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    sendCANMessage(polaris_config_.velocity_can_id, stop_data, 8);
    sendCANMessage(polaris_config_.steering_can_id, stop_data, 8);
    
    // Update status
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        status_.current_velocity = 0.0;
        status_.current_acceleration = 0.0;
    }
}

std::map<std::string, double> PolarisAdapter::getDiagnostics() const {
    std::map<std::string, double> diagnostics;
    
    auto now = std::chrono::steady_clock::now();
    auto time_since_last_command = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_command_time_).count();
    
    diagnostics["time_since_last_command_ms"] = static_cast<double>(time_since_last_command);
    diagnostics["consecutive_errors"] = static_cast<double>(consecutive_errors_);
    diagnostics["safety_system_healthy"] = safety_system_healthy_ ? 1.0 : 0.0;
    diagnostics["current_steering_angle"] = status_.current_steering_angle;
    diagnostics["current_velocity"] = status_.current_velocity;
    
    return diagnostics;
}

// Private methods

bool PolarisAdapter::initializeCAN() {
    // Simulated CAN initialization for testing
    // In real implementation, this would initialize the actual CAN socket
    can_socket_ = 1; // Fake socket ID
    return true;
}

bool PolarisAdapter::closeCAN() {
    can_socket_ = -1;
    return true;
}

bool PolarisAdapter::sendCANMessage(uint32_t can_id, const uint8_t* data, size_t length) {
    // Simulated CAN message sending
    // In real implementation, this would send actual CAN frames
    
    if (can_socket_ < 0) {
        consecutive_errors_++;
        return false;
    }
    
    // Simulate occasional communication errors
    if (consecutive_errors_ > 0 && (rand() % 100) < 5) { // 5% error rate when already having errors
        consecutive_errors_++;
        return false;
    }
    
    consecutive_errors_ = 0;
    return true;
}

bool PolarisAdapter::receiveCANMessage(uint32_t& can_id, uint8_t* data, size_t& length) {
    // Simulated CAN message receiving
    // In real implementation, this would receive actual CAN frames
    
    if (can_socket_ < 0) {
        return false;
    }
    
    // Simulate status message reception
    can_id = polaris_config_.status_can_id;
    length = 8;
    
    // Generate simulated status data
    uint16_t steering_value = static_cast<uint16_t>((status_.current_steering_angle + 1.57) * 1000);
    uint16_t velocity_value = static_cast<uint16_t>(status_.current_velocity * 100);
    
    data[0] = (steering_value >> 8) & 0xFF;
    data[1] = steering_value & 0xFF;
    data[2] = (velocity_value >> 8) & 0xFF;
    data[3] = velocity_value & 0xFF;
    data[4] = safety_system_healthy_ ? 0x01 : 0x00;
    data[5] = 0x00; // Reserved
    data[6] = 0x00; // Reserved
    data[7] = 0x00; // Reserved
    
    return true;
}

bool PolarisAdapter::sendSteeringCommand(double steering_angle) {
    // Convert steering angle to CAN value
    uint16_t can_value = static_cast<uint16_t>((steering_angle + config_.max_steering_angle) * 1000);
    
    uint8_t data[8] = {0};
    data[0] = (can_value >> 8) & 0xFF;
    data[1] = can_value & 0xFF;
    data[2] = 0x01; // Command type: steering
    
    bool success = sendCANMessage(polaris_config_.steering_can_id, data, 8);
    
    if (success) {
        last_valid_steering_ = steering_angle;
        std::lock_guard<std::mutex> lock(status_mutex_);
        status_.current_steering_angle = steering_angle;
    }
    
    return success;
}

bool PolarisAdapter::sendVelocityCommand(double velocity) {
    // Convert velocity to CAN value
    uint16_t can_value = static_cast<uint16_t>((velocity + config_.max_velocity) * 100);
    
    uint8_t data[8] = {0};
    data[0] = (can_value >> 8) & 0xFF;
    data[1] = can_value & 0xFF;
    data[2] = 0x02; // Command type: velocity
    
    bool success = sendCANMessage(polaris_config_.velocity_can_id, data, 8);
    
    if (success) {
        last_valid_velocity_ = velocity;
        std::lock_guard<std::mutex> lock(status_mutex_);
        status_.current_velocity = velocity;
    }
    
    return success;
}

bool PolarisAdapter::sendHeartbeat() {
    uint8_t data[8] = {0xAA, 0xBB, 0xCC, 0xDD, 0x00, 0x00, 0x00, 0x00};
    return sendCANMessage(polaris_config_.heartbeat_can_id, data, 8);
}

void PolarisAdapter::processStatusMessage(const uint8_t* data, size_t length) {
    if (length < 8) return;
    
    uint16_t steering_value = (data[0] << 8) | data[1];
    uint16_t velocity_value = (data[2] << 8) | data[3];
    
    std::lock_guard<std::mutex> lock(status_mutex_);
    status_.current_steering_angle = (steering_value / 1000.0) - 1.57;
    status_.current_velocity = velocity_value / 100.0;
    status_.is_healthy = (data[4] & 0x01) != 0;
    
    last_status_time_ = std::chrono::steady_clock::now();
}

void PolarisAdapter::communicationLoop() {
    while (communication_active_) {
        // Send heartbeat
        auto now = std::chrono::steady_clock::now();
        auto time_since_heartbeat = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_heartbeat_time_).count();
        
        if (time_since_heartbeat > polaris_config_.heartbeat_interval_ms) {
            sendHeartbeat();
            last_heartbeat_time_ = now;
        }
        
        // Receive status messages
        uint32_t can_id;
        uint8_t data[8];
        size_t length;
        
        if (receiveCANMessage(can_id, data, length)) {
            if (can_id == polaris_config_.status_can_id) {
                processStatusMessage(data, length);
            }
        }
        
        // Perform safety check
        performSafetyCheck();
        
        // Sleep for a short time
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void PolarisAdapter::startCommunicationThread() {
    communication_active_ = true;
    last_heartbeat_time_ = std::chrono::steady_clock::now();
    last_command_time_ = std::chrono::steady_clock::now();
    communication_thread_ = std::thread(&PolarisAdapter::communicationLoop, this);
}

void PolarisAdapter::stopCommunicationThread() {
    communication_active_ = false;
    if (communication_thread_.joinable()) {
        communication_thread_.join();
    }
}

} // namespace system_controller 