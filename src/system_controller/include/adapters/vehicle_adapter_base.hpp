#ifndef VEHICLE_ADAPTER_BASE_HPP
#define VEHICLE_ADAPTER_BASE_HPP

#include <string>
#include <memory>
#include <map>
#include <functional>
#include "rclcpp/rclcpp.hpp"

namespace system_controller {

/**
 * @brief Configuration parameters for vehicle adapters
 */
struct VehicleAdapterConfig {
    std::string adapter_type;        // "polaris", "new_holland", etc.
    std::string adapter_id;          // Unique adapter instance ID
    std::string communication_port;  // "/dev/ttyUSB0", "can0", etc.
    int baudrate = 115200;          // Communication baudrate
    double max_steering_angle = 1.57; // Maximum steering angle (rad)
    double max_velocity = 10.0;      // Maximum velocity (m/s)
    double steering_scale = 1.0;     // Steering command scaling
    double velocity_scale = 1.0;     // Velocity command scaling
    double steering_offset = 0.0;    // Steering command offset
    double velocity_offset = 0.0;    // Velocity command offset
    int timeout_ms = 1000;          // Command timeout (milliseconds)
    bool enable_safety_checks = true; // Enable safety validation
    std::map<std::string, std::string> custom_params; // Custom parameters
};

/**
 * @brief Vehicle status information
 */
struct VehicleStatusInfo {
    bool is_connected = false;
    bool is_healthy = false;
    double current_steering_angle = 0.0;
    double current_velocity = 0.0;
    double current_acceleration = 0.0;
    std::string last_error;
    uint64_t last_command_timestamp = 0;
    std::map<std::string, double> diagnostics;
};

/**
 * @brief Base class for all vehicle adapters
 * 
 * This class provides the standard interface that all vehicle-specific
 * adapters must implement. It handles the translation between high-level
 * VehicleCommand messages and low-level vehicle protocols.
 */
class VehicleAdapterBase {
public:
    VehicleAdapterBase(const std::string& adapter_name);
    virtual ~VehicleAdapterBase() = default;

    // Core adapter interface
    virtual bool initialize(const VehicleAdapterConfig& config) = 0;
    virtual bool connect() = 0;
    virtual bool disconnect() = 0;
    virtual bool isConnected() const = 0;
    
    // Command processing
    virtual bool processVehicleCommand(const VehicleCommand& command) = 0;
    virtual VehicleStatusInfo getStatus() const = 0;
    
    // Configuration management
    virtual bool configure(const std::string& param_name, const std::string& param_value) = 0;
    virtual std::string getParameter(const std::string& param_name) const = 0;
    virtual std::map<std::string, std::string> getAllParameters() const = 0;
    
    // Safety and validation
    virtual bool validateCommand(const VehicleCommand& command) const = 0;
    virtual bool performSafetyCheck() = 0;
    virtual void emergencyStop() = 0;
    
    // Diagnostics
    virtual std::map<std::string, double> getDiagnostics() const = 0;
    virtual std::string getLastError() const { return last_error_; }
    
    // Getters
    std::string getAdapterName() const { return adapter_name_; }
    std::string getAdapterType() const { return config_.adapter_type; }
    std::string getAdapterId() const { return config_.adapter_id; }

protected:
    // Helper methods for derived classes
    bool validateSteeringAngle(double angle) const;
    bool validateVelocity(double velocity) const;
    double applySteeringScale(double raw_angle) const;
    double applyVelocityScale(double raw_velocity) const;
    void setError(const std::string& error);
    void clearError();
    
    // Member variables
    std::string adapter_name_;
    VehicleAdapterConfig config_;
    VehicleStatusInfo status_;
    std::string last_error_;
    
    // Callbacks for status updates
    std::function<void(const VehicleStatusInfo&)> status_callback_;
    std::function<void(const std::string&)> error_callback_;

public:
    // Callback setters
    void setStatusCallback(std::function<void(const VehicleStatusInfo&)> callback) {
        status_callback_ = callback;
    }
    
    void setErrorCallback(std::function<void(const std::string&)> callback) {
        error_callback_ = callback;
    }
};

// Forward declaration for VehicleCommand
struct VehicleCommand {
    double steering_angle;
    double velocity;
    double acceleration;
    double angular_velocity;
    uint8_t drive_mode;
    bool enable_safety_systems;
    bool emergency_stop;
    std::string command_id;
    uint64_t timestamp;
};

} // namespace system_controller

#endif // VEHICLE_ADAPTER_BASE_HPP 