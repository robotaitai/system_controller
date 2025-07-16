#ifndef IMPLEMENT_ADAPTER_BASE_HPP
#define IMPLEMENT_ADAPTER_BASE_HPP

#include <string>
#include <memory>
#include <map>
#include <functional>
#include "rclcpp/rclcpp.hpp"

namespace system_controller {

/**
 * @brief Configuration parameters for implement adapters
 */
struct ImplementAdapterConfig {
    std::string adapter_type;        // "sprayer", "mower", "seeder", etc.
    std::string adapter_id;          // Unique adapter instance ID
    std::string communication_port;  // Communication interface
    int baudrate = 115200;          // Communication baudrate
    double max_flow_rate = 100.0;   // Maximum flow rate (L/min)
    double max_pressure = 20.0;     // Maximum pressure (bar)
    double min_height = 0.0;        // Minimum implement height (m)
    double max_height = 3.0;        // Maximum implement height (m)
    int timeout_ms = 1000;          // Command timeout
    bool enable_safety_checks = true; // Enable safety validation
    std::map<std::string, std::string> custom_params; // Custom parameters
};

/**
 * @brief Implement status information
 */
struct ImplementStatusInfo {
    bool is_connected = false;
    bool is_active = false;
    bool is_healthy = false;
    double current_height = 0.0;
    double current_flow_rate = 0.0;
    double current_pressure = 0.0;
    bool valve_open = false;
    std::string last_error;
    uint64_t last_command_timestamp = 0;
    std::map<std::string, double> diagnostics;
};

// Forward declaration for ImplementCommand
struct ImplementCommand {
    std::string implement_type;
    std::string implement_id;
    bool activate;
    bool raise;
    double height_position;
    bool valve_open;
    double flow_rate;
    double pressure;
    std::string chemical_type;
    double cutting_height;
    double blade_speed;
    bool side_discharge;
    double seed_rate;
    double planting_depth;
    bool metering_enable;
    bool emergency_stop;
    uint8_t safety_mode;
    std::string command_id;
    uint64_t timestamp;
};

/**
 * @brief Base class for all implement adapters
 * 
 * This class provides the standard interface that all implement-specific
 * adapters must implement. It handles the translation between high-level
 * ImplementCommand messages and low-level implement protocols.
 */
class ImplementAdapterBase {
public:
    ImplementAdapterBase(const std::string& adapter_name);
    virtual ~ImplementAdapterBase() = default;

    // Core adapter interface
    virtual bool initialize(const ImplementAdapterConfig& config) = 0;
    virtual bool connect() = 0;
    virtual bool disconnect() = 0;
    virtual bool isConnected() const = 0;
    
    // Command processing
    virtual bool processImplementCommand(const ImplementCommand& command) = 0;
    virtual ImplementStatusInfo getStatus() const = 0;
    
    // Configuration management
    virtual bool configure(const std::string& param_name, const std::string& param_value) = 0;
    virtual std::string getParameter(const std::string& param_name) const = 0;
    virtual std::map<std::string, std::string> getAllParameters() const = 0;
    
    // Safety and validation
    virtual bool validateCommand(const ImplementCommand& command) const = 0;
    virtual bool performSafetyCheck() = 0;
    virtual void emergencyStop() = 0;
    
    // Implement control
    virtual bool activate() = 0;
    virtual bool deactivate() = 0;
    virtual bool raiseImplement() = 0;
    virtual bool lowerImplement() = 0;
    virtual bool setHeight(double height) = 0;
    
    // Diagnostics
    virtual std::map<std::string, double> getDiagnostics() const = 0;
    virtual std::string getLastError() const { return last_error_; }
    
    // Getters
    std::string getAdapterName() const { return adapter_name_; }
    std::string getAdapterType() const { return config_.adapter_type; }
    std::string getAdapterId() const { return config_.adapter_id; }

protected:
    // Helper methods for derived classes
    bool validateHeight(double height) const;
    bool validateFlowRate(double flow_rate) const;
    bool validatePressure(double pressure) const;
    void setError(const std::string& error);
    void clearError();
    
    // Member variables
    std::string adapter_name_;
    ImplementAdapterConfig config_;
    ImplementStatusInfo status_;
    std::string last_error_;
    
    // Callbacks for status updates
    std::function<void(const ImplementStatusInfo&)> status_callback_;
    std::function<void(const std::string&)> error_callback_;

public:
    // Callback setters
    void setStatusCallback(std::function<void(const ImplementStatusInfo&)> callback) {
        status_callback_ = callback;
    }
    
    void setErrorCallback(std::function<void(const std::string&)> callback) {
        error_callback_ = callback;
    }
};

} // namespace system_controller

#endif // IMPLEMENT_ADAPTER_BASE_HPP 