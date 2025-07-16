#ifndef POLARIS_ADAPTER_HPP
#define POLARIS_ADAPTER_HPP

#include "vehicle_adapter_base.hpp"
#include <thread>
#include <atomic>
#include <mutex>

namespace system_controller {

/**
 * @brief Polaris-specific configuration parameters
 */
struct PolarisConfig {
    std::string can_interface = "can0";  // CAN interface name
    uint32_t steering_can_id = 0x100;    // CAN ID for steering commands
    uint32_t velocity_can_id = 0x101;    // CAN ID for velocity commands
    uint32_t status_can_id = 0x200;      // CAN ID for status messages
    uint32_t heartbeat_can_id = 0x300;   // CAN ID for heartbeat
    int heartbeat_interval_ms = 100;     // Heartbeat interval
    double gear_ratio = 15.7;            // Steering gear ratio
    double wheel_base = 2.5;             // Wheelbase in meters
    bool use_ackermann = true;           // Use Ackermann steering geometry
};

/**
 * @brief Vehicle adapter for Polaris vehicles
 * 
 * This adapter handles communication with Polaris vehicles through CAN bus.
 * It translates high-level VehicleCommand messages into Polaris-specific
 * CAN messages and handles status feedback.
 */
class PolarisAdapter : public VehicleAdapterBase {
public:
    PolarisAdapter();
    ~PolarisAdapter() override;

    // VehicleAdapterBase interface implementation
    bool initialize(const VehicleAdapterConfig& config) override;
    bool connect() override;
    bool disconnect() override;
    bool isConnected() const override;
    
    bool processVehicleCommand(const VehicleCommand& command) override;
    VehicleStatusInfo getStatus() const override;
    
    bool configure(const std::string& param_name, const std::string& param_value) override;
    std::string getParameter(const std::string& param_name) const override;
    std::map<std::string, std::string> getAllParameters() const override;
    
    bool validateCommand(const VehicleCommand& command) const override;
    bool performSafetyCheck() override;
    void emergencyStop() override;
    
    std::map<std::string, double> getDiagnostics() const override;

private:
    // CAN communication methods
    bool initializeCAN();
    bool closeCAN();
    bool sendCANMessage(uint32_t can_id, const uint8_t* data, size_t length);
    bool receiveCANMessage(uint32_t& can_id, uint8_t* data, size_t& length);
    
    // Message processing
    bool sendSteeringCommand(double steering_angle);
    bool sendVelocityCommand(double velocity);
    bool sendHeartbeat();
    void processStatusMessage(const uint8_t* data, size_t length);
    void processHeartbeatResponse(const uint8_t* data, size_t length);
    
    // Conversion methods
    double steeringAngleToCanValue(double angle);
    double canValueToSteeringAngle(uint16_t can_value);
    double velocityToCanValue(double velocity);
    double canValueToVelocity(uint16_t can_value);
    
    // Threading for continuous communication
    void communicationLoop();
    void startCommunicationThread();
    void stopCommunicationThread();
    
    // Member variables
    PolarisConfig polaris_config_;
    int can_socket_;
    std::atomic<bool> is_connected_;
    std::atomic<bool> communication_active_;
    std::thread communication_thread_;
    mutable std::mutex status_mutex_;
    
    // Timing
    std::chrono::steady_clock::time_point last_heartbeat_time_;
    std::chrono::steady_clock::time_point last_command_time_;
    std::chrono::steady_clock::time_point last_status_time_;
    
    // Safety monitoring
    bool safety_system_healthy_;
    double last_valid_steering_;
    double last_valid_velocity_;
    uint32_t consecutive_errors_;
    static constexpr uint32_t MAX_CONSECUTIVE_ERRORS = 10;
    static constexpr int COMMUNICATION_TIMEOUT_MS = 5000;
};

} // namespace system_controller

#endif // POLARIS_ADAPTER_HPP 