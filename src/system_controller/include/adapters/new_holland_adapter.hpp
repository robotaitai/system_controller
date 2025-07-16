#ifndef NEW_HOLLAND_ADAPTER_HPP
#define NEW_HOLLAND_ADAPTER_HPP

#include "vehicle_adapter_base.hpp"
#include <thread>
#include <atomic>
#include <mutex>

namespace system_controller {

/**
 * @brief New Holland-specific configuration parameters
 */
struct NewHollandConfig {
    std::string serial_port = "/dev/ttyUSB0";  // Serial port path
    int baudrate = 38400;                      // Serial baudrate
    uint8_t device_address = 0x01;             // Modbus device address
    int response_timeout_ms = 500;             // Response timeout
    double implement_width = 12.0;             // Implement width in meters
    double max_ground_speed = 25.0;            // Maximum ground speed km/h
    bool use_modbus_rtu = true;                // Use Modbus RTU protocol
};

/**
 * @brief Vehicle adapter for New Holland tractors
 * 
 * This adapter handles communication with New Holland tractors through serial
 * communication using Modbus RTU protocol. It translates high-level VehicleCommand
 * messages into New Holland-specific Modbus commands.
 */
class NewHollandAdapter : public VehicleAdapterBase {
public:
    NewHollandAdapter();
    ~NewHollandAdapter() override;

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
    // Serial communication methods
    bool initializeSerial();
    bool closeSerial();
    bool sendSerialData(const uint8_t* data, size_t length);
    bool receiveSerialData(uint8_t* data, size_t& length, int timeout_ms);
    
    // Modbus protocol methods
    bool sendModbusCommand(uint8_t function_code, uint16_t address, uint16_t value);
    bool readModbusRegisters(uint16_t start_address, uint16_t count, uint16_t* values);
    uint16_t calculateModbusCRC(const uint8_t* data, size_t length);
    
    // Command processing
    bool sendSteeringCommand(double steering_angle);
    bool sendVelocityCommand(double velocity);
    bool sendEngageCommand(bool engage);
    void processStatusResponse(const uint8_t* data, size_t length);
    
    // Conversion methods
    uint16_t steeringAngleToModbusValue(double angle);
    double modbusValueToSteeringAngle(uint16_t value);
    uint16_t velocityToModbusValue(double velocity);
    double modbusValueToVelocity(uint16_t value);
    
    // Communication thread
    void communicationLoop();
    void startCommunicationThread();
    void stopCommunicationThread();
    
    // Member variables
    NewHollandConfig new_holland_config_;
    int serial_fd_;
    std::atomic<bool> is_connected_;
    std::atomic<bool> communication_active_;
    std::thread communication_thread_;
    mutable std::mutex status_mutex_;
    mutable std::mutex communication_mutex_;
    
    // Timing
    std::chrono::steady_clock::time_point last_command_time_;
    std::chrono::steady_clock::time_point last_status_time_;
    
    // Status tracking
    bool hydraulic_system_enabled_;
    double engine_rpm_;
    double fuel_level_;
    uint32_t error_count_;
    
    // Modbus register addresses
    static constexpr uint16_t STEERING_REGISTER = 0x1000;
    static constexpr uint16_t VELOCITY_REGISTER = 0x1001;
    static constexpr uint16_t ENGAGE_REGISTER = 0x1002;
    static constexpr uint16_t STATUS_REGISTER = 0x2000;
    static constexpr uint16_t ENGINE_RPM_REGISTER = 0x2001;
    static constexpr uint16_t FUEL_LEVEL_REGISTER = 0x2002;
};

} // namespace system_controller

#endif // NEW_HOLLAND_ADAPTER_HPP 