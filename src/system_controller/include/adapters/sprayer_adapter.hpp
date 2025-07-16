#ifndef SPRAYER_ADAPTER_HPP
#define SPRAYER_ADAPTER_HPP

#include "adapters/implement_adapter_base.hpp"
#include "system_controller/msg/sprayer_command.hpp"
#include "system_controller/msg/implement_command.hpp"
#include <rclcpp/rclcpp.hpp>
#include <map>
#include <string>

namespace system_controller
{

class SprayerAdapter : public ImplementAdapterBase
{
public:
    explicit SprayerAdapter(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    virtual ~SprayerAdapter() = default;

    // Core adapter interface
    bool initialize() override;
    bool shutdown() override;
    bool process_command(const system_controller::msg::ImplementCommand& command) override;
    
    // Sprayer-specific command processing
    bool process_sprayer_command(const system_controller::msg::SprayerCommand& command);
    
    // Sprayer control methods
    bool set_spray_rate(double rate);
    bool set_spray_pressure(double pressure);
    bool control_valve(bool open);
    bool set_application_rate(double rate);
    bool control_boom_sections(const std::vector<bool>& sections);
    bool set_spray_pattern(uint8_t pattern);
    
    // Advanced sprayer features
    bool enable_section_control(bool enable);
    bool set_overlap_reduction(double percentage);
    bool enable_drift_reduction(bool enable);
    bool set_wind_compensation(double factor);
    
    // Safety and monitoring
    bool check_tank_level();
    bool check_environmental_conditions();
    bool emergency_stop_sprayer();
    
    // Configuration and status
    void load_sprayer_config();
    bool get_sprayer_status(std::map<std::string, double>& status);
    bool calibrate_sprayer();

private:
    // ROS subscriptions and publishers
    rclcpp::Subscription<system_controller::msg::SprayerCommand>::SharedPtr sprayer_command_sub_;
    rclcpp::Publisher<system_controller::msg::MissionStatus>::SharedPtr status_pub_;
    
    // Sprayer state variables
    struct SprayerState {
        bool valve_open = false;
        double spray_rate = 0.0;
        double spray_pressure = 0.0;
        double application_rate = 0.0;
        double boom_width = 12.0;  // default 12m boom
        std::vector<bool> nozzle_enable;
        uint8_t spray_pattern = 0;
        double tank_level = 100.0;
        bool section_control_enabled = false;
        double overlap_reduction = 0.0;
        bool drift_reduction_enabled = false;
        double wind_compensation = 1.0;
        bool weather_monitoring = true;
        double temperature_limit = 35.0;  // Celsius
        double wind_speed_limit = 8.0;    // m/s
    } sprayer_state_;
    
    // Hardware interface simulation
    struct HardwareInterface {
        bool can_bus_connected = false;
        std::string device_path = "/dev/can0";
        uint32_t can_id_base = 0x100;
        double valve_response_time = 0.5;  // seconds
        double pressure_response_time = 2.0;  // seconds
    } hardware_;
    
    // Configuration parameters
    struct SprayerConfig {
        double max_pressure = 20.0;        // bar
        double max_spray_rate = 500.0;     // L/min
        double min_application_rate = 50.0; // L/ha
        double max_application_rate = 400.0; // L/ha
        double boom_width = 12.0;          // meters
        uint8_t num_sections = 8;
        std::vector<std::string> chemical_types = {"herbicide", "pesticide", "fertilizer", "water"};
    } config_;
    
    // Timers and callbacks
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::TimerBase::SharedPtr monitoring_timer_;
    
    // Callback methods
    void sprayer_command_callback(const system_controller::msg::SprayerCommand::SharedPtr msg);
    void status_timer_callback();
    void monitoring_timer_callback();
    
    // Hardware communication methods
    bool send_can_message(uint32_t id, const std::vector<uint8_t>& data);
    bool read_can_message(uint32_t id, std::vector<uint8_t>& data);
    bool validate_hardware_connection();
    
    // Safety and validation methods
    bool validate_sprayer_command(const system_controller::msg::SprayerCommand& command);
    bool check_safety_limits();
    bool perform_self_test();
    
    // Utility methods
    double calculate_ground_speed_compensation(double target_rate, double ground_speed);
    bool update_nozzle_flow_rates();
    std::string get_current_timestamp();
};

} // namespace system_controller

#endif // SPRAYER_ADAPTER_HPP 