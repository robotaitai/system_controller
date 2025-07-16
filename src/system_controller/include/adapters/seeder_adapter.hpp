#ifndef SEEDER_ADAPTER_HPP
#define SEEDER_ADAPTER_HPP

#include "adapters/implement_adapter_base.hpp"
#include "system_controller/msg/seeder_command.hpp"
#include "system_controller/msg/implement_command.hpp"
#include <rclcpp/rclcpp.hpp>
#include <map>
#include <string>

namespace system_controller
{

class SeederAdapter : public ImplementAdapterBase
{
public:
    explicit SeederAdapter(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    virtual ~SeederAdapter() = default;

    // Core adapter interface
    bool initialize() override;
    bool shutdown() override;
    bool process_command(const system_controller::msg::ImplementCommand& command) override;
    
    // Seeder-specific command processing
    bool process_seeder_command(const system_controller::msg::SeederCommand& command);
    
    // Seeder control methods
    bool set_seed_rate(double rate);
    bool set_planting_depth(double depth);
    bool enable_metering(bool enable);
    bool set_seed_type(const std::string& type);
    bool set_row_spacing(double spacing);
    
    // Advanced seeding features
    bool set_metering_system(uint8_t system);
    bool set_down_pressure(double pressure);
    bool enable_variable_rate_seeding(bool enable);
    bool set_emergence_rate(double rate);
    bool enable_population_monitor(bool enable);
    
    // Soil and environmental controls
    bool check_soil_conditions();
    bool set_soil_moisture_requirement(double moisture);
    bool set_soil_temperature_requirement(double temperature);
    bool check_tillage_requirements(bool required);
    
    // Fertilizer integration
    bool enable_fertilizer_application(bool enable);
    bool set_fertilizer_rate(double rate);
    bool set_fertilizer_type(const std::string& type);
    bool set_fertilizer_depth(double depth);
    
    // Precision features
    bool enable_gps_guidance(bool enable);
    bool set_section_control(double width);
    bool enable_overlap_reduction(bool enable);
    bool enable_boundary_tracking(bool enable);
    
    // Safety and monitoring
    bool check_seed_level();
    bool emergency_stop_seeder();
    bool perform_calibration();
    
    // Configuration and status
    void load_seeder_config();
    bool get_seeder_status(std::map<std::string, double>& status);

private:
    // ROS subscriptions and publishers
    rclcpp::Subscription<system_controller::msg::SeederCommand>::SharedPtr seeder_command_sub_;
    rclcpp::Publisher<system_controller::msg::MissionStatus>::SharedPtr status_pub_;
    
    // Seeder state variables
    struct SeederState {
        bool metering_enabled = false;
        double seed_rate = 0.0;           // seeds/meter or kg/ha
        double planting_depth = 0.025;    // 2.5cm default
        std::string seed_type = "corn";
        double row_spacing = 0.76;        // 30" rows
        uint8_t metering_system = 0;      // 0=mechanical, 1=pneumatic, 2=electric
        double down_pressure = 150.0;     // N
        bool variable_rate_seeding = false;
        double emergence_rate = 85.0;     // percentage
        bool population_monitor = false;
        double soil_moisture = 20.0;      // percentage
        double soil_temperature = 10.0;   // Celsius
        bool tillage_required = false;
        uint8_t seed_bed_quality = 7;     // 0-10 scale
        bool fertilizer_application = false;
        double fertilizer_rate = 0.0;     // kg/ha
        std::string fertilizer_type = "starter";
        double fertilizer_depth = 0.05;   // meters
        bool gps_guidance = true;
        double section_control_width = 12.0; // meters
        bool overlap_reduction = true;
        bool boundary_tracking = true;
        double seed_level = 100.0;        // percentage
    } seeder_state_;
    
    // Hardware interface simulation
    struct HardwareInterface {
        bool can_bus_connected = false;
        std::string device_path = "/dev/can2";
        uint32_t can_id_base = 0x300;
        double metering_response_time = 2.0;  // seconds
        double depth_response_time = 5.0;     // seconds
    } hardware_;
    
    // Configuration parameters
    struct SeederConfig {
        double min_seed_rate = 10.0;       // kg/ha
        double max_seed_rate = 150.0;      // kg/ha
        double min_planting_depth = 0.01;  // 1cm
        double max_planting_depth = 0.08;  // 8cm
        double min_row_spacing = 0.15;     // 15cm
        double max_row_spacing = 1.0;      // 1m
        uint8_t num_rows = 12;
        std::vector<std::string> seed_types = {"corn", "soybean", "wheat", "canola", "sunflower"};
        std::vector<std::string> fertilizer_types = {"starter", "urea", "phosphorus", "potassium"};
    } config_;
    
    // Timers and callbacks
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::TimerBase::SharedPtr monitoring_timer_;
    rclcpp::TimerBase::SharedPtr calibration_timer_;
    
    // Callback methods
    void seeder_command_callback(const system_controller::msg::SeederCommand::SharedPtr msg);
    void status_timer_callback();
    void monitoring_timer_callback();
    void calibration_timer_callback();
    
    // Hardware communication methods
    bool send_can_message(uint32_t id, const std::vector<uint8_t>& data);
    bool read_can_message(uint32_t id, std::vector<uint8_t>& data);
    bool validate_hardware_connection();
    
    // Safety and validation methods
    bool validate_seeder_command(const system_controller::msg::SeederCommand& command);
    bool check_safety_limits();
    bool perform_self_test();
    bool check_seed_flow();
    
    // Utility methods
    double calculate_population_target(double rate, double spacing);
    bool update_metering_rates();
    bool monitor_emergence();
    std::string get_current_timestamp();
};

} // namespace system_controller

#endif // SEEDER_ADAPTER_HPP 