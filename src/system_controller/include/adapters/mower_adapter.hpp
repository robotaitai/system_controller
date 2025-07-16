#ifndef MOWER_ADAPTER_HPP
#define MOWER_ADAPTER_HPP

#include "adapters/implement_adapter_base.hpp"
#include "system_controller/msg/mower_command.hpp"
#include "system_controller/msg/implement_command.hpp"
#include <rclcpp/rclcpp.hpp>
#include <map>
#include <string>

namespace system_controller
{

class MowerAdapter : public ImplementAdapterBase
{
public:
    explicit MowerAdapter(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    virtual ~MowerAdapter() = default;

    // Core adapter interface
    bool initialize() override;
    bool shutdown() override;
    bool process_command(const system_controller::msg::ImplementCommand& command) override;
    
    // Mower-specific command processing
    bool process_mower_command(const system_controller::msg::MowerCommand& command);
    
    // Mower control methods
    bool set_cutting_height(double height);
    bool set_blade_speed(double speed);
    bool control_side_discharge(bool enable);
    bool set_cutting_pattern(uint8_t pattern);
    bool set_overlap_percentage(double percentage);
    
    // Advanced mower features
    bool enable_mulching_mode(bool enable);
    bool enable_grass_collection(bool enable);
    bool set_ground_speed(double speed);
    bool enable_auto_height_control(bool enable);
    bool set_blade_engagement(double engagement);
    
    // Terrain adaptation
    bool enable_slope_compensation(bool enable);
    bool set_max_slope_angle(double angle);
    bool enable_anti_scalp_wheels(bool enable);
    bool enable_deck_float(bool enable);
    
    // Safety and monitoring
    bool check_operator_presence();
    bool emergency_stop_mower();
    bool perform_blade_check();
    
    // Configuration and status
    void load_mower_config();
    bool get_mower_status(std::map<std::string, double>& status);
    bool calibrate_mower();

private:
    // ROS subscriptions and publishers
    rclcpp::Subscription<system_controller::msg::MowerCommand>::SharedPtr mower_command_sub_;
    rclcpp::Publisher<system_controller::msg::MissionStatus>::SharedPtr status_pub_;
    
    // Mower state variables
    struct MowerState {
        bool blades_engaged = false;
        double cutting_height = 0.05;     // 5cm default
        double blade_speed = 0.0;         // RPM
        bool side_discharge = false;
        uint8_t cutting_pattern = 0;      // 0=stripe, 1=spiral, 2=random
        double overlap_percentage = 10.0;
        bool mulching_mode = false;
        bool grass_collection = false;
        double ground_speed = 2.0;        // m/s
        bool auto_height_control = false;
        double blade_engagement = 0.0;    // 0-100%
        bool slope_compensation = false;
        double max_slope_angle = 15.0;    // degrees
        bool anti_scalp_wheels = true;
        bool deck_float = true;
        uint8_t cut_quality_mode = 1;     // 0=fast, 1=normal, 2=fine
        bool striping_kit = false;
        double blade_sharpness = 85.0;    // percentage
        bool operator_presence = false;
    } mower_state_;
    
    // Hardware interface simulation
    struct HardwareInterface {
        bool can_bus_connected = false;
        std::string device_path = "/dev/can1";
        uint32_t can_id_base = 0x200;
        double blade_response_time = 1.0;  // seconds
        double height_response_time = 3.0; // seconds
    } hardware_;
    
    // Configuration parameters
    struct MowerConfig {
        double min_cutting_height = 0.02;  // 2cm
        double max_cutting_height = 0.15;  // 15cm
        double max_blade_speed = 3600.0;   // RPM
        double max_ground_speed = 5.0;     // m/s
        double deck_width = 1.8;           // meters
        uint8_t num_blades = 3;
        std::vector<std::string> cutting_patterns = {"stripe", "spiral", "random"};
    } config_;
    
    // Timers and callbacks
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::TimerBase::SharedPtr monitoring_timer_;
    rclcpp::TimerBase::SharedPtr safety_timer_;
    
    // Callback methods
    void mower_command_callback(const system_controller::msg::MowerCommand::SharedPtr msg);
    void status_timer_callback();
    void monitoring_timer_callback();
    void safety_timer_callback();
    
    // Hardware communication methods
    bool send_can_message(uint32_t id, const std::vector<uint8_t>& data);
    bool read_can_message(uint32_t id, std::vector<uint8_t>& data);
    bool validate_hardware_connection();
    
    // Safety and validation methods
    bool validate_mower_command(const system_controller::msg::MowerCommand& command);
    bool check_safety_limits();
    bool perform_self_test();
    bool check_blade_condition();
    
    // Utility methods
    double calculate_cutting_efficiency(double speed, double height);
    bool update_cutting_parameters();
    std::string get_current_timestamp();
};

} // namespace system_controller

#endif // MOWER_ADAPTER_HPP 