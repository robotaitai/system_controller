#include "rclcpp/rclcpp.hpp"
#include "system_controller/msg/vehicle_status.hpp"
#include "system_controller/msg/mission_status.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include <memory>

class TelemetryCollectorNode : public rclcpp::Node
{
public:
    TelemetryCollectorNode() : Node("telemetry_collector_node")
    {
        // Subscribers for hardware status topics
        vehicle_status_sub_ = this->create_subscription<system_controller::msg::VehicleStatus>(
            "/VehicleStatus", 10,
            std::bind(&TelemetryCollectorNode::vehicleStatusCallback, this, std::placeholders::_1));

        mission_status_sub_ = this->create_subscription<system_controller::msg::MissionStatus>(
            "/MissionStatus", 10,
            std::bind(&TelemetryCollectorNode::missionStatusCallback, this, std::placeholders::_1));

        // Publishers for redistributed telemetry
        vehicle_status_to_mission_pub_ = this->create_publisher<system_controller::msg::VehicleStatus>(
            "/telemetry/vehicle_to_mission", 10);

        mission_status_to_system_pub_ = this->create_publisher<system_controller::msg::MissionStatus>(
            "/telemetry/mission_to_system", 10);

        combined_telemetry_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>(
            "/telemetry/combined", 10);

        // Timer for publishing combined telemetry
        telemetry_timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&TelemetryCollectorNode::publishCombinedTelemetry, this));

        RCLCPP_INFO(this->get_logger(), "Telemetry Collector Node initialized");
    }

private:
    void vehicleStatusCallback(const system_controller::msg::VehicleStatus::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), 
                    "Collected vehicle telemetry: %s (Battery: %.1f%%)", 
                    msg->status.c_str(), 
                    msg->battery_level * 100.0);

        // Store latest vehicle status
        latest_vehicle_status_ = *msg;
        
        // Forward to mission service
        vehicle_status_to_mission_pub_->publish(*msg);
        
        RCLCPP_DEBUG(this->get_logger(), "Forwarded vehicle status to mission service");
    }

    void missionStatusCallback(const system_controller::msg::MissionStatus::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), 
                    "Collected mission telemetry: %s (Progress: %.1f%%)", 
                    msg->status.c_str(), 
                    msg->progress_percentage);

        // Store latest mission status
        latest_mission_status_ = *msg;
        
        // Forward to system controller
        mission_status_to_system_pub_->publish(*msg);
        
        RCLCPP_DEBUG(this->get_logger(), "Forwarded mission status to system controller");
    }

    void publishCombinedTelemetry()
    {
        // Create a combined telemetry message using BatteryState as an example
        auto combined_msg = sensor_msgs::msg::BatteryState();
        combined_msg.header.stamp = this->get_clock()->now();
        combined_msg.header.frame_id = "telemetry_collector";

        if (latest_vehicle_status_.has_value()) {
            combined_msg.voltage = latest_vehicle_status_->battery_level * 12.0; // Example voltage
            combined_msg.percentage = latest_vehicle_status_->battery_level;
            combined_msg.present = true;
        } else {
            combined_msg.present = false;
        }

        combined_telemetry_pub_->publish(combined_msg);

        // Log telemetry summary
        std::string vehicle_summary = latest_vehicle_status_.has_value() ? 
            latest_vehicle_status_->status : "No vehicle data";
        std::string mission_summary = latest_mission_status_.has_value() ? 
            latest_mission_status_->status : "No mission data";

        RCLCPP_INFO(this->get_logger(), 
                    "Combined telemetry - Vehicle: %s, Mission: %s", 
                    vehicle_summary.c_str(), 
                    mission_summary.c_str());
    }

    rclcpp::Subscription<system_controller::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Subscription<system_controller::msg::MissionStatus>::SharedPtr mission_status_sub_;
    
    rclcpp::Publisher<system_controller::msg::VehicleStatus>::SharedPtr vehicle_status_to_mission_pub_;
    rclcpp::Publisher<system_controller::msg::MissionStatus>::SharedPtr mission_status_to_system_pub_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr combined_telemetry_pub_;
    
    rclcpp::TimerBase::SharedPtr telemetry_timer_;
    
    std::optional<system_controller::msg::VehicleStatus> latest_vehicle_status_;
    std::optional<system_controller::msg::MissionStatus> latest_mission_status_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TelemetryCollectorNode>();
    
    RCLCPP_INFO(node->get_logger(), "Telemetry Collector Node started");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
} 