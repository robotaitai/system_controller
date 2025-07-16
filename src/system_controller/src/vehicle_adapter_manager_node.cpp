#include "rclcpp/rclcpp.hpp"
#include "system_controller/msg/system_command.hpp"
#include "system_controller/msg/vehicle_status.hpp"
#include "system_controller/srv/get_status.hpp"
#include "std_msgs/msg/string.hpp"
#include "../include/vehicle_type_base.hpp"
#include <memory>
#include <vector>

class VehicleAdapterManagerNode : public rclcpp::Node
{
public:
    VehicleAdapterManagerNode() : Node("vehicle_adapter_manager_node")
    {
        // Subscriber for arbitrated commands
        command_sub_ = this->create_subscription<system_controller::msg::SystemCommand>(
            "/ArbitratedCommand", 10,
            std::bind(&VehicleAdapterManagerNode::commandCallback, this, std::placeholders::_1));

        // Publisher for vehicle hardware commands
        vehicle_hw_pub_ = this->create_publisher<std_msgs::msg::String>("/vehicle_hw_cmd", 10);

        // Publisher for vehicle status
        vehicle_status_pub_ = this->create_publisher<system_controller::msg::VehicleStatus>("/VehicleStatus", 10);

        // Service server for status requests
        status_service_ = this->create_service<system_controller::srv::GetStatus>(
            "vehicle_adapter_status",
            std::bind(&VehicleAdapterManagerNode::getStatusCallback, this, 
                     std::placeholders::_1, std::placeholders::_2));

        // Initialize vehicle types
        initializeVehicleTypes();

        // Timer for publishing vehicle status
        status_timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&VehicleAdapterManagerNode::publishVehicleStatus, this));

        RCLCPP_INFO(this->get_logger(), "Vehicle Adapter Manager Node initialized with %zu vehicle types", 
                    vehicle_types_.size());
    }

private:
    void initializeVehicleTypes()
    {
        vehicle_types_.push_back(std::make_unique<system_controller::VehicleType1>());
        vehicle_types_.push_back(std::make_unique<system_controller::VehicleType2>());
        
        // Initialize all vehicle types
        for (auto& vehicle_type : vehicle_types_) {
            vehicle_type->initialize();
        }
        
        // Set default active vehicle type
        current_vehicle_type_index_ = 0;
        
        RCLCPP_INFO(this->get_logger(), "Initialized %zu vehicle types, active: %s", 
                    vehicle_types_.size(),
                    vehicle_types_[current_vehicle_type_index_]->getName().c_str());
    }

    void commandCallback(const system_controller::msg::SystemCommand::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), 
                    "Received command: %s (type: %s)", 
                    msg->command_data.c_str(),
                    msg->command_type.c_str());

        if (vehicle_types_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No vehicle types available");
            return;
        }

        // Process command with current vehicle type
        auto& current_vehicle = vehicle_types_[current_vehicle_type_index_];
        current_vehicle->processCommand(msg->command_data);

        // Forward command to hardware interface
        auto hw_msg = std_msgs::msg::String();
        hw_msg.data = msg->command_data;
        vehicle_hw_pub_->publish(hw_msg);

        RCLCPP_INFO(this->get_logger(), "Forwarded command to vehicle hardware: %s", msg->command_data.c_str());
    }

    void publishVehicleStatus()
    {
        if (vehicle_types_.empty()) {
            return;
        }

        auto& current_vehicle = vehicle_types_[current_vehicle_type_index_];
        std::string status = current_vehicle->getStatus();

        auto status_msg = system_controller::msg::VehicleStatus();
        status_msg.header.stamp = this->get_clock()->now();
        status_msg.header.frame_id = "vehicle_adapter";
        status_msg.vehicle_id = "vehicle_001";
        status_msg.status = status;
        status_msg.battery_level = 0.85; // Example value
        
        // Set example pose and velocity
        status_msg.pose.position.x = 10.0;
        status_msg.pose.position.y = 5.0;
        status_msg.pose.position.z = 0.0;
        status_msg.velocity.linear.x = 0.5;
        
        status_msg.active_systems.push_back("navigation");
        status_msg.active_systems.push_back("propulsion");
        status_msg.active_systems.push_back(current_vehicle->getName());

        vehicle_status_pub_->publish(status_msg);

        RCLCPP_DEBUG(this->get_logger(), "Published vehicle status: %s", status.c_str());
    }

    void getStatusCallback(
        const std::shared_ptr<system_controller::srv::GetStatus::Request> request,
        std::shared_ptr<system_controller::srv::GetStatus::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Status request for component: %s", request->component_id.c_str());

        if (vehicle_types_.empty()) {
            response->success = false;
            response->status = "No vehicle types available";
            response->message = "Vehicle adapter not properly initialized";
            return;
        }

        auto& current_vehicle = vehicle_types_[current_vehicle_type_index_];
        response->success = true;
        response->status = current_vehicle->getStatus();
        response->message = "Vehicle adapter operational with " + current_vehicle->getName();

        RCLCPP_INFO(this->get_logger(), "Responded with status: %s", response->status.c_str());
    }

    // Switch between vehicle types (for demonstration)
    void switchVehicleType()
    {
        if (vehicle_types_.size() > 1) {
            current_vehicle_type_index_ = (current_vehicle_type_index_ + 1) % vehicle_types_.size();
            RCLCPP_INFO(this->get_logger(), "Switched to vehicle type: %s", 
                        vehicle_types_[current_vehicle_type_index_]->getName().c_str());
        }
    }

    rclcpp::Subscription<system_controller::msg::SystemCommand>::SharedPtr command_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr vehicle_hw_pub_;
    rclcpp::Publisher<system_controller::msg::VehicleStatus>::SharedPtr vehicle_status_pub_;
    rclcpp::Service<system_controller::srv::GetStatus>::SharedPtr status_service_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    
    std::vector<std::unique_ptr<system_controller::VehicleTypeBase>> vehicle_types_;
    size_t current_vehicle_type_index_ = 0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VehicleAdapterManagerNode>();
    
    RCLCPP_INFO(node->get_logger(), "Vehicle Adapter Manager Node started");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
} 