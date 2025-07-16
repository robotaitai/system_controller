#include "rclcpp/rclcpp.hpp"
#include "system_controller/msg/system_command.hpp"
#include "../include/policy_base.hpp"
#include <memory>
#include <vector>
#include <chrono>

class SystemManagerNode : public rclcpp::Node
{
public:
    SystemManagerNode() : Node("system_manager_node")
    {
        // Create publisher for policy commands
        policy_command_publisher_ = this->create_publisher<system_controller::msg::SystemCommand>(
            "/PolicyCommand", 10);

        // Initialize policies
        initializePolicies();

        // Create timer to publish commands periodically
        timer_ = this->create_wall_timer(
            std::chrono::seconds(3),
            std::bind(&SystemManagerNode::publishPolicyCommand, this));

        RCLCPP_INFO(this->get_logger(), "System Manager Node initialized with %zu policies", 
                    policies_.size());
    }

private:
    void initializePolicies()
    {
        policies_.push_back(std::make_unique<system_controller::Policy1>());
        policies_.push_back(std::make_unique<system_controller::Policy2>());
        policies_.push_back(std::make_unique<system_controller::Policy3>());
        
        // Set initial active policy
        current_policy_index_ = 0;
        
        RCLCPP_INFO(this->get_logger(), "Initialized %zu policies", policies_.size());
    }

    void publishPolicyCommand()
    {
        if (policies_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No policies available");
            return;
        }

        // Get command from current active policy
        auto& current_policy = policies_[current_policy_index_];
        std::string command = current_policy->getCommand();

        // Create and populate message
        auto message = system_controller::msg::SystemCommand();
        message.header.stamp = this->get_clock()->now();
        message.header.frame_id = "system_manager";
        message.command_type = "policy_command";
        message.command_data = command;
        message.priority = 1;
        message.timestamp = this->get_clock()->now().seconds();

        // Publish the command
        policy_command_publisher_->publish(message);

        RCLCPP_INFO(this->get_logger(), 
                    "Published policy command from %s: %s", 
                    current_policy->getName().c_str(), 
                    command.c_str());

        // Rotate to next policy for variety
        current_policy_index_ = (current_policy_index_ + 1) % policies_.size();
    }

    rclcpp::Publisher<system_controller::msg::SystemCommand>::SharedPtr policy_command_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::unique_ptr<system_controller::PolicyBase>> policies_;
    size_t current_policy_index_ = 0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SystemManagerNode>();
    
    RCLCPP_INFO(node->get_logger(), "System Manager Node started");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
} 