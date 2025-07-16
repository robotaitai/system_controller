#include "rclcpp/rclcpp.hpp"
#include "system_controller/msg/system_command.hpp"
#include "system_controller/srv/get_status.hpp"
#include "std_msgs/msg/string.hpp"
#include <memory>
#include <chrono>

class CommandInputArbiterNode : public rclcpp::Node
{
public:
    CommandInputArbiterNode() : Node("command_input_arbiter_node")
    {
        // Subscribers for different command sources
        policy_command_sub_ = this->create_subscription<system_controller::msg::SystemCommand>(
            "/PolicyCommand", 10,
            std::bind(&CommandInputArbiterNode::policyCommandCallback, this, std::placeholders::_1));

        teleop_command_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/Teleop/Command", 10,
            std::bind(&CommandInputArbiterNode::teleopCommandCallback, this, std::placeholders::_1));

        autonomy_command_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/Autonomy/Command", 10,
            std::bind(&CommandInputArbiterNode::autonomyCommandCallback, this, std::placeholders::_1));

        // Publisher for arbitrated commands
        arbitrated_command_pub_ = this->create_publisher<system_controller::msg::SystemCommand>(
            "/ArbitratedCommand", 10);

        // Service client to get status from adapter manager
        status_client_ = this->create_client<system_controller::srv::GetStatus>("vehicle_adapter_status");

        // Timer for periodic status checks
        status_timer_ = this->create_wall_timer(
            std::chrono::seconds(10),
            std::bind(&CommandInputArbiterNode::checkAdapterStatus, this));

        RCLCPP_INFO(this->get_logger(), "Command Input Arbiter Node initialized");
    }

private:
    void policyCommandCallback(const system_controller::msg::SystemCommand::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), 
                    "Received policy command: %s (priority: %d)", 
                    msg->command_data.c_str(), msg->priority);
        
        last_policy_command_ = *msg;
        last_policy_time_ = this->get_clock()->now();
        
        arbitrateAndPublish();
    }

    void teleopCommandCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received teleop command: %s", msg->data.c_str());
        
        // Convert to SystemCommand format
        system_controller::msg::SystemCommand cmd;
        cmd.header.stamp = this->get_clock()->now();
        cmd.header.frame_id = "teleop";
        cmd.command_type = "teleop_command";
        cmd.command_data = msg->data;
        cmd.priority = 3; // High priority for teleop
        cmd.timestamp = this->get_clock()->now().seconds();
        
        last_teleop_command_ = cmd;
        last_teleop_time_ = this->get_clock()->now();
        
        arbitrateAndPublish();
    }

    void autonomyCommandCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received autonomy command: %s", msg->data.c_str());
        
        // Convert to SystemCommand format
        system_controller::msg::SystemCommand cmd;
        cmd.header.stamp = this->get_clock()->now();
        cmd.header.frame_id = "autonomy";
        cmd.command_type = "autonomy_command";
        cmd.command_data = msg->data;
        cmd.priority = 2; // Medium priority for autonomy
        cmd.timestamp = this->get_clock()->now().seconds();
        
        last_autonomy_command_ = cmd;
        last_autonomy_time_ = this->get_clock()->now();
        
        arbitrateAndPublish();
    }

    void arbitrateAndPublish()
    {
        auto now = this->get_clock()->now();
        auto timeout = rclcpp::Duration::from_seconds(5.0); // 5 second timeout
        
        system_controller::msg::SystemCommand selected_command;
        std::string source = "none";
        
        // Simple arbitration: latest command wins, with priority consideration
        // Check teleop first (highest priority)
        if (last_teleop_time_.has_value() && (now - last_teleop_time_.value()) < timeout) {
            selected_command = last_teleop_command_.value();
            source = "teleop";
        }
        // Then autonomy
        else if (last_autonomy_time_.has_value() && (now - last_autonomy_time_.value()) < timeout) {
            selected_command = last_autonomy_command_.value();
            source = "autonomy";
        }
        // Finally policy
        else if (last_policy_time_.has_value() && (now - last_policy_time_.value()) < timeout) {
            selected_command = last_policy_command_.value();
            source = "policy";
        }
        else {
            RCLCPP_WARN(this->get_logger(), "No valid commands available for arbitration");
            return;
        }

        // Publish arbitrated command
        arbitrated_command_pub_->publish(selected_command);
        
        RCLCPP_INFO(this->get_logger(), 
                    "Arbitrated command from %s: %s", 
                    source.c_str(), 
                    selected_command.command_data.c_str());
    }

    void checkAdapterStatus()
    {
        if (!status_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Adapter status service not available");
            return;
        }

        auto request = std::make_shared<system_controller::srv::GetStatus::Request>();
        request->component_id = "vehicle_adapter";

        auto future = status_client_->async_send_request(request);
        
        // Note: In a real implementation, you'd want to handle the response properly
        RCLCPP_DEBUG(this->get_logger(), "Requesting adapter status...");
    }

    // Subscribers
    rclcpp::Subscription<system_controller::msg::SystemCommand>::SharedPtr policy_command_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr teleop_command_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr autonomy_command_sub_;
    
    // Publisher
    rclcpp::Publisher<system_controller::msg::SystemCommand>::SharedPtr arbitrated_command_pub_;
    
    // Service client
    rclcpp::Client<system_controller::srv::GetStatus>::SharedPtr status_client_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr status_timer_;
    
    // Last received commands and timestamps
    std::optional<system_controller::msg::SystemCommand> last_policy_command_;
    std::optional<system_controller::msg::SystemCommand> last_teleop_command_;
    std::optional<system_controller::msg::SystemCommand> last_autonomy_command_;
    
    std::optional<rclcpp::Time> last_policy_time_;
    std::optional<rclcpp::Time> last_teleop_time_;
    std::optional<rclcpp::Time> last_autonomy_time_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CommandInputArbiterNode>();
    
    RCLCPP_INFO(node->get_logger(), "Command Input Arbiter Node started");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
} 