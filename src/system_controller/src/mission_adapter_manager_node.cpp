#include "rclcpp/rclcpp.hpp"
#include "system_controller/msg/mission_status.hpp"
#include "std_msgs/msg/string.hpp"
#include "../include/payload_type_base.hpp"
#include <memory>
#include <vector>

class MissionAdapterManagerNode : public rclcpp::Node
{
public:
    MissionAdapterManagerNode() : Node("mission_adapter_manager_node")
    {
        // Subscriber for mission service status
        mission_status_sub_ = this->create_subscription<system_controller::msg::MissionStatus>(
            "/MissionStatus", 10,
            std::bind(&MissionAdapterManagerNode::missionStatusCallback, this, std::placeholders::_1));

        // Publisher for mission hardware commands
        mission_hw_pub_ = this->create_publisher<std_msgs::msg::String>("/mission_hw_cmd", 10);

        // Initialize payload types
        initializePayloadTypes();

        // Timer for managing payload operations
        payload_timer_ = this->create_wall_timer(
            std::chrono::seconds(4),
            std::bind(&MissionAdapterManagerNode::managePayloads, this));

        RCLCPP_INFO(this->get_logger(), "Mission Adapter Manager Node initialized with %zu payload types", 
                    payload_types_.size());
    }

private:
    void initializePayloadTypes()
    {
        payload_types_.push_back(std::make_unique<system_controller::PayloadType1>());
        payload_types_.push_back(std::make_unique<system_controller::PayloadType2>());
        
        RCLCPP_INFO(this->get_logger(), "Initialized %zu payload types", payload_types_.size());
        
        // Log available payload types
        for (const auto& payload : payload_types_) {
            RCLCPP_INFO(this->get_logger(), "Available payload: %s", payload->getName().c_str());
        }
    }

    void missionStatusCallback(const system_controller::msg::MissionStatus::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), 
                    "Received mission status: %s (Progress: %.1f%%)", 
                    msg->status.c_str(), 
                    msg->progress_percentage);

        current_mission_status_ = *msg;
        
        // React to mission status changes
        if (msg->status.find("ACTIVE") != std::string::npos) {
            activatePayloadsForMission(msg->mission_type);
        } else if (msg->status.find("IDLE") != std::string::npos) {
            deactivateAllPayloads();
        }

        // Send commands to mission hardware based on status
        sendMissionHardwareCommand(msg);
    }

    void activatePayloadsForMission(const std::string& mission_type)
    {
        RCLCPP_INFO(this->get_logger(), "Activating payloads for mission type: %s", mission_type.c_str());
        
        for (auto& payload : payload_types_) {
            if (!payload->isActive()) {
                payload->activate();
                RCLCPP_INFO(this->get_logger(), "Activated payload: %s", payload->getName().c_str());
            }
            
            // Send mission-specific commands to payloads
            if (mission_type == "MissionType1") {
                payload->processCommand("start_surveillance");
            } else if (mission_type == "MissionType2") {
                payload->processCommand("prepare_delivery");
            }
        }
    }

    void deactivateAllPayloads()
    {
        RCLCPP_INFO(this->get_logger(), "Deactivating all payloads");
        
        for (auto& payload : payload_types_) {
            if (payload->isActive()) {
                payload->deactivate();
                RCLCPP_INFO(this->get_logger(), "Deactivated payload: %s", payload->getName().c_str());
            }
        }
    }

    void sendMissionHardwareCommand(const system_controller::msg::MissionStatus::SharedPtr msg)
    {
        auto hw_msg = std_msgs::msg::String();
        
        if (msg->status.find("ACTIVE") != std::string::npos) {
            hw_msg.data = "mission_execute:" + msg->mission_type + ":progress=" + 
                         std::to_string(msg->progress_percentage);
        } else {
            hw_msg.data = "mission_standby:" + msg->mission_type;
        }

        mission_hw_pub_->publish(hw_msg);
        
        RCLCPP_INFO(this->get_logger(), "Sent mission hardware command: %s", hw_msg.data.c_str());
    }

    void managePayloads()
    {
        // Periodic payload management
        for (auto& payload : payload_types_) {
            std::string status = payload->getStatus();
            RCLCPP_DEBUG(this->get_logger(), "Payload %s status: %s", 
                        payload->getName().c_str(), status.c_str());
            
            // Send periodic commands to active payloads
            if (payload->isActive()) {
                if (current_mission_status_.has_value()) {
                    float progress = current_mission_status_->progress_percentage;
                    if (progress > 50.0f) {
                        payload->processCommand("increase_data_rate");
                    } else {
                        payload->processCommand("standard_operation");
                    }
                }
            }
        }
        
        // Log overall payload status
        int active_payloads = 0;
        for (const auto& payload : payload_types_) {
            if (payload->isActive()) active_payloads++;
        }
        
        RCLCPP_INFO(this->get_logger(), "Active payloads: %d/%zu", 
                    active_payloads, payload_types_.size());
    }

    rclcpp::Subscription<system_controller::msg::MissionStatus>::SharedPtr mission_status_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_hw_pub_;
    rclcpp::TimerBase::SharedPtr payload_timer_;
    
    std::vector<std::unique_ptr<system_controller::PayloadTypeBase>> payload_types_;
    std::optional<system_controller::msg::MissionStatus> current_mission_status_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MissionAdapterManagerNode>();
    
    RCLCPP_INFO(node->get_logger(), "Mission Adapter Manager Node started");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
} 