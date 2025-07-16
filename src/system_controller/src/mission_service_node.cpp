#include "rclcpp/rclcpp.hpp"
#include "system_controller/msg/mission_status.hpp"
#include "system_controller/srv/get_status.hpp"
#include "std_msgs/msg/string.hpp"
#include "../include/mission_type_base.hpp"
#include <memory>
#include <vector>

class MissionServiceNode : public rclcpp::Node
{
public:
    MissionServiceNode() : Node("mission_service_node")
    {
        // Subscriber for mission selection commands
        mission_command_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/MissionSel/Command", 10,
            std::bind(&MissionServiceNode::missionCommandCallback, this, std::placeholders::_1));

        // Publisher for mission status
        mission_status_pub_ = this->create_publisher<system_controller::msg::MissionStatus>("/MissionStatus", 10);

        // Service server for status requests
        status_service_ = this->create_service<system_controller::srv::GetStatus>(
            "mission_service_status",
            std::bind(&MissionServiceNode::getStatusCallback, this, 
                     std::placeholders::_1, std::placeholders::_2));

        // Initialize mission types
        initializeMissionTypes();

        // Timer for updating mission progress and publishing status
        mission_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MissionServiceNode::updateAndPublishMissionStatus, this));

        RCLCPP_INFO(this->get_logger(), "Mission Service Node initialized with %zu mission types", 
                    mission_types_.size());
    }

private:
    void initializeMissionTypes()
    {
        mission_types_.push_back(std::make_unique<system_controller::MissionType1>());
        mission_types_.push_back(std::make_unique<system_controller::MissionType2>());
        
        // Set default active mission type (initially not started)
        current_mission_type_index_ = 0;
        
        RCLCPP_INFO(this->get_logger(), "Initialized %zu mission types, current: %s", 
                    mission_types_.size(),
                    mission_types_[current_mission_type_index_]->getName().c_str());
    }

    void missionCommandCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received mission command: %s", msg->data.c_str());

        if (mission_types_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No mission types available");
            return;
        }

        auto& current_mission = mission_types_[current_mission_type_index_];

        if (msg->data == "start") {
            current_mission->startMission();
            RCLCPP_INFO(this->get_logger(), "Started mission: %s", current_mission->getName().c_str());
        }
        else if (msg->data == "stop") {
            current_mission->stopMission();
            RCLCPP_INFO(this->get_logger(), "Stopped mission: %s", current_mission->getName().c_str());
        }
        else if (msg->data == "switch") {
            // Switch to next mission type
            current_mission_type_index_ = (current_mission_type_index_ + 1) % mission_types_.size();
            RCLCPP_INFO(this->get_logger(), "Switched to mission: %s", 
                        mission_types_[current_mission_type_index_]->getName().c_str());
        }
        else {
            RCLCPP_WARN(this->get_logger(), "Unknown mission command: %s", msg->data.c_str());
        }
    }

    void updateAndPublishMissionStatus()
    {
        if (mission_types_.empty()) {
            return;
        }

        auto& current_mission = mission_types_[current_mission_type_index_];
        
        // Update mission progress
        current_mission->updateMission();

        // Create and publish status message
        auto status_msg = system_controller::msg::MissionStatus();
        status_msg.header.stamp = this->get_clock()->now();
        status_msg.header.frame_id = "mission_service";
        status_msg.mission_id = "mission_001";
        status_msg.mission_type = current_mission->getName();
        status_msg.status = current_mission->getStatus();
        status_msg.progress_percentage = current_mission->getProgress();
        status_msg.current_waypoint = "waypoint_" + std::to_string(static_cast<int>(current_mission->getProgress() / 25));
        
        // Add payload status examples
        if (current_mission->isActive()) {
            status_msg.payload_status.push_back("camera_active");
            status_msg.payload_status.push_back("sensors_collecting");
        } else {
            status_msg.payload_status.push_back("payloads_idle");
        }

        mission_status_pub_->publish(status_msg);

        RCLCPP_DEBUG(this->get_logger(), 
                     "Published mission status: %s (%.1f%%)", 
                     status_msg.status.c_str(), 
                     status_msg.progress_percentage);
    }

    void getStatusCallback(
        const std::shared_ptr<system_controller::srv::GetStatus::Request> request,
        std::shared_ptr<system_controller::srv::GetStatus::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Status request for component: %s", request->component_id.c_str());

        if (mission_types_.empty()) {
            response->success = false;
            response->status = "No mission types available";
            response->message = "Mission service not properly initialized";
            return;
        }

        auto& current_mission = mission_types_[current_mission_type_index_];
        response->success = true;
        response->status = current_mission->getStatus();
        response->message = "Mission service operational with " + current_mission->getName() + 
                           " (Progress: " + std::to_string(current_mission->getProgress()) + "%)";

        RCLCPP_INFO(this->get_logger(), "Responded with status: %s", response->status.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_command_sub_;
    rclcpp::Publisher<system_controller::msg::MissionStatus>::SharedPtr mission_status_pub_;
    rclcpp::Service<system_controller::srv::GetStatus>::SharedPtr status_service_;
    rclcpp::TimerBase::SharedPtr mission_timer_;
    
    std::vector<std::unique_ptr<system_controller::MissionTypeBase>> mission_types_;
    size_t current_mission_type_index_ = 0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MissionServiceNode>();
    
    RCLCPP_INFO(node->get_logger(), "Mission Service Node started");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
} 