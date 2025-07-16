#include "rclcpp/rclcpp.hpp"
#include "system_controller/msg/mission_status.hpp"
#include "system_controller/msg/sprayer_command.hpp"
#include "system_controller/msg/mower_command.hpp"
#include "system_controller/msg/seeder_command.hpp"
#include "std_msgs/msg/string.hpp"
#include "../include/adapters/sprayer_adapter.hpp"
#include "../include/adapters/mower_adapter.hpp"
#include "../include/adapters/seeder_adapter.hpp"
#include <memory>
#include <vector>
#include <map>
#include <yaml-cpp/yaml.h>
#include <set>

class MissionAdapterManagerNode : public rclcpp::Node
{
public:
    MissionAdapterManagerNode() : Node("mission_adapter_manager_node")
    {
        // Load configuration
        loadConfiguration();
        
        // Initialize implement adapters based on configuration
        initializeImplementAdapters();
        
        // Subscriber for mission service status
        mission_status_sub_ = this->create_subscription<system_controller::msg::MissionStatus>(
            "/MissionStatus", 10,
            std::bind(&MissionAdapterManagerNode::missionStatusCallback, this, std::placeholders::_1));

        // Subscribers for implement-specific commands
        sprayer_cmd_sub_ = this->create_subscription<system_controller::msg::SprayerCommand>(
            "/SprayerCommand", 10,
            std::bind(&MissionAdapterManagerNode::sprayerCommandCallback, this, std::placeholders::_1));
            
        mower_cmd_sub_ = this->create_subscription<system_controller::msg::MowerCommand>(
            "/MowerCommand", 10,
            std::bind(&MissionAdapterManagerNode::mowerCommandCallback, this, std::placeholders::_1));
            
        seeder_cmd_sub_ = this->create_subscription<system_controller::msg::SeederCommand>(
            "/SeederCommand", 10,
            std::bind(&MissionAdapterManagerNode::seederCommandCallback, this, std::placeholders::_1));

        // Publisher for mission hardware status
        mission_hw_status_pub_ = this->create_publisher<std_msgs::msg::String>("/mission_hw_status", 10);

        // Timer for managing implement operations and health monitoring
        implement_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MissionAdapterManagerNode::manageImplements, this));

        RCLCPP_INFO(this->get_logger(), "Mission Adapter Manager Node initialized with %zu implement adapters", 
                    implement_adapters_.size());
    }

private:
    void loadConfiguration()
    {
        try {
            // Load missions configuration
            std::string config_path = "/ros2_ws/src/system_controller/config/missions.yaml";
            YAML::Node config = YAML::LoadFile(config_path);
            
            if (config["missions"]) {
                for (auto mission : config["missions"]) {
                    auto mission_name = mission.first.as<std::string>();
                    auto mission_config = mission.second;
                    
                    MissionConfig mission_cfg;
                    mission_cfg.mission_type = mission_config["mission_type"].as<std::string>();
                    mission_cfg.description = mission_config["description"].as<std::string>();
                    
                    if (mission_config["required_implements"]) {
                        for (auto impl : mission_config["required_implements"]) {
                            mission_cfg.required_implements.push_back(impl.as<std::string>());
                        }
                    }
                    
                    mission_configs_[mission_name] = mission_cfg;
                    RCLCPP_INFO(this->get_logger(), "Loaded mission config: %s (type: %s)", 
                                mission_name.c_str(), mission_cfg.mission_type.c_str());
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to load configuration: %s", e.what());
        }
    }
    
    void initializeImplementAdapters()
    {
        // Dynamically create adapters based on required implements from missions
        std::set<std::string> required_implements;
        
        for (const auto& [mission_name, config] : mission_configs_) {
            for (const auto& implement : config.required_implements) {
                required_implements.insert(implement);
            }
        }
        
        for (const auto& implement_id : required_implements) {
            createImplementAdapter(implement_id);
        }
        
        // Create default adapters if none specified
        if (implement_adapters_.empty()) {
            createImplementAdapter("boom_sprayer_24m");
            createImplementAdapter("rotary_mower_3m");
            createImplementAdapter("precision_seeder_12m");
        }
    }
    
    void createImplementAdapter(const std::string& implement_id)
    {
        std::shared_ptr<system_controller::ImplementAdapterBase> adapter;
        
        // Determine adapter type based on implement ID
        if (implement_id.find("sprayer") != std::string::npos || 
            implement_id.find("boom") != std::string::npos) {
            adapter = std::make_shared<system_controller::SprayerAdapter>();
            RCLCPP_INFO(this->get_logger(), "Created SprayerAdapter for: %s", implement_id.c_str());
        }
        else if (implement_id.find("mower") != std::string::npos || 
                 implement_id.find("cutter") != std::string::npos) {
            adapter = std::make_shared<system_controller::MowerAdapter>();
            RCLCPP_INFO(this->get_logger(), "Created MowerAdapter for: %s", implement_id.c_str());
        }
        else if (implement_id.find("seeder") != std::string::npos || 
                 implement_id.find("planter") != std::string::npos) {
            adapter = std::make_shared<system_controller::SeederAdapter>();
            RCLCPP_INFO(this->get_logger(), "Created SeederAdapter for: %s", implement_id.c_str());
        }
        else {
            RCLCPP_WARN(this->get_logger(), "Unknown implement type for: %s, creating generic adapter", implement_id.c_str());
            return;
        }
        
        // Initialize adapter
        system_controller::ImplementAdapterConfig config;
        config.adapter_id = implement_id;
        
        if (adapter->initialize(config)) {
            implement_adapters_[implement_id] = adapter;
            RCLCPP_INFO(this->get_logger(), "Successfully initialized adapter: %s", implement_id.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize adapter: %s", implement_id.c_str());
        }
    }

    void sprayerCommandCallback(const system_controller::msg::SprayerCommand::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received sprayer command - activate: %s, chemical: %s, rate: %.2f", 
                    msg->base.activate ? "true" : "false", 
                    msg->chemical_type.c_str(), 
                    msg->application_rate);
        
        // Route to appropriate sprayer adapter
        for (const auto& [id, adapter] : implement_adapters_) {
            if (id.find("sprayer") != std::string::npos) {
                auto sprayer_adapter = std::dynamic_pointer_cast<system_controller::SprayerAdapter>(adapter);
                if (sprayer_adapter) {
                    sprayer_adapter->processSprayerCommand(*msg);
                }
                break;
            }
        }
    }
    
    void mowerCommandCallback(const system_controller::msg::MowerCommand::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received mower command - activate: %s, height: %.3f, pattern: %s", 
                    msg->base.activate ? "true" : "false", 
                    msg->cutting_height, 
                    msg->cutting_pattern.c_str());
        
        // Route to appropriate mower adapter
        for (const auto& [id, adapter] : implement_adapters_) {
            if (id.find("mower") != std::string::npos) {
                auto mower_adapter = std::dynamic_pointer_cast<system_controller::MowerAdapter>(adapter);
                if (mower_adapter) {
                    mower_adapter->processMowerCommand(*msg);
                }
                break;
            }
        }
    }
    
    void seederCommandCallback(const system_controller::msg::SeederCommand::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received seeder command - activate: %s, rate: %.0f, depth: %.3f", 
                    msg->base.activate ? "true" : "false", 
                    msg->seed_rate, 
                    msg->planting_depth);
        
        // Route to appropriate seeder adapter
        for (const auto& [id, adapter] : implement_adapters_) {
            if (id.find("seeder") != std::string::npos) {
                auto seeder_adapter = std::dynamic_pointer_cast<system_controller::SeederAdapter>(adapter);
                if (seeder_adapter) {
                    seeder_adapter->processSeederCommand(*msg);
                }
                break;
            }
        }
    }

    void missionStatusCallback(const system_controller::msg::MissionStatus::SharedPtr msg)
    {
        current_mission_status_ = *msg;
        
        RCLCPP_INFO(this->get_logger(), 
                    "Mission status: %s (Progress: %.1f%%)", 
                    msg->status.c_str(), 
                    msg->progress);
                    
        // Update implement adapters based on mission status
        updateImplementsBasedOnMissionStatus(*msg);
    }
    
    void updateImplementsBasedOnMissionStatus(const system_controller::msg::MissionStatus& status)
    {
        // Determine which implements should be active based on mission status
        if (status.status == "active") {
            // Activate required implements for current mission
            activateRequiredImplements();
        } else if (status.status == "paused" || status.status == "stopped") {
            // Safely stop all implements
            stopAllImplements();
        } else if (status.status == "emergency_stop") {
            // Emergency stop all implements
            emergencyStopAllImplements();
        }
    }
    
    void activateRequiredImplements()
    {
        // This would be enhanced to activate specific implements based on current mission
        for (const auto& [id, adapter] : implement_adapters_) {
            if (adapter->isHealthy()) {
                RCLCPP_DEBUG(this->get_logger(), "Implement %s is ready and healthy", id.c_str());
            }
        }
    }
    
    void stopAllImplements()
    {
        for (const auto& [id, adapter] : implement_adapters_) {
            // Send stop commands to all adapters
            system_controller::ImplementCommand stop_cmd;
            stop_cmd.activate = false;
            stop_cmd.emergency_stop = false;
            adapter->processImplementCommand(stop_cmd);
            
            RCLCPP_INFO(this->get_logger(), "Stopped implement: %s", id.c_str());
        }
    }
    
    void emergencyStopAllImplements()
    {
        for (const auto& [id, adapter] : implement_adapters_) {
            // Send emergency stop to all adapters
            system_controller::ImplementCommand estop_cmd;
            estop_cmd.activate = false;
            estop_cmd.emergency_stop = true;
            adapter->processImplementCommand(estop_cmd);
            
            RCLCPP_WARN(this->get_logger(), "Emergency stopped implement: %s", id.c_str());
        }
    }

    void manageImplements()
    {
        // Monitor health and status of all implement adapters
        std::string status_message = "Implement Status: ";
        bool all_healthy = true;
        
        for (const auto& [id, adapter] : implement_adapters_) {
            bool healthy = adapter->isHealthy();
            status_message += id + ":" + (healthy ? "OK" : "ERROR") + " ";
            
            if (!healthy) {
                all_healthy = false;
                RCLCPP_WARN(this->get_logger(), "Implement %s is not healthy", id.c_str());
            }
        }
        
        // Publish status
        auto status_msg = std_msgs::msg::String();
        status_msg.data = status_message;
        mission_hw_status_pub_->publish(status_msg);
        
        // Log overall health status
        if (!all_healthy) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                                 "Some implements are not healthy!");
        }
    }

    // Configuration structures
    struct MissionConfig {
        std::string mission_type;
        std::string description;
        std::vector<std::string> required_implements;
    };

    // Member variables
    std::map<std::string, MissionConfig> mission_configs_;
    std::map<std::string, std::shared_ptr<system_controller::ImplementAdapterBase>> implement_adapters_;
    system_controller::msg::MissionStatus current_mission_status_;

    // ROS subscribers and publishers
    rclcpp::Subscription<system_controller::msg::MissionStatus>::SharedPtr mission_status_sub_;
    rclcpp::Subscription<system_controller::msg::SprayerCommand>::SharedPtr sprayer_cmd_sub_;
    rclcpp::Subscription<system_controller::msg::MowerCommand>::SharedPtr mower_cmd_sub_;
    rclcpp::Subscription<system_controller::msg::SeederCommand>::SharedPtr seeder_cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_hw_status_pub_;
    rclcpp::TimerBase::SharedPtr implement_timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionAdapterManagerNode>());
    rclcpp::shutdown();
    return 0;
} 