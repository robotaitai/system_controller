#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <chrono>
#include <thread>

#include "system_controller/msg/system_command.hpp"
#include "system_controller/msg/vehicle_status.hpp"
#include "system_controller/msg/mission_status.hpp"
#include "system_controller/srv/get_status.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace system_controller {
namespace test {

class ROS2IntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        
        // Create test node
        test_node_ = rclcpp::Node::make_shared("test_node");
        
        // Set up publishers for testing
        policy_pub_ = test_node_->create_publisher<system_controller::msg::SystemCommand>(
            "/PolicyCommand", 10);
        teleop_pub_ = test_node_->create_publisher<std_msgs::msg::String>(
            "/Teleop/Command", 10);
        autonomy_pub_ = test_node_->create_publisher<std_msgs::msg::String>(
            "/Autonomy/Command", 10);
        mission_pub_ = test_node_->create_publisher<std_msgs::msg::String>(
            "/MissionSel/Command", 10);
        
        // Set up subscribers for monitoring
        arbitrated_sub_ = test_node_->create_subscription<system_controller::msg::SystemCommand>(
            "/ArbitratedCommand", 10,
            [this](const system_controller::msg::SystemCommand::SharedPtr msg) {
                last_arbitrated_command_ = *msg;
                arbitrated_received_ = true;
            });
            
        vehicle_status_sub_ = test_node_->create_subscription<system_controller::msg::VehicleStatus>(
            "/VehicleStatus", 10,
            [this](const system_controller::msg::VehicleStatus::SharedPtr msg) {
                last_vehicle_status_ = *msg;
                vehicle_status_received_ = true;
            });
            
        mission_status_sub_ = test_node_->create_subscription<system_controller::msg::MissionStatus>(
            "/MissionStatus", 10,
            [this](const system_controller::msg::MissionStatus::SharedPtr msg) {
                last_mission_status_ = *msg;
                mission_status_received_ = true;
            });
        
        // Set up service clients
        vehicle_status_client_ = test_node_->create_client<system_controller::srv::GetStatus>(
            "vehicle_adapter_status");
        mission_status_client_ = test_node_->create_client<system_controller::srv::GetStatus>(
            "mission_service_status");
        
        // Reset flags
        resetFlags();
        
        // Allow time for discovery
        std::this_thread::sleep_for(500ms);
    }
    
    void TearDown() override {
        rclcpp::shutdown();
    }
    
    void resetFlags() {
        arbitrated_received_ = false;
        vehicle_status_received_ = false;
        mission_status_received_ = false;
    }
    
    void spinFor(std::chrono::milliseconds duration) {
        auto start = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start < duration) {
            rclcpp::spin_some(test_node_);
            std::this_thread::sleep_for(10ms);
        }
    }
    
    rclcpp::Node::SharedPtr test_node_;
    
    // Publishers
    rclcpp::Publisher<system_controller::msg::SystemCommand>::SharedPtr policy_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr teleop_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr autonomy_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_pub_;
    
    // Subscribers  
    rclcpp::Subscription<system_controller::msg::SystemCommand>::SharedPtr arbitrated_sub_;
    rclcpp::Subscription<system_controller::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Subscription<system_controller::msg::MissionStatus>::SharedPtr mission_status_sub_;
    
    // Service clients
    rclcpp::Client<system_controller::srv::GetStatus>::SharedPtr vehicle_status_client_;
    rclcpp::Client<system_controller::srv::GetStatus>::SharedPtr mission_status_client_;
    
    // Received messages
    system_controller::msg::SystemCommand last_arbitrated_command_;
    system_controller::msg::VehicleStatus last_vehicle_status_;
    system_controller::msg::MissionStatus last_mission_status_;
    
    // Flags
    bool arbitrated_received_ = false;
    bool vehicle_status_received_ = false;
    bool mission_status_received_ = false;
};

// ============================================================================
// Message Flow Tests
// ============================================================================

TEST_F(ROS2IntegrationTest, PolicyCommandFlow) {
    // Create and publish a policy command
    auto msg = system_controller::msg::SystemCommand();
    msg.header.stamp = test_node_->get_clock()->now();
    msg.header.frame_id = "test";
    msg.command_type = "policy_command";
    msg.command_data = "test_policy_command";
    msg.priority = 1;
    msg.timestamp = test_node_->get_clock()->now().seconds();
    
    policy_pub_->publish(msg);
    
    // Wait for arbitrated command
    spinFor(2000ms);
    
    EXPECT_TRUE(arbitrated_received_);
    EXPECT_EQ(last_arbitrated_command_.command_data, "test_policy_command");
    EXPECT_EQ(last_arbitrated_command_.command_type, "policy_command");
}

TEST_F(ROS2IntegrationTest, TeleopCommandPriority) {
    resetFlags();
    
    // Send policy command first
    auto policy_msg = std_msgs::msg::String();
    policy_msg.data = "policy_move_forward";
    
    // Create policy command
    auto sys_msg = system_controller::msg::SystemCommand();
    sys_msg.header.stamp = test_node_->get_clock()->now();
    sys_msg.command_type = "policy_command";
    sys_msg.command_data = "policy_move_forward";
    sys_msg.priority = 1;
    
    policy_pub_->publish(sys_msg);
    spinFor(100ms);
    
    // Send teleop command (should override)
    auto teleop_msg = std_msgs::msg::String();
    teleop_msg.data = "emergency_stop";
    teleop_pub_->publish(teleop_msg);
    
    spinFor(2000ms);
    
    EXPECT_TRUE(arbitrated_received_);
    // Teleop should have higher priority
    EXPECT_EQ(last_arbitrated_command_.command_data, "emergency_stop");
    EXPECT_EQ(last_arbitrated_command_.command_type, "teleop_command");
}

TEST_F(ROS2IntegrationTest, VehicleStatusPublication) {
    // Send a command and check if vehicle status is published
    auto msg = std_msgs::msg::String();
    msg.data = "test_vehicle_command";
    teleop_pub_->publish(msg);
    
    spinFor(3000ms);
    
    EXPECT_TRUE(vehicle_status_received_);
    EXPECT_FALSE(last_vehicle_status_.vehicle_id.empty());
    EXPECT_FALSE(last_vehicle_status_.status.empty());
    EXPECT_GE(last_vehicle_status_.battery_level, 0.0);
    EXPECT_LE(last_vehicle_status_.battery_level, 1.0);
}

TEST_F(ROS2IntegrationTest, MissionStatusPublication) {
    // Send mission command
    auto msg = std_msgs::msg::String();
    msg.data = "start";
    mission_pub_->publish(msg);
    
    spinFor(2000ms);
    
    EXPECT_TRUE(mission_status_received_);
    EXPECT_FALSE(last_mission_status_.mission_id.empty());
    EXPECT_FALSE(last_mission_status_.mission_type.empty());
    EXPECT_FALSE(last_mission_status_.status.empty());
    EXPECT_GE(last_mission_status_.progress_percentage, 0.0f);
    EXPECT_LE(last_mission_status_.progress_percentage, 100.0f);
}

// ============================================================================
// Service Tests
// ============================================================================

TEST_F(ROS2IntegrationTest, VehicleStatusService) {
    // Wait for service to be available
    ASSERT_TRUE(vehicle_status_client_->wait_for_service(5s));
    
    auto request = std::make_shared<system_controller::srv::GetStatus::Request>();
    request->component_id = "test_vehicle";
    
    auto future = vehicle_status_client_->async_send_request(request);
    
    // Wait for response
    auto status = rclcpp::spin_until_future_complete(test_node_, future, 5s);
    ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);
    
    auto response = future.get();
    EXPECT_TRUE(response->success);
    EXPECT_FALSE(response->status.empty());
    EXPECT_FALSE(response->message.empty());
}

TEST_F(ROS2IntegrationTest, MissionStatusService) {
    // Wait for service to be available
    ASSERT_TRUE(mission_status_client_->wait_for_service(5s));
    
    auto request = std::make_shared<system_controller::srv::GetStatus::Request>();
    request->component_id = "test_mission";
    
    auto future = mission_status_client_->async_send_request(request);
    
    // Wait for response
    auto status = rclcpp::spin_until_future_complete(test_node_, future, 5s);
    ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);
    
    auto response = future.get();
    EXPECT_TRUE(response->success);
    EXPECT_FALSE(response->status.empty());
    EXPECT_FALSE(response->message.empty());
}

// ============================================================================
// End-to-End System Tests
// ============================================================================

TEST_F(ROS2IntegrationTest, FullSystemWorkflow) {
    resetFlags();
    
    // 1. Start mission
    auto mission_msg = std_msgs::msg::String();
    mission_msg.data = "start";
    mission_pub_->publish(mission_msg);
    
    spinFor(1000ms);
    EXPECT_TRUE(mission_status_received_);
    
    // 2. Send policy command
    auto policy_msg = system_controller::msg::SystemCommand();
    policy_msg.header.stamp = test_node_->get_clock()->now();
    policy_msg.command_type = "policy_command";
    policy_msg.command_data = "move_forward:speed=0.5";
    policy_msg.priority = 1;
    
    resetFlags();
    policy_pub_->publish(policy_msg);
    
    spinFor(1000ms);
    EXPECT_TRUE(arbitrated_received_);
    EXPECT_TRUE(vehicle_status_received_);
    
    // 3. Override with teleop
    auto teleop_msg = std_msgs::msg::String();
    teleop_msg.data = "emergency_stop";
    
    resetFlags();
    teleop_pub_->publish(teleop_msg);
    
    spinFor(1000ms);
    EXPECT_TRUE(arbitrated_received_);
    EXPECT_EQ(last_arbitrated_command_.command_data, "emergency_stop");
    
    // 4. Stop mission
    mission_msg.data = "stop";
    mission_pub_->publish(mission_msg);
    
    spinFor(1000ms);
    // System should handle mission stop gracefully
}

TEST_F(ROS2IntegrationTest, CommandArbitrationLogic) {
    resetFlags();
    
    // Test priority: Teleop > Autonomy > Policy
    
    // 1. Send policy command (lowest priority)
    auto policy_msg = system_controller::msg::SystemCommand();
    policy_msg.command_type = "policy_command";
    policy_msg.command_data = "policy_command_test";
    policy_msg.priority = 1;
    policy_pub_->publish(policy_msg);
    
    spinFor(500ms);
    EXPECT_TRUE(arbitrated_received_);
    EXPECT_EQ(last_arbitrated_command_.command_data, "policy_command_test");
    
    // 2. Send autonomy command (should override policy)
    resetFlags();
    auto autonomy_msg = std_msgs::msg::String();
    autonomy_msg.data = "autonomy_command_test";
    autonomy_pub_->publish(autonomy_msg);
    
    spinFor(500ms);
    EXPECT_TRUE(arbitrated_received_);
    EXPECT_EQ(last_arbitrated_command_.command_data, "autonomy_command_test");
    
    // 3. Send teleop command (should override autonomy)
    resetFlags();
    auto teleop_msg = std_msgs::msg::String();
    teleop_msg.data = "teleop_command_test";
    teleop_pub_->publish(teleop_msg);
    
    spinFor(500ms);
    EXPECT_TRUE(arbitrated_received_);
    EXPECT_EQ(last_arbitrated_command_.command_data, "teleop_command_test");
}

TEST_F(ROS2IntegrationTest, MessageTimestamps) {
    // Test that timestamps are properly set
    auto msg = system_controller::msg::SystemCommand();
    msg.header.stamp = test_node_->get_clock()->now();
    msg.command_type = "test";
    msg.command_data = "timestamp_test";
    msg.priority = 1;
    msg.timestamp = test_node_->get_clock()->now().seconds();
    
    policy_pub_->publish(msg);
    
    spinFor(1000ms);
    
    EXPECT_TRUE(arbitrated_received_);
    EXPECT_GT(last_arbitrated_command_.timestamp, 0.0);
    
    if (vehicle_status_received_) {
        EXPECT_GT(last_vehicle_status_.header.stamp.sec, 0);
    }
    
    if (mission_status_received_) {
        EXPECT_GT(last_mission_status_.header.stamp.sec, 0);
    }
}

} // namespace test
} // namespace system_controller

// Main function for running ROS 2 integration tests
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 