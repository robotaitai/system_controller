#include <gtest/gtest.h>
#include <memory>
#include "../include/policy_base.hpp"
#include "../include/vehicle_type_base.hpp"
#include "../include/mission_type_base.hpp"
#include "../include/payload_type_base.hpp"

namespace system_controller {
namespace test {

// ============================================================================
// Policy Classes Unit Tests
// ============================================================================

class PolicyTest : public ::testing::Test {
protected:
    void SetUp() override {
        policy1_ = std::make_unique<Policy1>();
        policy2_ = std::make_unique<Policy2>();
        policy3_ = std::make_unique<Policy3>();
    }

    std::unique_ptr<Policy1> policy1_;
    std::unique_ptr<Policy2> policy2_;
    std::unique_ptr<Policy3> policy3_;
};

TEST_F(PolicyTest, PolicyNamesAreCorrect) {
    EXPECT_EQ(policy1_->getName(), "Policy1");
    EXPECT_EQ(policy2_->getName(), "Policy2");
    EXPECT_EQ(policy3_->getName(), "Policy3");
}

TEST_F(PolicyTest, PoliciesAreActiveByDefault) {
    EXPECT_TRUE(policy1_->isActive());
    EXPECT_TRUE(policy2_->isActive());
    EXPECT_TRUE(policy3_->isActive());
}

TEST_F(PolicyTest, CanSetPolicyActiveState) {
    policy1_->setActive(false);
    EXPECT_FALSE(policy1_->isActive());
    
    policy1_->setActive(true);
    EXPECT_TRUE(policy1_->isActive());
}

TEST_F(PolicyTest, Policy1GeneratesValidCommands) {
    for (int i = 0; i < 10; ++i) {
        std::string cmd = policy1_->getCommand();
        EXPECT_FALSE(cmd.empty());
        EXPECT_NE(cmd.find("move_forward:speed="), std::string::npos);
    }
}

TEST_F(PolicyTest, Policy2GeneratesValidCommands) {
    std::vector<std::string> commands;
    for (int i = 0; i < 6; ++i) {  // Test full cycle
        commands.push_back(policy2_->getCommand());
    }
    
    // Should contain turn and move commands
    bool has_turn = false, has_move = false;
    for (const auto& cmd : commands) {
        if (cmd.find("turn") != std::string::npos) has_turn = true;
        if (cmd.find("move_forward") != std::string::npos) has_move = true;
    }
    EXPECT_TRUE(has_turn);
    EXPECT_TRUE(has_move);
}

TEST_F(PolicyTest, Policy3GeneratesValidCommands) {
    std::vector<std::string> commands;
    for (int i = 0; i < 8; ++i) {  // Test full cycle
        commands.push_back(policy3_->getCommand());
    }
    
    // Should contain hover, waypoint, scan, and return commands
    bool has_hover = false, has_waypoint = false, has_scan = false, has_return = false;
    for (const auto& cmd : commands) {
        if (cmd.find("hover") != std::string::npos) has_hover = true;
        if (cmd.find("move_to_waypoint") != std::string::npos) has_waypoint = true;
        if (cmd.find("scan_area") != std::string::npos) has_scan = true;
        if (cmd.find("return_to_base") != std::string::npos) has_return = true;
    }
    EXPECT_TRUE(has_hover);
    EXPECT_TRUE(has_waypoint);
    EXPECT_TRUE(has_scan);
    EXPECT_TRUE(has_return);
}

// ============================================================================
// Vehicle Type Classes Unit Tests
// ============================================================================

class VehicleTypeTest : public ::testing::Test {
protected:
    void SetUp() override {
        vehicle1_ = std::make_unique<VehicleType1>();
        vehicle2_ = std::make_unique<VehicleType2>();
    }

    std::unique_ptr<VehicleType1> vehicle1_;
    std::unique_ptr<VehicleType2> vehicle2_;
};

TEST_F(VehicleTypeTest, VehicleNamesAreCorrect) {
    EXPECT_EQ(vehicle1_->getName(), "VehicleType1");
    EXPECT_EQ(vehicle2_->getName(), "VehicleType2");
}

TEST_F(VehicleTypeTest, VehiclesProvideStatus) {
    std::string status1 = vehicle1_->getStatus();
    std::string status2 = vehicle2_->getStatus();
    
    EXPECT_FALSE(status1.empty());
    EXPECT_FALSE(status2.empty());
    
    // Ground vehicle should mention wheels/ground
    EXPECT_NE(status1.find("Ground"), std::string::npos);
    
    // Aerial vehicle should mention rotors/altitude
    EXPECT_NE(status2.find("Aerial"), std::string::npos);
}

TEST_F(VehicleTypeTest, VehiclesProcessCommands) {
    // Test that vehicles can process various commands without crashing
    std::vector<std::string> test_commands = {
        "move_forward:speed=0.5",
        "turn_right:angle=45",
        "hover:altitude=5.0",
        "unknown_command"
    };
    
    for (const auto& cmd : test_commands) {
        EXPECT_NO_THROW(vehicle1_->processCommand(cmd));
        EXPECT_NO_THROW(vehicle2_->processCommand(cmd));
    }
}

TEST_F(VehicleTypeTest, VehiclesCanBeInitialized) {
    EXPECT_NO_THROW(vehicle1_->initialize());
    EXPECT_NO_THROW(vehicle2_->initialize());
}

// ============================================================================
// Mission Type Classes Unit Tests
// ============================================================================

class MissionTypeTest : public ::testing::Test {
protected:
    void SetUp() override {
        mission1_ = std::make_unique<MissionType1>();
        mission2_ = std::make_unique<MissionType2>();
    }

    std::unique_ptr<MissionType1> mission1_;
    std::unique_ptr<MissionType2> mission2_;
};

TEST_F(MissionTypeTest, MissionNamesAreCorrect) {
    EXPECT_EQ(mission1_->getName(), "MissionType1");
    EXPECT_EQ(mission2_->getName(), "MissionType2");
}

TEST_F(MissionTypeTest, MissionsStartInactive) {
    EXPECT_FALSE(mission1_->isActive());
    EXPECT_FALSE(mission2_->isActive());
    EXPECT_EQ(mission1_->getProgress(), 0.0f);
    EXPECT_EQ(mission2_->getProgress(), 0.0f);
}

TEST_F(MissionTypeTest, MissionLifecycle) {
    // Test mission1 lifecycle
    mission1_->startMission();
    EXPECT_TRUE(mission1_->isActive());
    EXPECT_EQ(mission1_->getProgress(), 0.0f);
    
    mission1_->stopMission();
    EXPECT_FALSE(mission1_->isActive());
    EXPECT_EQ(mission1_->getProgress(), 100.0f);
    
    // Test mission2 lifecycle
    mission2_->startMission();
    EXPECT_TRUE(mission2_->isActive());
    EXPECT_EQ(mission2_->getProgress(), 0.0f);
    
    mission2_->stopMission();
    EXPECT_FALSE(mission2_->isActive());
    EXPECT_EQ(mission2_->getProgress(), 100.0f);
}

TEST_F(MissionTypeTest, MissionProgressUpdates) {
    mission1_->startMission();
    float initial_progress = mission1_->getProgress();
    
    // Update mission several times
    for (int i = 0; i < 5; ++i) {
        mission1_->updateMission();
    }
    
    float updated_progress = mission1_->getProgress();
    EXPECT_GE(updated_progress, initial_progress);
    
    // Mission should eventually complete
    while (mission1_->isActive()) {
        mission1_->updateMission();
    }
    EXPECT_EQ(mission1_->getProgress(), 100.0f);
}

TEST_F(MissionTypeTest, MissionStatusChanges) {
    std::string inactive_status = mission1_->getStatus();
    
    mission1_->startMission();
    std::string active_status = mission1_->getStatus();
    
    EXPECT_NE(inactive_status, active_status);
    EXPECT_NE(active_status.find("ACTIVE"), std::string::npos);
}

// ============================================================================
// Payload Type Classes Unit Tests
// ============================================================================

class PayloadTypeTest : public ::testing::Test {
protected:
    void SetUp() override {
        payload1_ = std::make_unique<PayloadType1>();
        payload2_ = std::make_unique<PayloadType2>();
    }

    std::unique_ptr<PayloadType1> payload1_;
    std::unique_ptr<PayloadType2> payload2_;
};

TEST_F(PayloadTypeTest, PayloadNamesAreCorrect) {
    EXPECT_EQ(payload1_->getName(), "PayloadType1");
    EXPECT_EQ(payload2_->getName(), "PayloadType2");
}

TEST_F(PayloadTypeTest, PayloadsStartInactive) {
    EXPECT_FALSE(payload1_->isActive());
    EXPECT_FALSE(payload2_->isActive());
}

TEST_F(PayloadTypeTest, PayloadLifecycle) {
    // Test payload1 lifecycle
    payload1_->activate();
    EXPECT_TRUE(payload1_->isActive());
    
    payload1_->deactivate();
    EXPECT_FALSE(payload1_->isActive());
    
    // Test payload2 lifecycle
    payload2_->activate();
    EXPECT_TRUE(payload2_->isActive());
    
    payload2_->deactivate();
    EXPECT_FALSE(payload2_->isActive());
}

TEST_F(PayloadTypeTest, PayloadStatusChanges) {
    std::string inactive_status1 = payload1_->getStatus();
    
    payload1_->activate();
    std::string active_status1 = payload1_->getStatus();
    
    EXPECT_NE(inactive_status1, active_status1);
    EXPECT_NE(active_status1.find("ACTIVE"), std::string::npos);
    
    payload1_->deactivate();
    std::string deactive_status1 = payload1_->getStatus();
    
    EXPECT_NE(active_status1, deactive_status1);
    EXPECT_NE(deactive_status1.find("INACTIVE"), std::string::npos);
}

TEST_F(PayloadTypeTest, PayloadsProcessCommands) {
    std::vector<std::string> test_commands = {
        "zoom:level=2.0",
        "focus:target=auto",
        "capture:resolution=4K",
        "calibrate:sensors=all",
        "unknown_command"
    };
    
    payload1_->activate();
    payload2_->activate();
    
    for (const auto& cmd : test_commands) {
        EXPECT_NO_THROW(payload1_->processCommand(cmd));
        EXPECT_NO_THROW(payload2_->processCommand(cmd));
    }
}

// ============================================================================
// Integration Tests
// ============================================================================

class IntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create instances of all types
        policy_ = std::make_unique<Policy1>();
        vehicle_ = std::make_unique<VehicleType1>();
        mission_ = std::make_unique<MissionType1>();
        payload_ = std::make_unique<PayloadType1>();
    }

    std::unique_ptr<PolicyBase> policy_;
    std::unique_ptr<VehicleTypeBase> vehicle_;
    std::unique_ptr<MissionTypeBase> mission_;
    std::unique_ptr<PayloadTypeBase> payload_;
};

TEST_F(IntegrationTest, PolicyToVehicleCommandFlow) {
    // Test the flow: Policy generates command -> Vehicle processes it
    std::string command = policy_->getCommand();
    EXPECT_FALSE(command.empty());
    
    // Vehicle should be able to process policy commands
    EXPECT_NO_THROW(vehicle_->processCommand(command));
    
    // Vehicle should provide status after processing
    std::string status = vehicle_->getStatus();
    EXPECT_FALSE(status.empty());
}

TEST_F(IntegrationTest, MissionToPayloadFlow) {
    // Test the flow: Mission starts -> Payload activates
    mission_->startMission();
    EXPECT_TRUE(mission_->isActive());
    
    // When mission is active, payload should be activatable
    payload_->activate();
    EXPECT_TRUE(payload_->isActive());
    
    // Mission progress should update
    mission_->updateMission();
    EXPECT_GT(mission_->getProgress(), 0.0f);
    
    // When mission stops, payload should deactivate
    mission_->stopMission();
    payload_->deactivate();
    EXPECT_FALSE(mission_->isActive());
    EXPECT_FALSE(payload_->isActive());
}

TEST_F(IntegrationTest, SystemStateConsistency) {
    // Test that all components can coexist and maintain consistent state
    
    // Start mission
    mission_->startMission();
    payload_->activate();
    
    // Generate and process commands
    for (int i = 0; i < 5; ++i) {
        std::string cmd = policy_->getCommand();
        vehicle_->processCommand(cmd);
        mission_->updateMission();
        
        // All components should remain in valid states
        EXPECT_TRUE(mission_->isActive() || mission_->getProgress() == 100.0f);
        EXPECT_TRUE(payload_->isActive());
        EXPECT_FALSE(vehicle_->getStatus().empty());
        EXPECT_FALSE(cmd.empty());
    }
}

} // namespace test
} // namespace system_controller

// Main function for running tests
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 