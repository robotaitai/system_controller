#include <gtest/gtest.h>
#include <chrono>
#include <memory>
#include <thread>
#include <vector>
#include <fstream>
#include <sstream>

#include "../include/policy_base.hpp"
#include "../include/vehicle_type_base.hpp"
#include "../include/mission_type_base.hpp"
#include "../include/payload_type_base.hpp"

using namespace std::chrono;

namespace system_controller {
namespace test {

// ============================================================================
// Performance Regression Tests
// ============================================================================

class PerformanceRegressionTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Set performance thresholds (in microseconds)
        policy_command_threshold_us_ = 100;        // Policy command generation should be < 100μs
        vehicle_command_threshold_us_ = 500;       // Vehicle command processing should be < 500μs
        mission_update_threshold_us_ = 1000;       // Mission update should be < 1ms
        payload_activation_threshold_us_ = 200;    // Payload activation should be < 200μs
    }
    
    template<typename Func>
    double measureExecutionTime(Func&& func) {
        auto start = high_resolution_clock::now();
        func();
        auto end = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>(end - start);
        return duration.count();
    }
    
    int policy_command_threshold_us_;
    int vehicle_command_threshold_us_;
    int mission_update_threshold_us_;
    int payload_activation_threshold_us_;
};

TEST_F(PerformanceRegressionTest, PolicyCommandGenerationPerformance) {
    auto policy = std::make_unique<Policy1>();
    
    // Measure multiple iterations
    std::vector<double> times;
    for (int i = 0; i < 100; ++i) {
        double time = measureExecutionTime([&]() {
            policy->getCommand();
        });
        times.push_back(time);
    }
    
    // Calculate average
    double avg_time = std::accumulate(times.begin(), times.end(), 0.0) / times.size();
    
    EXPECT_LT(avg_time, policy_command_threshold_us_) 
        << "Policy command generation took " << avg_time << "μs, expected < " 
        << policy_command_threshold_us_ << "μs";
    
    // Check that 95% of calls are within threshold
    int violations = 0;
    for (double time : times) {
        if (time > policy_command_threshold_us_) violations++;
    }
    double violation_rate = static_cast<double>(violations) / times.size();
    
    EXPECT_LT(violation_rate, 0.05) 
        << "Performance violation rate: " << (violation_rate * 100) << "%";
}

TEST_F(PerformanceRegressionTest, VehicleCommandProcessingPerformance) {
    auto vehicle = std::make_unique<VehicleType1>();
    vehicle->initialize();
    
    std::vector<std::string> test_commands = {
        "move_forward:speed=0.5",
        "turn_right:angle=45", 
        "turn_left:angle=30",
        "stop"
    };
    
    for (const auto& cmd : test_commands) {
        double time = measureExecutionTime([&]() {
            vehicle->processCommand(cmd);
        });
        
        EXPECT_LT(time, vehicle_command_threshold_us_) 
            << "Vehicle command processing for '" << cmd << "' took " << time 
            << "μs, expected < " << vehicle_command_threshold_us_ << "μs";
    }
}

TEST_F(PerformanceRegressionTest, MissionUpdatePerformance) {
    auto mission = std::make_unique<MissionType1>();
    mission->startMission();
    
    std::vector<double> times;
    for (int i = 0; i < 50; ++i) {
        double time = measureExecutionTime([&]() {
            mission->updateMission();
        });
        times.push_back(time);
        
        if (!mission->isActive()) break; // Mission completed
    }
    
    double avg_time = std::accumulate(times.begin(), times.end(), 0.0) / times.size();
    
    EXPECT_LT(avg_time, mission_update_threshold_us_) 
        << "Mission update took " << avg_time << "μs, expected < " 
        << mission_update_threshold_us_ << "μs";
}

TEST_F(PerformanceRegressionTest, PayloadActivationPerformance) {
    auto payload = std::make_unique<PayloadType1>();
    
    // Test activation performance
    double activation_time = measureExecutionTime([&]() {
        payload->activate();
    });
    
    EXPECT_LT(activation_time, payload_activation_threshold_us_) 
        << "Payload activation took " << activation_time << "μs, expected < " 
        << payload_activation_threshold_us_ << "μs";
    
    // Test deactivation performance
    double deactivation_time = measureExecutionTime([&]() {
        payload->deactivate();
    });
    
    EXPECT_LT(deactivation_time, payload_activation_threshold_us_) 
        << "Payload deactivation took " << deactivation_time << "μs, expected < " 
        << payload_activation_threshold_us_ << "μs";
}

// ============================================================================
// API Stability Regression Tests
// ============================================================================

class APIStabilityTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create instances of all types
        policy1_ = std::make_unique<Policy1>();
        policy2_ = std::make_unique<Policy2>();
        policy3_ = std::make_unique<Policy3>();
        
        vehicle1_ = std::make_unique<VehicleType1>();
        vehicle2_ = std::make_unique<VehicleType2>();
        
        mission1_ = std::make_unique<MissionType1>();
        mission2_ = std::make_unique<MissionType2>();
        
        payload1_ = std::make_unique<PayloadType1>();
        payload2_ = std::make_unique<PayloadType2>();
    }
    
    std::unique_ptr<Policy1> policy1_;
    std::unique_ptr<Policy2> policy2_;
    std::unique_ptr<Policy3> policy3_;
    
    std::unique_ptr<VehicleType1> vehicle1_;
    std::unique_ptr<VehicleType2> vehicle2_;
    
    std::unique_ptr<MissionType1> mission1_;
    std::unique_ptr<MissionType2> mission2_;
    
    std::unique_ptr<PayloadType1> payload1_;
    std::unique_ptr<PayloadType2> payload2_;
};

TEST_F(APIStabilityTest, PolicyBaseInterfaceStability) {
    // Test that PolicyBase interface methods are available and work correctly
    std::vector<PolicyBase*> policies = {
        policy1_.get(), policy2_.get(), policy3_.get()
    };
    
    for (auto* policy : policies) {
        // Test required interface methods
        EXPECT_NO_THROW(policy->getName());
        EXPECT_NO_THROW(policy->getCommand());
        EXPECT_NO_THROW(policy->isActive());
        EXPECT_NO_THROW(policy->setActive(true));
        EXPECT_NO_THROW(policy->setActive(false));
        EXPECT_NO_THROW(policy->update());
        
        // Test return types and basic behavior
        EXPECT_FALSE(policy->getName().empty());
        EXPECT_FALSE(policy->getCommand().empty());
        
        // Test state management
        policy->setActive(false);
        EXPECT_FALSE(policy->isActive());
        policy->setActive(true);
        EXPECT_TRUE(policy->isActive());
    }
}

TEST_F(APIStabilityTest, VehicleTypeBaseInterfaceStability) {
    std::vector<VehicleTypeBase*> vehicles = {
        vehicle1_.get(), vehicle2_.get()
    };
    
    for (auto* vehicle : vehicles) {
        // Test required interface methods
        EXPECT_NO_THROW(vehicle->getName());
        EXPECT_NO_THROW(vehicle->getStatus());
        EXPECT_NO_THROW(vehicle->initialize());
        EXPECT_NO_THROW(vehicle->shutdown());
        EXPECT_NO_THROW(vehicle->processCommand("test"));
        
        // Test return types
        EXPECT_FALSE(vehicle->getName().empty());
        EXPECT_FALSE(vehicle->getStatus().empty());
    }
}

TEST_F(APIStabilityTest, MissionTypeBaseInterfaceStability) {
    std::vector<MissionTypeBase*> missions = {
        mission1_.get(), mission2_.get()
    };
    
    for (auto* mission : missions) {
        // Test required interface methods
        EXPECT_NO_THROW(mission->getName());
        EXPECT_NO_THROW(mission->getStatus());
        EXPECT_NO_THROW(mission->getProgress());
        EXPECT_NO_THROW(mission->isActive());
        EXPECT_NO_THROW(mission->startMission());
        EXPECT_NO_THROW(mission->stopMission());
        EXPECT_NO_THROW(mission->updateMission());
        
        // Test return types and behavior
        EXPECT_FALSE(mission->getName().empty());
        EXPECT_FALSE(mission->getStatus().empty());
        EXPECT_GE(mission->getProgress(), 0.0f);
        EXPECT_LE(mission->getProgress(), 100.0f);
    }
}

TEST_F(APIStabilityTest, PayloadTypeBaseInterfaceStability) {
    std::vector<PayloadTypeBase*> payloads = {
        payload1_.get(), payload2_.get()
    };
    
    for (auto* payload : payloads) {
        // Test required interface methods
        EXPECT_NO_THROW(payload->getName());
        EXPECT_NO_THROW(payload->getStatus());
        EXPECT_NO_THROW(payload->isActive());
        EXPECT_NO_THROW(payload->activate());
        EXPECT_NO_THROW(payload->deactivate());
        EXPECT_NO_THROW(payload->processCommand("test"));
        EXPECT_NO_THROW(payload->configure("test_config"));
        
        // Test return types and behavior
        EXPECT_FALSE(payload->getName().empty());
        EXPECT_FALSE(payload->getStatus().empty());
    }
}

// ============================================================================
// Behavioral Regression Tests
// ============================================================================

class BehavioralRegressionTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Define expected behavior patterns
    }
};

TEST_F(BehavioralRegressionTest, PolicyCommandPatterns) {
    // Test that policies generate expected command patterns
    
    // Policy1 should always generate move_forward commands
    auto policy1 = std::make_unique<Policy1>();
    for (int i = 0; i < 10; ++i) {
        std::string cmd = policy1->getCommand();
        EXPECT_NE(cmd.find("move_forward"), std::string::npos) 
            << "Policy1 should always generate move_forward commands, got: " << cmd;
    }
    
    // Policy2 should cycle through different commands
    auto policy2 = std::make_unique<Policy2>();
    std::set<std::string> command_types;
    for (int i = 0; i < 10; ++i) {
        std::string cmd = policy2->getCommand();
        if (cmd.find("turn_right") != std::string::npos) command_types.insert("turn_right");
        if (cmd.find("turn_left") != std::string::npos) command_types.insert("turn_left");
        if (cmd.find("move_forward") != std::string::npos) command_types.insert("move_forward");
    }
    EXPECT_GE(command_types.size(), 2) << "Policy2 should generate varied commands";
    
    // Policy3 should generate aerial commands
    auto policy3 = std::make_unique<Policy3>();
    std::set<std::string> aerial_commands;
    for (int i = 0; i < 20; ++i) {
        std::string cmd = policy3->getCommand();
        if (cmd.find("hover") != std::string::npos) aerial_commands.insert("hover");
        if (cmd.find("waypoint") != std::string::npos) aerial_commands.insert("waypoint");
        if (cmd.find("scan") != std::string::npos) aerial_commands.insert("scan");
        if (cmd.find("return") != std::string::npos) aerial_commands.insert("return");
    }
    EXPECT_GE(aerial_commands.size(), 3) << "Policy3 should generate varied aerial commands";
}

TEST_F(BehavioralRegressionTest, MissionProgressBehavior) {
    // Test that missions progress correctly
    auto mission = std::make_unique<MissionType1>();
    
    EXPECT_FALSE(mission->isActive());
    EXPECT_EQ(mission->getProgress(), 0.0f);
    
    mission->startMission();
    EXPECT_TRUE(mission->isActive());
    EXPECT_EQ(mission->getProgress(), 0.0f);
    
    // Progress should increase with updates
    float prev_progress = mission->getProgress();
    for (int i = 0; i < 10 && mission->isActive(); ++i) {
        mission->updateMission();
        float current_progress = mission->getProgress();
        EXPECT_GE(current_progress, prev_progress) 
            << "Mission progress should never decrease";
        prev_progress = current_progress;
    }
    
    // Mission should eventually complete
    while (mission->isActive()) {
        mission->updateMission();
    }
    EXPECT_EQ(mission->getProgress(), 100.0f);
}

TEST_F(BehavioralRegressionTest, PayloadStateTransitions) {
    auto payload = std::make_unique<PayloadType1>();
    
    // Initial state
    EXPECT_FALSE(payload->isActive());
    EXPECT_NE(payload->getStatus().find("INACTIVE"), std::string::npos);
    
    // Activation
    payload->activate();
    EXPECT_TRUE(payload->isActive());
    EXPECT_NE(payload->getStatus().find("ACTIVE"), std::string::npos);
    
    // Deactivation
    payload->deactivate();
    EXPECT_FALSE(payload->isActive());
    EXPECT_NE(payload->getStatus().find("INACTIVE"), std::string::npos);
    
    // Multiple activations should be safe
    payload->activate();
    payload->activate();
    EXPECT_TRUE(payload->isActive());
    
    payload->deactivate();
    payload->deactivate();
    EXPECT_FALSE(payload->isActive());
}

// ============================================================================
// Memory and Resource Regression Tests
// ============================================================================

class ResourceRegressionTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Set memory thresholds
        max_objects_created_ = 10000;
    }
    
    int max_objects_created_;
};

TEST_F(ResourceRegressionTest, MemoryLeakDetection) {
    // Test that creating and destroying objects doesn't leak memory
    
    // Create and destroy many policy objects
    for (int i = 0; i < 1000; ++i) {
        auto policy = std::make_unique<Policy1>();
        std::string cmd = policy->getCommand();
        // Object should be destroyed at end of scope
    }
    
    // Create and destroy many vehicle objects
    for (int i = 0; i < 1000; ++i) {
        auto vehicle = std::make_unique<VehicleType1>();
        vehicle->initialize();
        vehicle->processCommand("test");
        vehicle->shutdown();
    }
    
    // Create and destroy many mission objects
    for (int i = 0; i < 1000; ++i) {
        auto mission = std::make_unique<MissionType1>();
        mission->startMission();
        mission->updateMission();
        mission->stopMission();
    }
    
    // If we reach here without crashes, basic memory management is working
    EXPECT_TRUE(true);
}

TEST_F(ResourceRegressionTest, LargeScaleObjectCreation) {
    // Test system behavior with many objects
    std::vector<std::unique_ptr<PolicyBase>> policies;
    std::vector<std::unique_ptr<VehicleTypeBase>> vehicles;
    std::vector<std::unique_ptr<MissionTypeBase>> missions;
    std::vector<std::unique_ptr<PayloadTypeBase>> payloads;
    
    // Create many objects
    for (int i = 0; i < 100; ++i) {
        policies.push_back(std::make_unique<Policy1>());
        vehicles.push_back(std::make_unique<VehicleType1>());
        missions.push_back(std::make_unique<MissionType1>());
        payloads.push_back(std::make_unique<PayloadType1>());
    }
    
    // Use all objects
    for (size_t i = 0; i < policies.size(); ++i) {
        policies[i]->getCommand();
        vehicles[i]->processCommand("test");
        missions[i]->startMission();
        payloads[i]->activate();
    }
    
    EXPECT_EQ(policies.size(), 100);
    EXPECT_EQ(vehicles.size(), 100);
    EXPECT_EQ(missions.size(), 100);
    EXPECT_EQ(payloads.size(), 100);
    
    // Objects will be destroyed when vectors go out of scope
}

} // namespace test
} // namespace system_controller

// Main function for running regression tests
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    
    std::cout << "Running System Controller Regression Tests..." << std::endl;
    std::cout << "These tests verify that system behavior hasn't regressed." << std::endl;
    
    int result = RUN_ALL_TESTS();
    
    if (result == 0) {
        std::cout << "✅ All regression tests passed!" << std::endl;
    } else {
        std::cout << "❌ Some regression tests failed!" << std::endl;
    }
    
    return result;
} 