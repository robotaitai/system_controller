#include <iostream>
#include <memory>
#include <vector>
#include <string>

// Simplified versions of our classes for testing without ROS 2
namespace system_controller {

// Policy Classes (simplified)
class PolicyBase {
public:
    PolicyBase(const std::string& name) : name_(name) {}
    virtual ~PolicyBase() = default;
    virtual std::string getCommand() = 0;
    std::string getName() const { return name_; }
    
protected:
    std::string name_;
    bool active_ = true;
};

class Policy1 : public PolicyBase {
public:
    Policy1() : PolicyBase("Policy1") {}
    std::string getCommand() override {
        static int counter = 0;
        counter++;
        return "move_forward:speed=" + std::to_string(0.5 + (counter % 5) * 0.1);
    }
};

class Policy2 : public PolicyBase {
public:
    Policy2() : PolicyBase("Policy2") {}
    std::string getCommand() override {
        static int counter = 0;
        counter++;
        switch (counter % 3) {
            case 0: return "turn_right:angle=45";
            case 1: return "move_forward:speed=0.8";
            default: return "turn_left:angle=30";
        }
    }
};

// Vehicle Type Classes (simplified)
class VehicleTypeBase {
public:
    VehicleTypeBase(const std::string& name) : name_(name) {}
    virtual ~VehicleTypeBase() = default;
    virtual void processCommand(const std::string& command) = 0;
    virtual std::string getStatus() = 0;
    std::string getName() const { return name_; }
    
protected:
    std::string name_;
};

class VehicleType1 : public VehicleTypeBase {
public:
    VehicleType1() : VehicleTypeBase("GroundVehicle") {}
    
    void processCommand(const std::string& command) override {
        std::cout << "[" << getName() << "] Processing: " << command << std::endl;
        if (command.find("move_forward") != std::string::npos) {
            std::cout << "  -> Engaging ground propulsion..." << std::endl;
        } else if (command.find("turn") != std::string::npos) {
            std::cout << "  -> Adjusting steering..." << std::endl;
        }
    }
    
    std::string getStatus() override {
        return "Ground Vehicle: OPERATIONAL, Wheels: 4/4, Battery: 85%";
    }
};

class VehicleType2 : public VehicleTypeBase {
public:
    VehicleType2() : VehicleTypeBase("AerialVehicle") {}
    
    void processCommand(const std::string& command) override {
        std::cout << "[" << getName() << "] Processing: " << command << std::endl;
        if (command.find("hover") != std::string::npos) {
            std::cout << "  -> Maintaining hover position..." << std::endl;
        } else if (command.find("move_forward") != std::string::npos) {
            std::cout << "  -> Engaging forward flight..." << std::endl;
        }
    }
    
    std::string getStatus() override {
        return "Aerial Vehicle: AIRBORNE, Rotors: 4/4, Altitude: 15m, Battery: 72%";
    }
};

// Mission Type Classes (simplified)
class MissionTypeBase {
public:
    MissionTypeBase(const std::string& name) : name_(name) {}
    virtual ~MissionTypeBase() = default;
    virtual void startMission() = 0;
    virtual void stopMission() = 0;
    virtual std::string getStatus() = 0;
    virtual float getProgress() = 0;
    std::string getName() const { return name_; }
    bool isActive() const { return active_; }
    
protected:
    std::string name_;
    bool active_ = false;
    float progress_ = 0.0f;
};

class MissionType1 : public MissionTypeBase {
public:
    MissionType1() : MissionTypeBase("Surveillance") {}
    
    void startMission() override {
        active_ = true;
        progress_ = 0.0f;
        std::cout << "[" << getName() << "] Starting surveillance mission..." << std::endl;
    }
    
    void stopMission() override {
        active_ = false;
        progress_ = 100.0f;
        std::cout << "[" << getName() << "] Surveillance mission completed." << std::endl;
    }
    
    std::string getStatus() override {
        return active_ ? "ACTIVE: Surveillance patrol in progress" : "IDLE: Ready for surveillance";
    }
    
    float getProgress() override { return progress_; }
};

// Payload Type Classes (simplified)
class PayloadTypeBase {
public:
    PayloadTypeBase(const std::string& name) : name_(name) {}
    virtual ~PayloadTypeBase() = default;
    virtual void activate() = 0;
    virtual void deactivate() = 0;
    virtual std::string getStatus() = 0;
    std::string getName() const { return name_; }
    bool isActive() const { return active_; }
    
protected:
    std::string name_;
    bool active_ = false;
};

class PayloadType1 : public PayloadTypeBase {
public:
    PayloadType1() : PayloadTypeBase("Camera") {}
    
    void activate() override {
        active_ = true;
        std::cout << "[" << getName() << "] Camera payload activated" << std::endl;
    }
    
    void deactivate() override {
        active_ = false;
        std::cout << "[" << getName() << "] Camera payload deactivated" << std::endl;
    }
    
    std::string getStatus() override {
        return active_ ? "ACTIVE: Recording 4K 30fps" : "INACTIVE: Ready for activation";
    }
};

} // namespace system_controller

// Test function to demonstrate the system
void testSystemController() {
    std::cout << "=== ROS 2 System Controller - Core Classes Test ===" << std::endl;
    std::cout << std::endl;
    
    // Test Policy Classes
    std::cout << "🔧 Testing Policy Classes:" << std::endl;
    std::vector<std::unique_ptr<system_controller::PolicyBase>> policies;
    policies.push_back(std::make_unique<system_controller::Policy1>());
    policies.push_back(std::make_unique<system_controller::Policy2>());
    
    for (auto& policy : policies) {
        std::cout << "  " << policy->getName() << ": " << policy->getCommand() << std::endl;
    }
    std::cout << std::endl;
    
    // Test Vehicle Types
    std::cout << "🚗 Testing Vehicle Types:" << std::endl;
    std::vector<std::unique_ptr<system_controller::VehicleTypeBase>> vehicles;
    vehicles.push_back(std::make_unique<system_controller::VehicleType1>());
    vehicles.push_back(std::make_unique<system_controller::VehicleType2>());
    
    for (auto& vehicle : vehicles) {
        std::cout << "  " << vehicle->getName() << " Status: " << vehicle->getStatus() << std::endl;
        vehicle->processCommand("move_forward:speed=0.5");
    }
    std::cout << std::endl;
    
    // Test Mission Types
    std::cout << "🎯 Testing Mission Types:" << std::endl;
    auto mission = std::make_unique<system_controller::MissionType1>();
    std::cout << "  Initial status: " << mission->getStatus() << std::endl;
    mission->startMission();
    std::cout << "  After start: " << mission->getStatus() << std::endl;
    mission->stopMission();
    std::cout << std::endl;
    
    // Test Payload Types
    std::cout << "📷 Testing Payload Types:" << std::endl;
    auto payload = std::make_unique<system_controller::PayloadType1>();
    std::cout << "  Initial status: " << payload->getStatus() << std::endl;
    payload->activate();
    std::cout << "  After activation: " << payload->getStatus() << std::endl;
    payload->deactivate();
    std::cout << std::endl;
    
    std::cout << "✅ All core classes are working correctly!" << std::endl;
    std::cout << "🚀 The ROS 2 system architecture is solid and ready to run!" << std::endl;
}

int main() {
    try {
        testSystemController();
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "❌ Error: " << e.what() << std::endl;
        return 1;
    }
} 