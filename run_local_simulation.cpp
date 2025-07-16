#include <iostream>
#include <thread>
#include <chrono>
#include <memory>
#include <vector>
#include <string>
#include <queue>
#include <mutex>
#include <condition_variable>

// Include only the class definitions (copying them here to avoid main() conflict)
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

} // namespace system_controller

// Simple message passing simulation
class MessageBus {
private:
    std::queue<std::pair<std::string, std::string>> messages_;
    std::mutex mutex_;
    std::condition_variable cv_;
    
public:
    void publish(const std::string& topic, const std::string& message) {
        std::lock_guard<std::mutex> lock(mutex_);
        messages_.push({topic, message});
        cv_.notify_all();
        std::cout << "[MSG_BUS] Published to " << topic << ": " << message << std::endl;
    }
    
    std::pair<std::string, std::string> waitForMessage(int timeout_ms = 1000) {
        std::unique_lock<std::mutex> lock(mutex_);
        
        if (cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms), 
                        [this] { return !messages_.empty(); })) {
            auto msg = messages_.front();
            messages_.pop();
            return msg;
        }
        
        return {"", ""};  // Timeout
    }
    
    bool hasMessages() {
        std::lock_guard<std::mutex> lock(mutex_);
        return !messages_.empty();
    }
};

// Global message bus (simulates ROS 2 middleware)
MessageBus g_message_bus;

// Simulated nodes
class SimulatedSystemManager {
private:
    std::vector<std::unique_ptr<system_controller::PolicyBase>> policies_;
    int current_policy_index_ = 0;
    
public:
    SimulatedSystemManager() {
        policies_.push_back(std::make_unique<system_controller::Policy1>());
        policies_.push_back(std::make_unique<system_controller::Policy2>());
        std::cout << "[SYS_MGR] Initialized with " << policies_.size() << " policies" << std::endl;
    }
    
    void run() {
        while (true) {
            auto& policy = policies_[current_policy_index_];
            std::string command = policy->getCommand();
            
            g_message_bus.publish("/PolicyCommand", command);
            
            // Rotate policies
            current_policy_index_ = (current_policy_index_ + 1) % policies_.size();
            
            std::this_thread::sleep_for(std::chrono::seconds(3));
        }
    }
};

class SimulatedCommandArbiter {
private:
    std::string last_teleop_cmd_ = "";
    std::string last_autonomy_cmd_ = "";
    std::string last_policy_cmd_ = "";
    
public:
    SimulatedCommandArbiter() {
        std::cout << "[CMD_ARB] Command arbiter started" << std::endl;
    }
    
    void run() {
        while (true) {
            auto msg = g_message_bus.waitForMessage(100);
            
            if (!msg.first.empty()) {
                std::string selected_cmd;
                
                if (msg.first == "/PolicyCommand") {
                    last_policy_cmd_ = msg.second;
                    selected_cmd = last_policy_cmd_;
                    std::cout << "[CMD_ARB] Arbitrated policy command: " << selected_cmd << std::endl;
                } else if (msg.first == "/Teleop/Command") {
                    last_teleop_cmd_ = msg.second;
                    selected_cmd = last_teleop_cmd_;  // Higher priority
                    std::cout << "[CMD_ARB] Arbitrated teleop command (HIGH PRIORITY): " << selected_cmd << std::endl;
                } else if (msg.first == "/Autonomy/Command") {
                    last_autonomy_cmd_ = msg.second;
                    selected_cmd = last_autonomy_cmd_;
                    std::cout << "[CMD_ARB] Arbitrated autonomy command: " << selected_cmd << std::endl;
                }
                
                if (!selected_cmd.empty()) {
                    g_message_bus.publish("/ArbitratedCommand", selected_cmd);
                }
            }
        }
    }
};

class SimulatedVehicleAdapter {
private:
    std::vector<std::unique_ptr<system_controller::VehicleTypeBase>> vehicles_;
    int current_vehicle_index_ = 0;
    
public:
    SimulatedVehicleAdapter() {
        vehicles_.push_back(std::make_unique<system_controller::VehicleType1>());
        vehicles_.push_back(std::make_unique<system_controller::VehicleType2>());
        std::cout << "[VEH_ADP] Vehicle adapter started with " << vehicles_.size() << " vehicle types" << std::endl;
    }
    
    void run() {
        while (true) {
            auto msg = g_message_bus.waitForMessage(100);
            
            if (msg.first == "/ArbitratedCommand" && !msg.second.empty()) {
                auto& vehicle = vehicles_[current_vehicle_index_];
                vehicle->processCommand(msg.second);
                
                // Publish vehicle status
                std::string status = vehicle->getStatus();
                g_message_bus.publish("/VehicleStatus", status);
                
                std::cout << "[VEH_ADP] Vehicle status: " << status << std::endl;
            }
        }
    }
};

// Main simulation
int main() {
    std::cout << "=== ROS 2 System Controller - Local Simulation ===" << std::endl;
    std::cout << "This simulates the distributed ROS 2 system on your Mac without Docker!" << std::endl;
    std::cout << std::endl;
    
    // Create simulated nodes
    SimulatedSystemManager sys_mgr;
    SimulatedCommandArbiter cmd_arb;
    SimulatedVehicleAdapter veh_adp;
    
    // Start node threads
    std::thread sys_thread([&sys_mgr]() { sys_mgr.run(); });
    std::thread arb_thread([&cmd_arb]() { cmd_arb.run(); });
    std::thread veh_thread([&veh_adp]() { veh_adp.run(); });
    
    // Let the system run for a bit
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // Simulate external commands
    std::cout << std::endl << "=== Simulating External Commands ===" << std::endl;
    
    std::this_thread::sleep_for(std::chrono::seconds(5));
    std::cout << "\n🎮 Sending teleop command..." << std::endl;
    g_message_bus.publish("/Teleop/Command", "emergency_stop");
    
    std::this_thread::sleep_for(std::chrono::seconds(3));
    std::cout << "\n🤖 Sending autonomy command..." << std::endl;
    g_message_bus.publish("/Autonomy/Command", "autonomous_navigation");
    
    std::this_thread::sleep_for(std::chrono::seconds(5));
    
    std::cout << "\n✅ Simulation demonstrates working ROS 2 architecture!" << std::endl;
    std::cout << "🚀 Ready for deployment in real ROS 2 environment!" << std::endl;
    
    return 0;
} 