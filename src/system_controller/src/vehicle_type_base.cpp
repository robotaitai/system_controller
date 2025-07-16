#include "../include/vehicle_type_base.hpp"
#include <sstream>
#include <iostream>

namespace system_controller {

// VehicleType1 Implementation
void VehicleType1::initialize() {
    initialized_ = true;
    std::cout << "[VehicleType1] Initializing ground vehicle systems..." << std::endl;
}

void VehicleType1::processCommand(const std::string& command) {
    std::cout << "[VehicleType1] Processing command: " << command << std::endl;
    
    if (command.find("move_forward") != std::string::npos) {
        std::cout << "[VehicleType1] Engaging ground propulsion..." << std::endl;
    } else if (command.find("turn") != std::string::npos) {
        std::cout << "[VehicleType1] Adjusting steering..." << std::endl;
    } else {
        std::cout << "[VehicleType1] Unknown command for ground vehicle" << std::endl;
    }
}

std::string VehicleType1::getStatus() {
    return "Ground Vehicle: OPERATIONAL, Wheels: 4/4, Battery: 85%";
}

// VehicleType2 Implementation  
void VehicleType2::initialize() {
    initialized_ = true;
    std::cout << "[VehicleType2] Initializing aerial vehicle systems..." << std::endl;
}

void VehicleType2::processCommand(const std::string& command) {
    std::cout << "[VehicleType2] Processing command: " << command << std::endl;
    
    if (command.find("hover") != std::string::npos) {
        std::cout << "[VehicleType2] Maintaining hover position..." << std::endl;
    } else if (command.find("move_to_waypoint") != std::string::npos) {
        std::cout << "[VehicleType2] Flying to waypoint..." << std::endl;
    } else if (command.find("move_forward") != std::string::npos) {
        std::cout << "[VehicleType2] Engaging forward flight..." << std::endl;
    } else {
        std::cout << "[VehicleType2] Executing aerial maneuver..." << std::endl;
    }
}

std::string VehicleType2::getStatus() {
    return "Aerial Vehicle: AIRBORNE, Rotors: 4/4, Altitude: 15m, Battery: 72%";
}

} // namespace system_controller 