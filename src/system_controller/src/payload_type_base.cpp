#include "../include/payload_type_base.hpp"
#include <iostream>

namespace system_controller {

// PayloadType1 Implementation - Camera Payload
void PayloadType1::activate() {
    active_ = true;
    std::cout << "[PayloadType1] Camera payload activated - starting image capture" << std::endl;
}

void PayloadType1::deactivate() {
    active_ = false;
    std::cout << "[PayloadType1] Camera payload deactivated - stopping image capture" << std::endl;
}

std::string PayloadType1::getStatus() {
    if (active_) {
        return "ACTIVE: Camera recording at 4K 30fps, Storage: 78% remaining";
    } else {
        return "INACTIVE: Camera ready for activation";
    }
}

void PayloadType1::processCommand(const std::string& command) {
    std::cout << "[PayloadType1] Processing camera command: " << command << std::endl;
    
    if (command.find("zoom") != std::string::npos) {
        std::cout << "[PayloadType1] Adjusting camera zoom..." << std::endl;
    } else if (command.find("focus") != std::string::npos) {
        std::cout << "[PayloadType1] Adjusting camera focus..." << std::endl;
    } else if (command.find("capture") != std::string::npos) {
        std::cout << "[PayloadType1] Taking high-resolution photo..." << std::endl;
    } else {
        std::cout << "[PayloadType1] Unknown camera command" << std::endl;
    }
}

// PayloadType2 Implementation - Sensor Package
void PayloadType2::activate() {
    active_ = true;
    std::cout << "[PayloadType2] Sensor package activated - starting data collection" << std::endl;
}

void PayloadType2::deactivate() {
    active_ = false;
    std::cout << "[PayloadType2] Sensor package deactivated - stopping data collection" << std::endl;
}

std::string PayloadType2::getStatus() {
    if (active_) {
        return "ACTIVE: Collecting environmental data, Temperature: 22.5°C, Humidity: 65%";
    } else {
        return "INACTIVE: Sensors ready for activation";
    }
}

void PayloadType2::processCommand(const std::string& command) {
    std::cout << "[PayloadType2] Processing sensor command: " << command << std::endl;
    
    if (command.find("calibrate") != std::string::npos) {
        std::cout << "[PayloadType2] Calibrating sensors..." << std::endl;
    } else if (command.find("sample") != std::string::npos) {
        std::cout << "[PayloadType2] Taking environmental sample..." << std::endl;
    } else if (command.find("scan") != std::string::npos) {
        std::cout << "[PayloadType2] Performing area scan..." << std::endl;
    } else {
        std::cout << "[PayloadType2] Unknown sensor command" << std::endl;
    }
}

} // namespace system_controller 