#include "../include/mission_type_base.hpp"
#include <iostream>
#include <cstdlib>

namespace system_controller {

// MissionType1 Implementation - Surveillance Mission
void MissionType1::startMission() {
    active_ = true;
    progress_ = 0.0f;
    std::cout << "[MissionType1] Starting surveillance mission..." << std::endl;
}

void MissionType1::stopMission() {
    active_ = false;
    progress_ = 100.0f;
    std::cout << "[MissionType1] Surveillance mission completed." << std::endl;
}

std::string MissionType1::getStatus() {
    if (active_) {
        return "ACTIVE: Surveillance patrol in progress";
    } else {
        return "IDLE: Ready for surveillance mission";
    }
}

float MissionType1::getProgress() {
    return progress_;
}

void MissionType1::updateMission() {
    if (active_ && progress_ < 100.0f) {
        progress_ += 2.5f; // Increment progress
        if (progress_ >= 100.0f) {
            progress_ = 100.0f;
            stopMission();
        }
        std::cout << "[MissionType1] Surveillance progress: " << progress_ << "%" << std::endl;
    }
}

// MissionType2 Implementation - Delivery Mission
void MissionType2::startMission() {
    active_ = true;
    progress_ = 0.0f;
    std::cout << "[MissionType2] Starting delivery mission..." << std::endl;
}

void MissionType2::stopMission() {
    active_ = false;
    progress_ = 100.0f;
    std::cout << "[MissionType2] Delivery mission completed." << std::endl;
}

std::string MissionType2::getStatus() {
    if (active_) {
        return "ACTIVE: Package delivery in progress";
    } else {
        return "IDLE: Ready for delivery mission";
    }
}

float MissionType2::getProgress() {
    return progress_;
}

void MissionType2::updateMission() {
    if (active_ && progress_ < 100.0f) {
        progress_ += 5.0f; // Faster progress for delivery
        if (progress_ >= 100.0f) {
            progress_ = 100.0f;
            stopMission();
        }
        std::cout << "[MissionType2] Delivery progress: " << progress_ << "%" << std::endl;
    }
}

} // namespace system_controller 