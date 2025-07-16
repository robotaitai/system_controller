#include "../../include/adapters/vehicle_adapter_base.hpp"
#include <cmath>
#include <algorithm>

namespace system_controller {

VehicleAdapterBase::VehicleAdapterBase(const std::string& adapter_name)
    : adapter_name_(adapter_name) {
    clearError();
}

bool VehicleAdapterBase::validateSteeringAngle(double angle) const {
    return std::abs(angle) <= config_.max_steering_angle;
}

bool VehicleAdapterBase::validateVelocity(double velocity) const {
    return std::abs(velocity) <= config_.max_velocity;
}

double VehicleAdapterBase::applySteeringScale(double raw_angle) const {
    return (raw_angle * config_.steering_scale) + config_.steering_offset;
}

double VehicleAdapterBase::applyVelocityScale(double raw_velocity) const {
    return (raw_velocity * config_.velocity_scale) + config_.velocity_offset;
}

void VehicleAdapterBase::setError(const std::string& error) {
    last_error_ = error;
    status_.last_error = error;
    
    // Call error callback if set
    if (error_callback_) {
        error_callback_(error);
    }
}

void VehicleAdapterBase::clearError() {
    last_error_.clear();
    status_.last_error.clear();
}

} // namespace system_controller 