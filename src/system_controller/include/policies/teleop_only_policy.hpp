#ifndef TELEOP_ONLY_POLICY_HPP
#define TELEOP_ONLY_POLICY_HPP

#include "policy_base_expanded.hpp"
#include <chrono>

namespace system_controller {

/**
 * @brief Teleop-only policy that restricts operation to manual control
 * 
 * This policy enforces manual control only with basic safety checks.
 * It prevents autonomous operation and limits implement usage to
 * manual override conditions.
 */
class TeleopOnlyPolicy : public PolicyBaseExpanded {
public:
    TeleopOnlyPolicy();
    ~TeleopOnlyPolicy() override = default;

    // PolicyBaseExpanded interface implementation
    bool initialize(const std::map<std::string, std::string>& config) override;
    void update() override;
    PolicyStateInfo getState() const override;
    
    // Mode management - only allows TELEOP_ONLY mode
    bool requestModeChange(PolicyStateInfo::OperatingMode requested_mode) override;
    bool canTransitionTo(PolicyStateInfo::OperatingMode target_mode) const override;
    void processModeTransition() override;
    
    // Command filtering - allows teleop commands, blocks autonomous
    CommandFilterResult filterVehicleCommand(const VehicleCommand& command) override;
    CommandFilterResult filterImplementCommand(const ImplementCommand& command) override;
    bool isCommandSourceAllowed(const std::string& source) const override;
    
    // Safety and override management
    bool activateSafetyOverride(const std::string& reason) override;
    bool deactivateSafetyOverride() override;
    bool isEmergencyStopRequired() const override;
    void processEmergencyStop() override;
    
    // Mission-specific policies - limited missions allowed
    bool isMissionAllowed(const std::string& mission_type) const override;
    bool isAreaAllowed(const std::string& area_id) const override;
    bool isTimeAllowed() const override;
    std::vector<std::string> getActiveConstraints() const override;
    
    bool configure(const std::string& param_name, const std::string& param_value) override;
    std::string getParameter(const std::string& param_name) const override;
    std::map<std::string, std::string> getAllParameters() const override;
    
    bool validateConfiguration() const override;
    std::vector<std::string> getCurrentViolations() const override;
    std::map<std::string, double> getDiagnostics() const override;

private:
    // Safety monitoring
    bool checkOperatorPresence();
    bool checkDeadManSwitch();
    bool validateTeleopCommand(const VehicleCommand& command);
    bool validateTeleopImplementCommand(const ImplementCommand& command);
    
    // Configuration parameters
    bool require_operator_presence_ = true;
    bool require_deadman_switch_ = true;
    double max_teleop_velocity_ = 5.0;  // m/s
    double max_teleop_steering_rate_ = 1.0; // rad/s
    bool allow_implement_teleop_ = true;
    std::vector<std::string> allowed_mission_types_;
    
    // State tracking
    bool operator_present_ = false;
    bool deadman_switch_active_ = false;
    bool safety_override_active_ = false;
    std::string safety_override_reason_;
    std::chrono::steady_clock::time_point last_teleop_command_time_;
    
    // Statistics
    uint64_t teleop_commands_processed_ = 0;
    uint64_t autonomous_commands_blocked_ = 0;
    uint64_t safety_violations_ = 0;
    
    // Constants
    static constexpr double TELEOP_COMMAND_TIMEOUT_SEC = 5.0;
    static constexpr double MAX_SAFE_VELOCITY = 10.0; // m/s
    static constexpr double MAX_SAFE_STEERING_ANGLE = 1.57; // rad
};

} // namespace system_controller

#endif // TELEOP_ONLY_POLICY_HPP 