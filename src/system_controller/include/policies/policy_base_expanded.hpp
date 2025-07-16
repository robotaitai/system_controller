#ifndef POLICY_BASE_EXPANDED_HPP
#define POLICY_BASE_EXPANDED_HPP

#include <string>
#include <memory>
#include <map>
#include <vector>
#include <functional>

namespace system_controller {

// Forward declarations
struct VehicleCommand;
struct ImplementCommand;

/**
 * @brief Policy state information (matches PolicyState message)
 */
struct PolicyStateInfo {
    enum OperatingMode {
        TELEOP_ONLY = 0,
        AUTONOMOUS = 1,
        SEMI_AUTONOMOUS = 2,
        ADAS_ASSISTED = 3,
        SAFETY_OVERRIDE = 4,
        MAINTENANCE = 5
    };
    
    OperatingMode current_mode = TELEOP_ONLY;
    OperatingMode requested_mode = TELEOP_ONLY;
    bool mode_transition_active = false;
    
    // Policy flags
    bool teleop_allowed = true;
    bool autonomy_allowed = false;
    bool safety_override_active = false;
    bool emergency_stop_active = false;
    
    // Input source priorities (0-100, higher = more priority)
    uint8_t teleop_priority = 100;
    uint8_t autonomy_priority = 50;
    uint8_t safety_priority = 255;
    uint8_t mission_priority = 75;
    
    // Safety conditions
    bool gps_valid = false;
    bool sensor_health_ok = true;
    bool vehicle_health_ok = true;
    bool implement_health_ok = true;
    double safety_zone_distance = 1000.0; // meters
    
    // Mission/implement specific policies
    bool spray_allowed = false;
    bool mowing_allowed = false;
    bool seeding_allowed = false;
    std::vector<std::string> restricted_zones;
    std::vector<std::string> active_constraints;
    
    // Policy metadata
    std::string policy_id;
    std::string policy_version;
    uint64_t last_update_time = 0;
    std::vector<std::string> policy_violations;
};

/**
 * @brief Command filtering result
 */
struct CommandFilterResult {
    bool allowed = false;
    std::string reason;
    double priority_override = -1.0; // -1 means no override
    VehicleCommand* modified_vehicle_command = nullptr;
    ImplementCommand* modified_implement_command = nullptr;
};

/**
 * @brief Base class for all policy implementations
 * 
 * This class provides the plugin architecture for policy management.
 * Policies control operating modes, command filtering, safety overrides,
 * and mission-specific constraints.
 */
class PolicyBaseExpanded {
public:
    PolicyBaseExpanded(const std::string& policy_name, const std::string& version = "1.0");
    virtual ~PolicyBaseExpanded() = default;

    // Core policy interface
    virtual bool initialize(const std::map<std::string, std::string>& config) = 0;
    virtual void update() = 0; // Called periodically to update policy state
    virtual PolicyStateInfo getState() const = 0;
    
    // Mode management
    virtual bool requestModeChange(PolicyStateInfo::OperatingMode requested_mode) = 0;
    virtual bool canTransitionTo(PolicyStateInfo::OperatingMode target_mode) const = 0;
    virtual void processModeTransition() = 0;
    
    // Command filtering
    virtual CommandFilterResult filterVehicleCommand(const VehicleCommand& command) = 0;
    virtual CommandFilterResult filterImplementCommand(const ImplementCommand& command) = 0;
    virtual bool isCommandSourceAllowed(const std::string& source) const = 0;
    
    // Safety and override management
    virtual bool activateSafetyOverride(const std::string& reason) = 0;
    virtual bool deactivateSafetyOverride() = 0;
    virtual bool isEmergencyStopRequired() const = 0;
    virtual void processEmergencyStop() = 0;
    
    // Mission-specific policies
    virtual bool isMissionAllowed(const std::string& mission_type) const = 0;
    virtual bool isAreaAllowed(const std::string& area_id) const = 0;
    virtual bool isTimeAllowed() const = 0;
    virtual std::vector<std::string> getActiveConstraints() const = 0;
    
    // Configuration management
    virtual bool configure(const std::string& param_name, const std::string& param_value) = 0;
    virtual std::string getParameter(const std::string& param_name) const = 0;
    virtual std::map<std::string, std::string> getAllParameters() const = 0;
    
    // Validation and diagnostics
    virtual bool validateConfiguration() const = 0;
    virtual std::vector<std::string> getCurrentViolations() const = 0;
    virtual std::map<std::string, double> getDiagnostics() const = 0;
    
    // Getters
    std::string getPolicyName() const { return policy_name_; }
    std::string getPolicyVersion() const { return policy_version_; }
    std::string getPolicyId() const { return policy_id_; }

protected:
    // Helper methods for derived classes
    void setState(PolicyStateInfo::OperatingMode mode);
    void setFlag(const std::string& flag_name, bool value);
    void addViolation(const std::string& violation);
    void clearViolations();
    void addConstraint(const std::string& constraint);
    void removeConstraint(const std::string& constraint);
    
    // Member variables
    std::string policy_name_;
    std::string policy_version_;
    std::string policy_id_;
    PolicyStateInfo state_;
    std::map<std::string, std::string> config_params_;
    
    // Callbacks for policy events
    std::function<void(PolicyStateInfo::OperatingMode)> mode_change_callback_;
    std::function<void(const std::string&)> violation_callback_;
    std::function<void(const std::string&)> error_callback_;

public:
    // Callback setters
    void setModeChangeCallback(std::function<void(PolicyStateInfo::OperatingMode)> callback) {
        mode_change_callback_ = callback;
    }
    
    void setViolationCallback(std::function<void(const std::string&)> callback) {
        violation_callback_ = callback;
    }
    
    void setErrorCallback(std::function<void(const std::string&)> callback) {
        error_callback_ = callback;
    }
};

/**
 * @brief Policy factory for creating policy instances
 */
class PolicyFactory {
public:
    using PolicyCreator = std::function<std::shared_ptr<PolicyBaseExpanded>()>;
    
    static PolicyFactory& getInstance();
    
    bool registerPolicy(const std::string& policy_type, PolicyCreator creator);
    std::shared_ptr<PolicyBaseExpanded> createPolicy(const std::string& policy_type);
    std::vector<std::string> getAvailablePolicyTypes() const;

private:
    std::map<std::string, PolicyCreator> policy_creators_;
};

// Macro for easy policy registration
#define REGISTER_POLICY(PolicyClass, policy_type) \
    static bool policy_registered_##PolicyClass = \
        PolicyFactory::getInstance().registerPolicy(policy_type, \
            []() -> std::shared_ptr<PolicyBaseExpanded> { \
                return std::make_shared<PolicyClass>(); \
            });

} // namespace system_controller

#endif // POLICY_BASE_EXPANDED_HPP 