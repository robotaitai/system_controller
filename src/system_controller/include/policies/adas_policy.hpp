#ifndef ADAS_POLICY_HPP
#define ADAS_POLICY_HPP

#include "policy_base_expanded.hpp"
#include <chrono>

namespace system_controller {

/**
 * @brief ADAS (Advanced Driver Assistance Systems) policy
 * 
 * This policy provides assisted driving capabilities with safety interventions.
 * It allows both manual and autonomous commands but applies safety constraints
 * and can override commands when safety conditions are violated.
 */
class AdasPolicy : public PolicyBaseExpanded {
public:
    AdasPolicy();
    ~AdasPolicy() override = default;

    // PolicyBaseExpanded interface implementation
    bool initialize(const std::map<std::string, std::string>& config) override;
    void update() override;
    PolicyStateInfo getState() const override;
    
    // Mode management - supports multiple modes with safety constraints
    bool requestModeChange(PolicyStateInfo::OperatingMode requested_mode) override;
    bool canTransitionTo(PolicyStateInfo::OperatingMode target_mode) const override;
    void processModeTransition() override;
    
    // Command filtering with safety assistance
    CommandFilterResult filterVehicleCommand(const VehicleCommand& command) override;
    CommandFilterResult filterImplementCommand(const ImplementCommand& command) override;
    bool isCommandSourceAllowed(const std::string& source) const override;
    
    // Safety and override management
    bool activateSafetyOverride(const std::string& reason) override;
    bool deactivateSafetyOverride() override;
    bool isEmergencyStopRequired() const override;
    void processEmergencyStop() override;
    
    // Mission-specific policies with safety constraints
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
    // Safety systems
    bool checkCollisionAvoidance();
    bool checkLaneKeeping();
    bool checkSpeedLimits();
    bool checkObstacleDetection();
    bool checkGeofencing();
    bool checkWeatherConditions();
    
    // Command modification for safety
    VehicleCommand applySafetyConstraints(const VehicleCommand& command);
    ImplementCommand applySafetyConstraints(const ImplementCommand& command);
    bool needsSpeedReduction(const VehicleCommand& command);
    bool needsSteeringCorrection(const VehicleCommand& command);
    
    // Tree detection and avoidance (for spray missions)
    bool detectTrees();
    bool isTreeInSprayPath();
    bool shouldStopSprayingForTree();
    
    // GPS error handling
    bool checkGPSAccuracy();
    bool isGPSErrorCritical();
    bool shouldStopForGPSError();
    
    // Configuration parameters
    bool enable_collision_avoidance_ = true;
    bool enable_lane_keeping_ = true;
    bool enable_speed_limiting_ = true;
    bool enable_geofencing_ = true;
    bool enable_tree_detection_ = true;
    
    double max_safe_speed_ = 15.0;        // km/h
    double collision_avoidance_distance_ = 5.0; // meters
    double lane_keeping_tolerance_ = 0.5;  // meters
    double gps_accuracy_threshold_ = 0.2;  // meters
    double tree_detection_distance_ = 10.0; // meters
    
    // Environmental limits
    double max_wind_speed_ = 20.0;        // km/h
    double min_visibility_ = 100.0;       // meters
    double max_slope_ = 15.0;             // degrees
    
    // Safety zones and restrictions
    std::vector<std::string> safety_zones_;
    std::vector<std::string> restricted_areas_;
    std::vector<std::string> time_restrictions_;
    
    // State tracking
    bool collision_detected_ = false;
    bool tree_detected_ = false;
    bool gps_error_active_ = false;
    bool weather_violation_ = false;
    bool geofence_violation_ = false;
    
    double current_speed_ = 0.0;
    double current_gps_accuracy_ = 0.0;
    double current_wind_speed_ = 0.0;
    double distance_to_obstacle_ = 1000.0;
    
    // Safety intervention tracking
    uint64_t speed_interventions_ = 0;
    uint64_t steering_interventions_ = 0;
    uint64_t emergency_stops_ = 0;
    uint64_t tree_stops_ = 0;
    uint64_t gps_stops_ = 0;
    
    // Timing
    std::chrono::steady_clock::time_point last_safety_check_;
    std::chrono::steady_clock::time_point last_tree_detection_;
    std::chrono::steady_clock::time_point last_gps_check_;
    
    // Constants
    static constexpr double SAFETY_CHECK_INTERVAL_SEC = 0.1;
    static constexpr double TREE_DETECTION_INTERVAL_SEC = 0.5;
    static constexpr double GPS_CHECK_INTERVAL_SEC = 1.0;
    static constexpr double EMERGENCY_DECELERATION = -5.0; // m/s^2
    static constexpr double MAX_STEERING_CORRECTION = 0.2; // rad
};

} // namespace system_controller

#endif // ADAS_POLICY_HPP 