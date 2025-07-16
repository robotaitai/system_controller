#ifndef SPRAYER_MISSION_HPP
#define SPRAYER_MISSION_HPP

#include "mission_base.hpp"
#include <chrono>

namespace system_controller {

/**
 * @brief Sprayer-specific configuration parameters
 */
struct SprayerMissionConfig {
    std::string chemical_type = "herbicide";  // herbicide, pesticide, fungicide
    double application_rate = 200.0;          // L/ha application rate
    double spray_pressure = 3.0;              // bar
    double boom_height = 0.5;                 // meters above crop
    double max_wind_speed = 15.0;             // km/h maximum wind speed
    double min_temperature = 5.0;             // °C minimum temperature
    double max_temperature = 35.0;            // °C maximum temperature
    double buffer_zone_distance = 10.0;       // meters from sensitive areas
    bool gps_required = true;                 // GPS accuracy required
    double gps_accuracy_threshold = 0.1;      // meters
    std::vector<std::string> restricted_areas; // Areas to avoid spraying
};

/**
 * @brief Sprayer mission for herbicide and pesticide application
 * 
 * This mission implements the business logic for spraying operations including:
 * - Chemical application rate control
 * - Weather condition monitoring
 * - Buffer zone management
 * - GPS accuracy requirements
 * - Turn sequence optimization (lift at end of row, lower at start)
 */
class SprayerMission : public MissionBase {
public:
    SprayerMission();
    ~SprayerMission() override = default;

    // MissionBase interface implementation
    bool initialize(const MissionConfig& config) override;
    bool start() override;
    bool pause() override;
    bool resume() override;
    bool stop() override;
    bool abort() override;
    
    void update() override;
    MissionStatus getStatus() const override;
    double getProgress() const override;
    
    // Row management with sprayer-specific logic
    bool startNewRow() override;
    bool endCurrentRow() override;
    bool performTurn() override;
    
    // Implement management
    bool selectImplement(const std::string& implement_id) override;
    bool activateImplement() override;
    bool deactivateImplement() override;
    std::vector<std::string> getRequiredImplements() const override;
    
    bool configure(const std::string& param_name, const std::string& param_value) override;
    std::string getParameter(const std::string& param_name) const override;
    std::map<std::string, std::string> getAllParameters() const override;
    
    bool validateMissionConfig(const MissionConfig& config) const override;
    bool isReadyToStart() const override;
    bool performSafetyCheck() override;

protected:
    // Business logic hooks
    void onMissionStart() override;
    void onMissionComplete() override;
    void onRowStart() override;
    void onRowEnd() override;
    void onTurnStart() override;
    void onTurnEnd() override;
    void onError(const std::string& error) override;

private:
    // Sprayer-specific methods
    bool checkWeatherConditions();
    bool checkGPSAccuracy();
    bool checkBufferZones();
    bool checkChemicalLevel();
    bool isInRestrictedArea();
    
    // Application rate calculations
    double calculateFlowRate(double ground_speed);
    bool adjustApplicationRate(double target_rate);
    bool calibrateNozzles();
    
    // Safety checks
    bool validateSprayConditions();
    bool checkWindSpeed();
    bool checkTemperature();
    bool checkHumidity();
    
    // Turn sequence management
    bool liftBoomForTurn();
    bool lowerBoomForWork();
    bool performHeadlandTurn();
    
    // Member variables
    SprayerMissionConfig sprayer_config_;
    
    // Environmental monitoring
    double current_wind_speed_ = 0.0;
    double current_temperature_ = 20.0;
    double current_humidity_ = 50.0;
    double gps_accuracy_ = 0.0;
    
    // Application tracking
    double total_chemical_applied_ = 0.0;  // Liters
    double current_application_rate_ = 0.0; // L/ha
    double tank_level_ = 100.0;            // Percentage
    
    // State tracking
    bool boom_lowered_ = false;
    bool spray_active_ = false;
    bool in_turn_sequence_ = false;
    std::chrono::steady_clock::time_point turn_start_time_;
    std::chrono::steady_clock::time_point spray_start_time_;
    
    // Required implements
    static const std::vector<std::string> REQUIRED_IMPLEMENTS;
    
    // Constants
    static constexpr double TURN_SEQUENCE_TIMEOUT_SEC = 30.0;
    static constexpr double MIN_TANK_LEVEL = 5.0; // Percentage
    static constexpr double MAX_APPLICATION_RATE = 500.0; // L/ha
};

} // namespace system_controller

#endif // SPRAYER_MISSION_HPP 