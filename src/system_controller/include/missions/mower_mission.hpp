#ifndef MOWER_MISSION_HPP
#define MOWER_MISSION_HPP

#include "mission_base.hpp"
#include <chrono>
#include <vector>

namespace system_controller {

/**
 * @brief Mower-specific configuration parameters
 */
struct MowerMissionConfig {
    double cutting_height = 0.05;              // meters (5cm default)
    double cutting_speed = 2.5;                // m/s ground speed
    uint8_t cutting_pattern = 0;               // 0=stripe, 1=spiral, 2=random
    double overlap_percentage = 10.0;          // % overlap between passes
    bool mulching_mode = true;                 // Enable mulching
    bool side_discharge = false;               // Side discharge vs collection
    double max_slope_angle = 15.0;             // degrees maximum safe slope
    bool weather_dependent = true;             // Stop in wet conditions
    double grass_height_threshold = 0.15;     // meters - max grass height
    std::vector<std::string> no_mow_zones;    // Areas to avoid
};

/**
 * @brief Mower mission for grass cutting operations
 * 
 * This mission implements the business logic for mowing operations including:
 * - Cutting height and speed optimization
 * - Pattern-based mowing (stripe, spiral, random)
 * - Weather condition monitoring
 * - Slope safety management
 * - Turn sequence optimization (raise deck at end of row, lower at start)
 * - Grass condition assessment
 */
class MowerMission : public MissionBase {
public:
    MowerMission();
    ~MowerMission() override = default;

    // MissionBase interface implementation
    bool initialize(const MissionConfig& config) override;
    bool start() override;
    bool pause() override;
    bool resume() override;
    bool stop() override;
    bool abort() override;
    
    void update() override;
    MissionStatus getStatus() const override;

    // Mower-specific business logic
    bool performPreMowChecks();
    bool checkMowingConditions();
    bool checkGrassConditions();
    bool checkTerrainSafety();
    
    // Deck and blade control
    bool lowerDeck();
    bool raiseDeck();
    bool engageBlades();
    bool disengageBlades();
    
    // Pattern execution
    void executeStripeMowing();
    void executeSpiralMowing();
    void executeRandomMowing();
    
    // Turn sequence management
    void handleTurnSequence();
    
    // Quality monitoring
    void monitorCuttingQuality();
    void adjustCuttingParameters();
    
    // Progress tracking
    void updateProgress();

private:
    MowerMissionConfig mower_config_;
    
    // Environmental monitoring
    struct WeatherData {
        double temperature = 20.0;      // °C
        double humidity = 60.0;         // %
        double precipitation = 0.0;     // mm/hr
        bool ground_wet = false;
    } weather_data_;
    
    // Terrain monitoring
    struct TerrainData {
        double current_slope = 0.0;     // degrees
        double surface_roughness = 0.0; // relative scale
        bool obstacles_detected = false;
    } terrain_data_;
    
    // Operational state
    struct OperationalState {
        bool blades_engaged = false;
        bool deck_lowered = false;
        uint8_t current_pattern = 0;
        uint32_t current_pass = 0;
        double area_mowed = 0.0;        // hectares
        double cutting_time = 0.0;      // hours
        bool turn_in_progress = false;
        double blade_wear = 100.0;      // % sharpness
    } operational_state_;
    
    // Grass assessment
    struct GrassData {
        double height = 0.08;           // meters
        double density = 80.0;          // % coverage
        double moisture = 30.0;         // % moisture content
        bool growth_stage_ok = true;
    } grass_data_;
    
    // Utility methods
    void updateWeatherData();
    void updateTerrainData();
    void updateGrassData();
    void updateOperationalData();
    
    // Safety and condition checks
    bool checkSafetyConditions();
    bool approachingEndOfRow() const;
    bool startOfNewPass() const;
    bool detectObstacles() const;
    bool inNoMowZone() const;
    
    // Simulation helpers
    double getBladeSharpness() const;
    double getDeckHeight() const;
    bool checkBladeDamage() const;
    void simulateCutting();
};

} // namespace system_controller

#endif // MOWER_MISSION_HPP 