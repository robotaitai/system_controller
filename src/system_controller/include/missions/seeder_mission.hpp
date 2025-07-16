#ifndef SEEDER_MISSION_HPP
#define SEEDER_MISSION_HPP

#include "mission_base.hpp"
#include <chrono>
#include <vector>
#include <map>

namespace system_controller {

/**
 * @brief Seeder-specific configuration parameters
 */
struct SeederMissionConfig {
    std::string seed_type = "corn";           // Type of seed being planted
    double seed_rate = 80000.0;               // seeds/ha target population
    double planting_depth = 0.025;            // meters (2.5cm default)
    double row_spacing = 0.76;                // meters (30" rows)
    double planting_speed = 3.0;              // m/s ground speed
    bool variable_rate_seeding = true;        // Enable VRS
    double soil_moisture_min = 20.0;          // % minimum soil moisture
    double soil_temperature_min = 10.0;       // °C minimum soil temperature
    bool fertilizer_application = true;       // Apply starter fertilizer
    double fertilizer_rate = 50.0;            // kg/ha fertilizer rate
    std::string fertilizer_type = "starter";
    std::vector<std::string> no_seed_zones;   // Areas to avoid seeding
};

/**
 * @brief Seeder mission for precision planting operations
 * 
 * This mission implements the business logic for seeding operations including:
 * - Seed rate optimization based on field conditions
 * - Soil condition monitoring (moisture, temperature)
 * - Variable rate seeding based on field zones
 * - Fertilizer application control
 * - Turn sequence optimization (lift seeder at end of row, lower at start)
 * - Population target management
 * - Emergence monitoring and feedback
 */
class SeederMission : public MissionBase {
public:
    SeederMission();
    ~SeederMission() override = default;

    // MissionBase interface implementation
    bool initialize(const MissionConfig& config) override;
    bool start() override;
    bool pause() override;
    bool resume() override;
    bool stop() override;
    bool abort() override;
    
    void update() override;
    MissionStatus getStatus() const override;

    // Seeder-specific business logic
    bool performPreSeedChecks();
    bool checkSoilConditions();
    bool checkSeedConditions();
    bool checkEquipmentStatus();
    
    // Seeder control
    bool lowerSeeder();
    bool raiseSeeder();
    bool enableMetering();
    bool disableMetering();
    bool startFertilizerApplication();
    bool stopFertilizerApplication();
    
    // Variable rate seeding
    void calculateSeedingRate();
    void adjustSeedingParameters();
    void updateFieldZone();
    
    // Turn sequence management
    void handleTurnSequence();
    
    // Quality monitoring
    void monitorSeedingQuality();
    void checkEmergence();
    void validatePopulation();
    
    // Progress tracking
    void updateProgress();

private:
    SeederMissionConfig seeder_config_;
    
    // Soil monitoring
    struct SoilData {
        double moisture = 25.0;         // % moisture content
        double temperature = 12.0;      // °C soil temperature
        double ph = 6.8;               // soil pH
        double organic_matter = 3.5;   // % organic matter
        bool compaction_detected = false;
    } soil_data_;
    
    // Field zone mapping
    struct FieldZone {
        std::string zone_id = "zone_1";
        double productivity_index = 85.0;  // % relative productivity
        double recommended_population = 80000; // seeds/ha
        double soil_quality = 8.0;        // 1-10 scale
        bool requires_tillage = false;
    } current_zone_;
    
    // Operational state
    struct OperationalState {
        bool metering_active = false;
        bool seeder_lowered = false;
        bool fertilizer_active = false;
        uint32_t current_row = 0;
        double area_seeded = 0.0;         // hectares
        double seeds_planted = 0.0;       // total count
        double fertilizer_applied = 0.0;  // kg
        bool turn_in_progress = false;
        double seed_level = 100.0;        // % tank level
        double fertilizer_level = 100.0;  // % tank level
    } operational_state_;
    
    // Seeding quality metrics
    struct QualityMetrics {
        double spacing_accuracy = 95.0;   // % correct spacing
        double depth_accuracy = 90.0;     // % correct depth
        double emergence_rate = 0.0;      // % emerged (for post-planting)
        double population_achieved = 0.0; // actual plants/ha
        double singulation_rate = 98.0;   // % single seeds (no doubles/skips)
    } quality_metrics_;
    
    // Equipment status
    struct EquipmentStatus {
        bool seed_meter_ok = true;
        bool fertilizer_meter_ok = true;
        bool depth_control_ok = true;
        bool down_pressure_ok = true;
        double meter_wear = 5.0;          // % wear
    } equipment_status_;
    
    // Utility methods
    void updateSoilData();
    void updateFieldZone();
    void updateOperationalData();
    void updateQualityMetrics();
    
    // Safety and condition checks
    bool checkSafetyConditions();
    bool approachingEndOfRow() const;
    bool startOfNewRow() const;
    bool inNoSeedZone() const;
    bool checkSeedFlow() const;
    
    // Variable rate calculations
    double calculateOptimalPopulation();
    double adjustForSoilConditions();
    double adjustForFieldHistory();
    
    // Simulation helpers
    double getSeedLevel() const;
    double getFertilizerLevel() const;
    bool detectSeedBlockage() const;
    void simulatePlanting();
};

} // namespace system_controller

#endif // SEEDER_MISSION_HPP 